/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_median_gray8.c
 * Description:  Gray8 Median filter for CMSIS-CV
 *
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2014 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cv/non_linear_filters.h"

/* numbers of parallel histograms handled by the Helium variant */
#define NB_MVE_HISTOG 16

/* 4-bit radix sort histogram size (2^rdx) */
#define RDXSORT_HISTOG_SZ 16

/**
 * @brief       Return the scratch size for median filter function, given a kernel dimension.
 *
 * @param[in]   kernDim     The median filter square kernel dimension. Must be an odd value.
 * @return      The scratch size in bytes
 */

uint16_t arm_median_filter_gray8_get_scratch_size(uint8_t kernDim)
{
#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)
    return ((kernDim * kernDim) + RDXSORT_HISTOG_SZ) * NB_MVE_HISTOG;
#else
    return (kernDim * kernDim) + RDXSORT_HISTOG_SZ;
#endif
}

/* nibble extraction helpers (applicable for scalar & vector)*/
#define GET_LOW_NIB(val) ((val) & 0x0f)
#define GET_HIGH_NIB(val) (((val) >> 4) & 0xf)

/*
 * image border padding methods selection:
 * can choose between constant or nearest/replicate
 * (PAD_BORDER_CST / PAD_BORDER_NEAREST)
 * Default methos is contant as less costly
 */

#define PAD_BORDER_CST_VAL 0

#define PAD_BORDER PAD_BORDER_CST

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)

/*
 * Pad with border replication
 * row padding requires the duplication of an vector of row border
 * and need to be mixed with an vector of extended column border
 */
#define PAD_BORDER_NEAREST(curCol, row, cols, imgcol, imgrow, arr, kernDim, cst)                             \
    mve_pred16_t p    = mve_pred_mask_border(imgrow, imgcol, row, cols);                                     \
    uint8x16_t in     = vldrbq_z_u8(curCol, p);                                                              \
    int bordRowIdx    = (row < 0) ? 0 : (row >= imgrow) ? imgrow - 1 : row;                                  \
    int bordColIdx    = (cols < 0) ? 0 : (cols + kernDim + 16 >= imgcol) ? imgcol - 1 : cols;                \
    int outOfBoundRow = (row < 0 || row >= imgrow) ? 1 : 0;                                                  \
                                                                                                             \
    /* duplicate column border */                                                                            \
    uint8x16_t vfill = vdupq_n_u8(arr[bordRowIdx * imgcol + bordColIdx]);                                    \
    if (outOfBoundRow)                                                                                       \
    {                                                                                                        \
        /* build predicate for row border with exclusion of out of bound columns */                          \
        mve_pred16_t p1 = vctp8q(imgcol - cols);                                                             \
        p1 &= (cols < 0) ? -(1 << (-cols)) : 0xFFFF;                                                         \
        /* load row border */                                                                                \
        uint8x16_t vfill1 = vldrbq_z_u8(&arr[bordRowIdx * imgcol + cols], p1);                               \
        /* mix out of bound column / row */                                                                  \
        vfill = vorrq_m_u8(vfill1, vfill1, vfill, vpnot(p1));                                                \
    }                                                                                                        \
    /* mix pad and input */                                                                                  \
    in = vorrq_m_u8(in, vfill, in, vpnot(p));

#define PAD_BORDER_CST_ANY(curCol, row, cols, imgcol, imgrow, arr, kernDim, cst)                             \
    mve_pred16_t p   = mve_pred_mask_border(imgrow, imgcol, row, cols);                                      \
    uint8x16_t in    = vldrbq_z_u8(curCol, p);                                                               \
    uint8x16_t vfill = vdupq_n_u8(cst);                                                                      \
    /* mix pad and input */                                                                                  \
    in = vorrq_m_u8(in, vfill, in, vpnot(p));

#define PAD_BORDER_CST_0(curCol, row, cols, imgcol, imgrow, arr, kernDim, cst)                               \
    mve_pred16_t p = mve_pred_mask_border(imgrow, imgcol, row, cols);                                        \
    /* zero-predication is filling with 0 */                                                                 \
    uint8x16_t in = vldrbq_z_u8(curCol, p);

#if PAD_BORDER_CST_VAL == 0
#define PAD_BORDER_CST(curCol, row, cols, imgcol, imgrow, arr, kernDim, cst)                                 \
    PAD_BORDER_CST_0(curCol, row, cols, imgcol, imgrow, arr, kernDim, _)
#else
#define PAD_BORDER_CST(curCol, row, cols, imgcol, imgrow, arr, kernDim, cst)                                 \
    PAD_BORDER_CST_ANY(curCol, row, cols, imgcol, imgrow, arr, kernDim, cst)
#endif

/* normal vector of pixel load for the no-border cases */
#define GET_ARR_VAL(curCol, row, cols, imgcol, imgrow, arr, kernDim, cst) uint8x16_t in = vldrbq_u8(curCol);

/* pixel extraction with border handling and parallel sorting based on low nibble */
#define RDXSORT_LOW_NIB(BORDDER_HDLR, BORDER_CST_VAL, IMGROW)                                                \
    int startCol          = curColIdx - kernDim / 2;                                                         \
    int startRow          = curRowIdx - kernDim / 2;                                                         \
    uint16_t kern_elts    = kernDim * kernDim;                                                               \
    uint8x16_t stride16_8 = NB_MVE_HISTOG * vidupq_n_u8(0, 1);                                               \
    const uint8_t *pStart = arr + startCol + imgcol * startRow;                                              \
    uint8_t *histogr      = (uint8_t *)scratch;                                                              \
                                                                                                             \
    for (int i = 0; i < RDXSORT_HISTOG_SZ * 2; i++)                                                          \
        scratch[i] = 0;                                                                                      \
                                                                                                             \
    const uint8_t *curRow = pStart;                                                                          \
    for (int row = startRow; row < startRow + kernDim; row++)                                                \
    {                                                                                                        \
        const uint8_t *curCol = curRow;                                                                      \
        for (int cols = startCol; cols < kernDim + startCol; cols++)                                         \
        {                                                                                                    \
            BORDDER_HDLR(curCol, row, cols, imgcol, IMGROW, arr, kernDim, BORDER_CST_VAL);                   \
            curCol += 1;                                                                                     \
                                                                                                             \
            uint8x16_t ofs = GET_LOW_NIB(in);                                                                \
            ofs += stride16_8;                                                                               \
                                                                                                             \
            update_histogrs(histogr, ofs);                                                                   \
        }                                                                                                    \
        /* move to next line */                                                                              \
        curRow += imgcol;                                                                                    \
    }                                                                                                        \
                                                                                                             \
    accum_histogrs(histogr);                                                                                 \
                                                                                                             \
    curRow = pStart;                                                                                         \
    for (int row = startRow; row < startRow + kernDim; row++)                                                \
    {                                                                                                        \
        const uint8_t *curCol = curRow;                                                                      \
        for (int cols = startCol; cols < kernDim + startCol; cols++)                                         \
        {                                                                                                    \
            BORDDER_HDLR(curCol, row, cols, imgcol, IMGROW, arr, kernDim, BORDER_CST_VAL);                   \
            curCol += 1;                                                                                     \
                                                                                                             \
            /* convert inputs into histogram offsets */                                                      \
            uint8x16_t ofsHist = GET_LOW_NIB(in) + stride16_8;                                               \
            /* get values from histograms */                                                                 \
            uint8x16_t cnt = vldrbq_gather_offset_u8(histogr, ofsHist);                                      \
                                                                                                             \
            /* widen 16 histogram index into location in the sorting buffer */                               \
            vstrbq_scatter_ofs_strideN(output, cnt, in, kern_elts);                                          \
            cnt -= 1;                                                                                        \
            vstrbq_scatter_offset_u8(histogr, ofsHist, cnt);                                                 \
        }                                                                                                    \
        /* move to next line */                                                                              \
        curRow += imgcol;                                                                                    \
    }

/* Vector gather load with stride N */
static uint8x16_t vldrbq_gather_ofs_strideN(uint8_t const *parr, uint16_t n)
{
    if (n == 9)
        /* offsets do not exceed 16x9 and fit into vector of bytes */
        /* efficient compilers like AC6/LLVM will create 2 separated loops for arm_sort_high_nib_mve */
        /* and remove comparison */
        return vldrbq_gather_offset_u8(parr, vidupq_n_u8(0, 1) * (uint8_t)n);
    else
    {
        /* 16-bit intermediate load index as N x 16 will exceed byte size when kern size >= 5 */
        uint16x8_t strideN0 = vmulq(vidupq_n_u16(0, 2), (n));
        uint16x8_t strideN1 = vmulq(vidupq_n_u16(1, 2), (n));

        /* load elements placed at index [ 0, 2n, 4n, 6n, 8n, 10n, 12n, 14n ] */
        uint16x8_t in0 = vldrbq_gather_offset_u16(parr, strideN0);
        /* load elements placed at index [ n, 3n, 5n, 7n, 9n, 11n, 13n, 15n ] */
        uint16x8_t in1 = vldrbq_gather_offset_u16(parr, strideN1);
        uint8x16_t in;

        /* merge and narrow elements from [ 0, n, 2n, 3n, 4n, ..., 15n ] */
        in = vmovnbq_u16(vuninitializedq_u8(), in0);
        in = vmovntq_u16(in, in1);

        return in;
    }
}

/* Vector scatter store with stride N */
static void vstrbq_scatter_ofs_strideN(uint8_t *output, uint8x16_t ofs, uint8x16_t in, uint16_t n)
{
    if (n == 9)
    {
        /* offsets do not exceed 16x9 and fit into vector of bytes */
        uint8x16_t ofsOut = ofs - 1 + (vidupq_n_u8(0, 1) * (uint8_t)n);
        vstrbq_scatter_offset_u8(output, ofsOut, in);
    }
    else
    {
        uint16x8_t strideN0 = vmulq(vidupq_n_u16(0, 2), (n));
        uint16x8_t strideN1 = vmulq(vidupq_n_u16(1, 2), (n));

        /*
         * generates
         * [ofs0, ofs2, ofs4, ofs6, ofs8, ofs10, ofs12, ofs14]
         * and
         * [ofs1, ofs3, ofs5, ofs7, ofs9, ofs11, ofs13, ofs15]
         */
        uint16x8_t ofsb = vmovlbq(ofs);
        uint16x8_t ofst = vmovltq(ofs);

        /*
         * generates
         * [ofs0 + 0n - 1, ofs2 + 2n -1, ofs4 + 4n - 1...]
         * [ofs1 + 1n - 1, ofs3 + 3n -1, ofs5 + 5n - 1...]
         */
        uint16x8_t ofsOutb = ofsb + (strideN0 - 1);
        uint16x8_t ofsOutt = ofst + (strideN1 - 1);

        /*
         * generates
         * [in0, in2, in4, in6, in8, in10, in12, in14]
         * and
         * [in1, in3, in5, in7, in9, in11, in13, in15]
         */
        uint16x8_t inb = vmovlbq(in);
        uint16x8_t in1 = vmovltq(in);

        /*
         * store
         * in0 => output[ofs0 + 0n - 1]
         * in2 => output[ofs2 + 2n - 1]
         * ...
         */
        vstrbq_scatter_offset_u16(output, ofsOutb, inb);
        /*
         * store
         * in1 => output[ofs1 + 1n - 1]
         * in3 => output[ofs3 + 3n - 1]
         * ...
         */
        vstrbq_scatter_offset_u16(output, ofsOutt, in1);
    }
}

/* get predication mask to disable border elements given image dimension and current vector of pixel row/col
 * coordinates */
static uint16_t mve_pred_mask_border(int imgrow, int imgcol, int curR, int curC)
{
    /* right border limitation */
    uint16_t pred = vctp8q(imgcol - curC);

    /* out of bound rows */
    if ((curR < 0) || (curR >= imgrow) || ((imgcol - curC) < 0))
        pred = 0;
    /* left border limitation */
    else if (curC < 0)
        /* inactivate lanes between -curC and 0 */
        /* if curC = -3, lane 0, 1, 2 need to be cleared, predicate = -2^3 = 0xfff8 */
        pred &= -(1 << (-curC));

    return pred;
}

/* increment 16 parallel histogram bins */
static void update_histogrs(uint8_t *histogr, uint8x16_t ofs)
{
    uint8x16_t cnt = vldrbq_gather_offset_u8(histogr, ofs);
    cnt += 1;
    vstrbq_scatter_offset_u8(histogr, ofs, cnt);
}

/* 16 parallel cumulative histogram conversion */
static void accum_histogrs(uint8_t *histogr)
{
    uint8x16_t cur = NB_MVE_HISTOG * vidupq_n_u8(0, 1);
    uint8x16_t cnt = vldrbq_gather_offset_u8(histogr, cur);

    for (int i = 1; i < RDXSORT_HISTOG_SZ; i++)
    {
        cur += 1;
        uint8x16_t cnt1 = vldrbq_gather_offset_u8(histogr, cur);
        /* accumulate */
        cnt1 += cnt;
        vstrbq_scatter_offset_u8(histogr, cur, cnt1);
        cnt = cnt1;
    }
}

/*
 * high nibble radix sorting
 * input is composed of 16 adjacent low nibble sorted arrays
 * output is a vector of 16 medians
 */
static uint8x16_t arm_sort_high_nib_mve(uint8_t *arr, uint16_t kernDim, uint64_t *scratch)
{

    uint16_t n       = kernDim * kernDim;
    uint8_t medIdx   = (n + 1) / 2; /* median index */
    uint8_t *histogr = (uint8_t *)scratch;

    /* clear 16 parallel histograms */
    for (int i = 0; i < RDXSORT_HISTOG_SZ * 2; i++)
        scratch[i] = 0;

    uint8x16_t stride16_8 = NB_MVE_HISTOG * vidupq_n_u8(0, 1);

    uint8_t *parr         = arr;
    for (int i = 0; i < n; i++)
    {
        uint8x16_t ofs = vldrbq_gather_ofs_strideN(parr, n);
        parr += 1;

        ofs = GET_HIGH_NIB(ofs);
        ofs += stride16_8;

        update_histogrs(histogr, ofs);
    }

    accum_histogrs(histogr);

    parr            = &arr[n - 1];
    uint8x16_t medV = vdupq_n_u8(0); /* vector of medians */

    for (int i = 0; i < n; i++)
    {
        uint8x16_t in = vldrbq_gather_ofs_strideN(parr, n);
        parr -= 1;

        uint8x16_t ofs = GET_HIGH_NIB(in) + stride16_8;
        uint8x16_t cnt = vldrbq_gather_offset_u8(histogr, ofs);

        /* capture medians */
        medV = vorrq_x(medV, in, vcmpeqq_n_u8(cnt, medIdx));
        cnt -= 1;
        vstrbq_scatter_offset_u8(histogr, ofs, cnt);
    }
    return medV;
}

/*
 * low nibble radix sorting
 * extract pixels for 16 adjacent kernels (stride =1) with border handling
 * generate 16 adjacent low nibble sorted arrays (scratch)
 */
void arm_extract_sort_low_nib_border_mve(uint8_t *output, const arm_cv_image_gray8_t *pImageIn, int kernDim,
                                         int curRowIdx, int curColIdx, uint64_t *scratch)
{
    int imgcol   = pImageIn->width;
    int imgrow   = pImageIn->height;
    uint8_t *arr = pImageIn->pData;

    RDXSORT_LOW_NIB(PAD_BORDER, PAD_BORDER_CST_VAL, imgrow);
}

/*
 * low nibble radix sorting
 * same as before but faster, for elements away from borders
 */
static void arm_extract_sort_low_nib_mve(uint8_t *output, const arm_cv_image_gray8_t *pImageIn, int kernDim,
                                         int curRowIdx, int curColIdx, uint64_t *scratch)
{
    int imgcol   = pImageIn->width;
    uint8_t *arr = pImageIn->pData;

    RDXSORT_LOW_NIB(GET_ARR_VAL, _, _);
}

/* batch median extraction for 16 pixels */
static uint8x16_t arm_get_median_mve(const arm_cv_image_gray8_t *pImageIn, int kernDim, int curRowIdx,
                                     int curColIdx, uint32_t borderCare, uint64_t *scratch)
{
    /* kernDim^2 scratch area containing low-nibble-sorted pixels */
    uint8_t *sortedTmp = (uint8_t *)scratch + (NB_MVE_HISTOG * RDXSORT_HISTOG_SZ);

    if (borderCare)
        arm_extract_sort_low_nib_border_mve(sortedTmp, pImageIn, kernDim, curRowIdx, curColIdx, scratch);
    else
        arm_extract_sort_low_nib_mve(sortedTmp, pImageIn, kernDim, curRowIdx, curColIdx, scratch);

    return arm_sort_high_nib_mve(sortedTmp, kernDim, scratch);
}

#else

#define CLAMP_BORDER(m, n, imgcol, imgrow)                                                                   \
    (((m) < 0 ? 0 : ((m) >= (imgrow) ? (imgrow) - 1 : (m))) * (imgcol) +                                     \
     ((n) < 0 ? 0 : ((n) >= (imgcol) ? (imgcol) - 1 : (n))))

/*
 * Pad with border replication
 */
#define PAD_BORDER_NEAREST(m, n, imgcol, imgrow, arr, _) (arr[CLAMP_BORDER((m), (n), (imgcol), (imgrow))])

/*
 * Pad with constant value
 */
#define PAD_BORDER_CST(m, n, imgcol, imgrow, arr, cst)                                                       \
    (((m) < 0 || (n) < 0 || (m) >= (imgrow) || (n) >= (imgcol)) ? cst : (arr[(m) * (imgcol) + (n)]))

#define GET_ARR_VAL(m, n, imgcol, imgrow, arr, cst) (arr[(m) * imgcol + (n)])

/* cumulative (single) histogram conversion */
#define ACCUM_HISTOGR(histogr)                                                                               \
    for (int i = 1; i < RDXSORT_HISTOG_SZ; i++)                                                              \
        histogr[i] += histogr[i - 1];

/* pixel extraction with border handling & sorting based on low nibble */
#define RDXSORT_LOW_NIB(BORDDER_HDLR, BORDER_CST_VAL, IMGROW)                                                \
    int pad_size     = kernDim / 2;                                                                          \
    uint8_t *histogr = (uint8_t *)scratch; /* clear histogram (16 bins) */                                   \
    scratch[0]       = 0;                                                                                    \
    scratch[1]       = 0;                                                                                    \
                                                                                                             \
    for (int patchRowIdx = -pad_size; patchRowIdx <= pad_size; patchRowIdx++)                                \
    {                                                                                                        \
        for (int patchColIdx = -pad_size; patchColIdx <= pad_size; patchColIdx++)                            \
        {                                                                                                    \
            uint8_t val = BORDDER_HDLR(rowIdx + patchRowIdx, colIdx + patchColIdx, imgcol, IMGROW, arr,      \
                                       BORDER_CST_VAL);                                                      \
                                                                                                             \
            histogr[GET_LOW_NIB(val)]++;                                                                     \
        }                                                                                                    \
    }                                                                                                        \
                                                                                                             \
    ACCUM_HISTOGR(histogr);                                                                                  \
                                                                                                             \
    for (int patchRowIdx = -pad_size; patchRowIdx <= pad_size; patchRowIdx++)                                \
    {                                                                                                        \
        for (int patchColIdx = -pad_size; patchColIdx <= pad_size; patchColIdx++)                            \
        {                                                                                                    \
            uint8_t val = BORDDER_HDLR(rowIdx + patchRowIdx, colIdx + patchColIdx, imgcol, IMGROW, arr,      \
                                       BORDER_CST_VAL);                                                      \
                                                                                                             \
            pOutput[histogr[GET_LOW_NIB(val)] - 1] = val;                                                    \
            histogr[GET_LOW_NIB(val)]--;                                                                     \
        }                                                                                                    \
    }

/*
 * low nibble radix sorting
 * extract pixelswith border handling
 * generate low nibble sorted arrays (stored in scratch)
 */
static void arm_extract_sort_low_nib_border(uint8_t pOutput[], const arm_cv_image_gray8_t *pImageIn,
                                            int kernDim, int rowIdx, int colIdx, uint64_t *scratch)
{
    uint8_t *arr = pImageIn->pData;
    int imgcol   = pImageIn->width;
    int imgrow   = pImageIn->height;

    RDXSORT_LOW_NIB(PAD_BORDER, PAD_BORDER_CST_VAL, imgrow);
}

static void arm_extract_sort_low_nib(uint8_t pOutput[], const arm_cv_image_gray8_t *pImageIn, int kernDim,
                                     int rowIdx, int colIdx, uint64_t *scratch)
{
    uint8_t *arr = pImageIn->pData;
    int imgcol   = pImageIn->width;

    RDXSORT_LOW_NIB(GET_ARR_VAL, _, _);
}

/*
 * high nibble radix sorting
 * output is kernel median
 */
static uint8_t arm_sort_high_nib(uint8_t *arr, uint16_t kernDim, uint64_t *scratch)
{
    uint8_t *histogr = (uint8_t *)scratch;
    uint16_t n       = kernDim * kernDim;
    uint8_t medIdx   = (n + 1) / 2;

    /* clear histogram (2 x 64-bit or 16 x 8-bit bins) */
    scratch[0] = 0;
    scratch[1] = 0;

    for (int i = 0; i < n; i++)
    {
        histogr[GET_HIGH_NIB(arr[i])]++;
    }

    ACCUM_HISTOGR(histogr);

    for (int i = n - 1; i >= 0; i--)
    {
        /* return when histogram index matches median index */
        if ((histogr[GET_HIGH_NIB(arr[i])]) == medIdx)
            return arr[i];

        histogr[GET_HIGH_NIB(arr[i])]--;
    }

    return 0;
}

static uint8_t arm_get_median(const arm_cv_image_gray8_t *pImageIn, int kernDim, int curRowIdx, int curColIdx,
                              uint32_t borderCare, uint64_t *scratch)
{
    /* kernDim^2 scratch area containing low-nibble-sorted pixels */
    uint8_t *sortedTmp = (uint8_t *)scratch + RDXSORT_HISTOG_SZ;

    if (borderCare)
    {
        arm_extract_sort_low_nib_border(sortedTmp, pImageIn, kernDim, curRowIdx, curColIdx, scratch);
    }
    else
        arm_extract_sort_low_nib(sortedTmp, pImageIn, kernDim, curRowIdx, curColIdx, scratch);

    return arm_sort_high_nib(sortedTmp, kernDim, scratch);
}

#endif

/**
 * @brief       median filter
 *
 * @param[in]   pImageIn        : input image
 * @param[in]   pImageOut       : output filtered image
 * @param[in]   kernDim         : The median filter square kernel dimension. Must be an odd value.
 * @param[in]   scratch         : 64-bit aligned scratch area. Size given by
 * arm_median_filter_gray8_get_scratch_size
 * @return      filtering status
 */

arm_cv_status arm_median_filter_gray8(const arm_cv_image_gray8_t *pImageIn, arm_cv_image_gray8_t *pImageOut,
                                      uint8_t kernDim, uint64_t *scratch)
{
    /* kernel size needs to be odd */
    if ((kernDim % 2) == 0)
        return ARM_CV_ARGUMENT_ERROR;

#if defined(ARM_MATH_MVEI) && !defined(ARM_MATH_AUTOVECTORIZE)
    uint8_t *imgOut = pImageOut->pData;
    for (int curRowIdx = 0; curRowIdx < pImageIn->height; curRowIdx++)
    {
        uint8_t *curOut = imgOut;

        /* Flag to track the presence of horizontal borders. This enables optimization by skipping horiz+vert
         * borders handling when not required. */
        uint32_t borderCareHoriz = 0;

        if (curRowIdx < kernDim / 2 || (curRowIdx >= (pImageIn->height - (kernDim / 2))))
            borderCareHoriz = 1;

        for (int curColIdx = 0; curColIdx < pImageIn->width; curColIdx += 16)
        {
            uint32_t borderCare = borderCareHoriz;
            /* presence of vertical borders */
            if (curColIdx < kernDim / 2 || (curColIdx >= pImageIn->width - (kernDim / 2 + 16)))
                borderCare |= 1;

            uint8x16_t medianV =
                arm_get_median_mve(pImageIn, kernDim, curRowIdx, curColIdx, borderCare, scratch);

            vst1q_p(curOut, medianV, vctp8q(pImageIn->width - curColIdx));
            curOut += 16;
        }
        imgOut += pImageIn->width;
    }
#else
    for (int curRowIdx = 0; curRowIdx < pImageIn->height; curRowIdx++)
    {
        /* Flag to track the presence of horizontal borders. This enables optimization by skipping horiz+vert
         * borders handling when not required. */
        uint32_t borderCareHoriz = 0;

        if (curRowIdx < kernDim / 2 || (curRowIdx >= (pImageIn->height - (kernDim / 2))))
            borderCareHoriz = 1;

        for (int curColIdx = 0; curColIdx < pImageIn->width; curColIdx++)
        {
            uint32_t borderCare = borderCareHoriz;
            /* presence of vertical borders */
            if (curColIdx < kernDim / 2 || (curColIdx >= pImageIn->width - (kernDim / 2)))
                borderCare |= 1;

            pImageOut->pData[curRowIdx * pImageIn->width + curColIdx] =
                arm_get_median(pImageIn, kernDim, curRowIdx, curColIdx, borderCare, scratch);
        }
    }
#endif
    return ARM_CV_SUCCESS;
}
