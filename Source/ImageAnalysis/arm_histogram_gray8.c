/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_histogram_gray8.c
 * Description:  Gray8 histogram
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

#include "cv/feature_detection.h"
#include "dsp/basic_math_functions.h"
#include "dsp/fast_math_functions.h"
#include "cv/image_analysis.h"
#include "arm_common_tables.h"




static
void arm_histogr_core_u8(const channel_uint8_t * pIn, uint16_t * pHistog, uint32_t size)
{

    for (uint32_t i = 0; i < size; i++)
        pHistog[*pIn++] += 1;
}

static
void arm_histogr_core_u8_mask(const channel_uint8_t * pIn, const channel_uint8_t * pMask,
                              uint16_t * pHistog, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        channel_uint8_t in = *pIn++;
        channel_uint8_t mask = *pMask++;

        if (mask)
            pHistog[in] += 1;
    }
}



#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))


/*
 In the Helium implementation, the construction of histograms utilizes 4 sub-histograms to distribute indentical indices
 evenly and prevent overlap.
 4 x 8-bit values are combined into a single 32-bit vector, which serves as a scatter/gather offset.
 This is then expanded into offsets for the 4 adjacent sub-histograms, ensuring that identical values are considered.
 For instance, if the input sequence includes values like [1, 1, 2, 2, …], a standard gather load, increment, scatter store process
 would lead to data loss.
 This happens because the results of the additions would be stored in the same location due to their non-uniqueness.

 For better performance, it would have been possible to use 16-bit vector as well and 8 sub-histograms, which comes with
 increased memory needs and 16-bit scatter/gather cost would offset the benefits

 The scratch size required for holding the 4 sub-histogram is given by arm_histogram_gray8_get_scratch_size
 */

#define NB_SUB_HISTOGGRAMS 4

__STATIC_FORCEINLINE
    mve_pred16_t vset_predq_bounds_u32_p(uint32x4_t offset, uint32_t min, uint32_t max,
                                         mve_pred16_t p)
{
    p = vcmpcsq_m_n_u32(offset, min, p);
    p = vcmphiq_m_u32(vdupq_n_u32(max), offset, p);
    return p;
}




typedef void    (*_arm_histogr_split_fct) (const channel_uint8_t * pSrc,
                                           const channel_uint8_t * pMask,
                                           const channel_uint8_t * lut, uint16_t min, uint16_t max,
                                           uint16_t nbBins, uint32_t invHistWidth, uint32_t size,
                                           uint16_t * pHistoSplit);


/*
     different sun-histogram split methods:
     - arm_subhistogr_split_basic           : split unbounded 8-bit data into [0-256[ buckets sub-histograms
     - arm_subhistogr_split_basic_mask      : split unbounded 8-bit data after mask validation into [0-256[ buckets sub-histograms
     - arm_subhistogr_split_non_uniform     : split bounded 8-bit data into non-uniform buckets sub-histograms
     - arm_subhistogr_split_non_uniform_mask: split bounded 8-bit data after mask validation into non-uniform buckets sub-histograms
     - arm_subhistogr_split_buckets         : split bounded 8-bit data into custom buckets sub-histograms
     - arm_subhistogr_split_buckets         : split bounded 8-bit data after mask validation into custom buckets sub-histograms

    mask variants are created to avoid slowing down non-maks variant because of mask pointer check
  */


static void arm_subhistogr_split_basic(const channel_uint8_t * pSrc,
                                       const channel_uint8_t * pMask, const channel_uint8_t * lut,
                                       uint16_t min, uint16_t max, uint16_t nbBins,
                                       uint32_t invHistWidth, uint32_t size, uint16_t * pHistoSplit)
{
    (void) lut;
    (void) min;
    (void) max;
    (void) invHistWidth;
    (void) pMask;

    uint32x4_t      histoOffset;
    histoOffset = vidupq_u32((uint32_t) 0, 1);
    /* each lane will be dispatched */
    /* into differents sub-histograms */

    histoOffset = histoOffset * nbBins;

    /* unrolling */
    int32_t         loopCnt = size / 8;

    uint32x4_t      offset0, offset1, hist;

    offset0 = vldrbq_u32(pSrc);
    offset0 = vaddq_u32(offset0, histoOffset);
    pSrc += 4;
    do {

        offset1 = vldrbq_u32(pSrc);
        offset1 = vaddq_u32(offset1, histoOffset);
        pSrc += 4;

        hist = vldrhq_gather_shifted_offset_u32(pHistoSplit, offset0);
        hist = vaddq_n_u32(hist, 1);
        vstrhq_scatter_shifted_offset_u32(pHistoSplit, offset0, hist);

        offset0 = vldrbq_u32(pSrc);
        offset0 = vaddq_u32(offset0, histoOffset);
        pSrc += 4;

        hist = vldrhq_gather_shifted_offset_u32(pHistoSplit, offset1);
        hist = vaddq_n_u32(hist, 1);
        vstrhq_scatter_shifted_offset_u32(pHistoSplit, offset1, hist);

        loopCnt--;
    } while (loopCnt > 0);

    /* residual */
    loopCnt = size % 8;
    if (loopCnt) {
        do {

            mve_pred16_t    p = vctp32q(loopCnt);
            hist = vldrhq_gather_shifted_offset_z_u32(pHistoSplit, offset0, p);
            hist = vaddq_x(hist, 1, p);
            vstrhq_scatter_shifted_offset_p_u32(pHistoSplit, offset0, hist, p);

            offset0 = vldrbq_u32(pSrc);
            offset0 = vaddq_u32(offset0, histoOffset);
            pSrc += 4;
            loopCnt -= 4;
        }
        while (loopCnt > 0);
    }
}





static void arm_subhistogr_split_basic_mask(const channel_uint8_t * pSrc,
                                            const channel_uint8_t * pMask,
                                            const channel_uint8_t * lut, uint16_t min, uint16_t max,
                                            uint16_t nbBins, uint32_t invHistWidth, uint32_t size,
                                            uint16_t * pHistoSplit)
{
    (void) lut;
    (void) min;
    (void) max;
    (void) invHistWidth;

    uint32x4_t      histoOffset;
    histoOffset = vidupq_u32((uint32_t) 0, 1);
    histoOffset = histoOffset * nbBins;
    /* each lane will be dispatched */
    /* into differents sub-histograms */

    int32_t         loopCnt = size / 8;

    uint32x4_t      offset0, offset1, hist, mask;

    offset0 = vldrbq_u32(pSrc);
    offset0 = vaddq_u32(offset0, histoOffset);
    pSrc += 4;
    do {

        mask = vldrbq_u32(pMask);
        pMask += 4;

        offset1 = vldrbq_u32(pSrc);
        offset1 = vaddq_u32(offset1, histoOffset);
        pSrc += 4;

        hist = vldrhq_gather_shifted_offset_u32(pHistoSplit, offset0);
        hist = vaddq_x(hist, 1, vcmpeqq(mask, 1));
        vstrhq_scatter_shifted_offset_u32(pHistoSplit, offset0, hist);

        mask = vldrbq_u32(pMask);
        pMask += 4;

        offset0 = vldrbq_u32(pSrc);
        offset0 = vaddq_u32(offset0, histoOffset);
        pSrc += 4;

        hist = vldrhq_gather_shifted_offset_u32(pHistoSplit, offset1);
        hist = vaddq_x(hist, 1, vcmpeqq(mask, 1));
        vstrhq_scatter_shifted_offset_u32(pHistoSplit, offset1, hist);

        loopCnt--;
    } while (loopCnt > 0);


    /* residual */
    loopCnt = size % 8;
    if (loopCnt) {
        do {

            mve_pred16_t    p = vctp32q(loopCnt);

            mask = vldrbq_u32(pMask);
            pMask += 4;

            hist = vldrhq_gather_shifted_offset_z_u32(pHistoSplit, offset0, p);
            hist = vaddq_x(hist, 1, vcmpeqq(mask, 1));
            vstrhq_scatter_shifted_offset_p_u32(pHistoSplit, offset0, hist, p);

            offset0 = vldrbq_u32(pSrc);
            offset0 = vaddq_u32(offset0, histoOffset);
            pSrc += 4;
            loopCnt -= 4;
        }
        while (loopCnt > 0);
    }
}


static void arm_subhistogr_split_non_uniform(const channel_uint8_t * pSrc,
                                             const channel_uint8_t * pMask,
                                             const channel_uint8_t * lut, uint16_t min,
                                             uint16_t max, uint16_t nbBins, uint32_t invHistWidth,
                                             uint32_t size, uint16_t * pHistoSplit)
{
    (void) pMask;
    (void) invHistWidth;

    /* each lane will be dispatched */
    /* into differents sub-histograms */
    uint32x4_t      histoOffset;
    histoOffset = vidupq_u32((uint32_t) 0, 1);
    histoOffset = histoOffset * nbBins;
    int32_t         loopCnt = size;
    uint32x4_t      offset, hist;


    do {
        mve_pred16_t    p = vctp32q(loopCnt);

        offset = vldrbq_u32(pSrc);

        p = vset_predq_bounds_u32_p(offset, min, max, p);

        offset = vldrbq_gather_offset_z_u32(lut, offset, p);

        offset = vaddq_u32(offset, histoOffset);
        pSrc += 4;


        hist = vldrhq_gather_shifted_offset_z_u32(pHistoSplit, offset, p);
        hist = vaddq_x(hist, 1, p);
        vstrhq_scatter_shifted_offset_p_u32(pHistoSplit, offset, hist, p);

        loopCnt -= 4;
    }
    while (loopCnt > 0);
}


static void arm_subhistogr_split_non_uniform_mask(const channel_uint8_t * pSrc,
                                                  const channel_uint8_t * pMask,
                                                  const channel_uint8_t * lut, uint16_t min,
                                                  uint16_t max, uint16_t nbBins,
                                                  uint32_t invHistWidth, uint32_t size,
                                                  uint16_t * pHistoSplit)
{
    /* each lane will be dispatched */
    /* into differents sub-histograms */
    uint32x4_t      histoOffset;
    histoOffset = vidupq_u32((uint32_t) 0, 1);
    histoOffset = histoOffset * nbBins;
    (void) pMask;
    (void) invHistWidth;

    int32_t         loopCnt = size;
    uint32x4_t      offset, hist;


    do {
        mve_pred16_t    p = vctp32q(loopCnt);

        offset = vldrbq_z_u32(pSrc, p);

        /* set predicate for value between min/max bounds */
        p = vset_predq_bounds_u32_p(offset, min, max, p);

        /* get new bound offsets from LUT */
        offset = vldrbq_gather_offset_z_u32(lut, offset, p);

        /* spread over 4 histograms */
        offset = vaddq_u32(offset, histoOffset);
        pSrc += 4;

        /* load masks */
        uint32x4_t      mask = vldrbq_z_u32(pMask, p);
        pMask += 4;

        /* combine predicate with masks */
        p = vcmpeqq_m(mask, 1, p);

        hist = vldrhq_gather_shifted_offset_z_u32(pHistoSplit, offset, p);
        hist = vaddq_x(hist, 1, p);
        vstrhq_scatter_shifted_offset_p_u32(pHistoSplit, offset, hist, p);

        loopCnt -= 4;

    }
    while (loopCnt > 0);
}


static void arm_subhistogr_split_buckets(const channel_uint8_t * pSrc,
                                         const channel_uint8_t * pMask,
                                         const channel_uint8_t * lut, uint16_t min,
                                         uint16_t max, uint16_t nbBins, uint32_t invHistWidth,
                                         uint32_t size, uint16_t * pHistoSplit)
{
    /* each lane will be dispatched */
    /* into differents sub-histograms */
    uint32x4_t      histoOffset;
    histoOffset = vidupq_u32((uint32_t) 0, 1);
    histoOffset = histoOffset * nbBins;
    (void) pMask;
    (void) lut;

    int32_t         loopCnt = size;
    uint32x4_t      offset, hist;


    do {
        mve_pred16_t    p = vctp32q(loopCnt);

        offset = vldrbq_u32(pSrc);
        pSrc += 4;

        /* filter out of bound + offset with -min */
        p = vset_predq_bounds_u32_p(offset, min, max, p);

        offset = vsubq_x(offset, min, p);

        /* convert values into bucket index */
        offset = offset * nbBins;
        offset = vmulhq_u32(offset, vdupq_n_u32(invHistWidth));

        /* spread over 4 histograms */
        offset = vaddq_u32(offset, histoOffset);


        hist = vldrhq_gather_shifted_offset_z_u32(pHistoSplit, offset, p);
        hist = vaddq_x(hist, 1, p);
        vstrhq_scatter_shifted_offset_p_u32(pHistoSplit, offset, hist, p);

        loopCnt -= 4;
    }
    while (loopCnt > 0);

}


static void arm_subhistogr_split_buckets_mask(const channel_uint8_t * pSrc,
                                              const channel_uint8_t * pMask,
                                              const channel_uint8_t * lut, uint16_t min,
                                              uint16_t max, uint16_t nbBins, uint32_t invHistWidth,
                                              uint32_t size, uint16_t * pHistoSplit)
{
    /* each lane will be dispatched */
    /* into differents sub-histograms */
    uint32x4_t      histoOffset;
    histoOffset = vidupq_u32((uint32_t) 0, 1);
    histoOffset = histoOffset * nbBins;
    (void) lut;

    int32_t         loopCnt = size;
    uint32x4_t      offset, hist;

    do {
        mve_pred16_t    p = vctp32q(loopCnt);

        offset = vldrbq_u32(pSrc);
        pSrc += 4;

        /* filter out of bound + offset with -min */
        p = vset_predq_bounds_u32_p(offset, min, max, p);
        offset = vsubq_x(offset, min, p);

        /* convert values into bucket index */
        offset = offset * nbBins;
        offset = vmulhq_u32(offset, vdupq_n_u32(invHistWidth));

        /* spread over 4 histograms */
        offset = vaddq_u32(offset, histoOffset);

        /* load masks */
        uint32x4_t      mask = vldrbq_z_u32(pMask, p);
        pMask += 4;

        /* combine predicate with masks */
        p = vcmpeqq_m(mask, 1, p);

        hist = vldrhq_gather_shifted_offset_z_u32(pHistoSplit, offset, p);
        hist = vaddq_x(hist, 1, p);
        vstrhq_scatter_shifted_offset_p_u32(pHistoSplit, offset, hist, p);

        loopCnt -= 4;
    }
    while (loopCnt > 0);

}

static void arm_subhistogr_combine(const uint16_t * pHistoSplit, uint16_t nbBins,
                                   uint16_t * pHistog, uint32_t size)
{
    int32_t         loopCnt = size;
    const uint16_t *pHistoTmp0 = pHistoSplit;
    const uint16_t *pHistoTmp1 = pHistoTmp0 + nbBins;
    const uint16_t *pHistoTmp2 = pHistoTmp1 + nbBins;
    const uint16_t *pHistoTmp3 = pHistoTmp2 + nbBins;

    loopCnt = nbBins;
    do {
        mve_pred16_t    p = vctp16q(loopCnt);
        uint16x8_t      v0, v1, v2, v3, sum;

        v0 = vldrhq_z_u16(pHistoTmp0, p);
        v1 = vldrhq_z_u16(pHistoTmp1, p);
        v2 = vldrhq_z_u16(pHistoTmp2, p);
        v3 = vldrhq_z_u16(pHistoTmp3, p);

        sum = vaddq_x(v0, v1, p);
        sum = vaddq_x(sum, v2, p);
        sum = vaddq_x(sum, v3, p);

        /* reload current histogram content and accumulate */
        v0 = vldrhq_z_u16(pHistog, p);
        sum = vaddq_x(sum, v0, p);

        pHistoTmp0 += 8;
        pHistoTmp1 += 8;
        pHistoTmp2 += 8;
        pHistoTmp3 += 8;

        vstrhq_p_u16(pHistog, sum, p);
        pHistog += 8;
        loopCnt -= 8;
    }
    while (loopCnt > 0);
}




static void arm_histogr_common_u8_mve(const channel_uint8_t * pSrc,
                                      const channel_uint8_t * pMask, const channel_uint8_t * lut,
                                      uint16_t min, uint16_t max, uint16_t nbBins,
                                      uint32_t invHistWidth, uint16_t * pHistog, uint32_t size,
                                      uint32_t * scratch, _arm_histogr_split_fct split)
{
    int32_t         loopCnt = size;
    uint16_t       *pHistoSplit = (uint16_t *) scratch;

    /* clear sub-histograms */
    loopCnt = nbBins * NB_SUB_HISTOGGRAMS;
    do {
        mve_pred16_t    p = vctp16q(loopCnt);

        vstrhq_p_u16(pHistoSplit, vdupq_m_n_u16(vuninitializedq_u16(), 0, p), p);
        pHistoSplit += 8;
        loopCnt -= 8;
    }
    while (loopCnt > 0);

    pHistoSplit = (uint16_t *) scratch;

    split(pSrc, pMask, lut, min, max, nbBins, invHistWidth, size, pHistoSplit);

    arm_subhistogr_combine(pHistoSplit, nbBins, pHistog, size);
}


static void arm_histogr_core_u8_mve(const channel_uint8_t * pSrc, uint16_t * pHistog,
                                    uint16_t nbBins, uint16_t size, uint32_t * scratch)
{
    arm_histogr_common_u8_mve(pSrc, NULL, NULL, 0, 0, nbBins, 0, pHistog, size, scratch,
                              arm_subhistogr_split_basic);
}



static void arm_histogr_core_u8_mask_mve(const channel_uint8_t * pSrc,
                                         const channel_uint8_t * pMask, uint16_t * pHistog,
                                         uint16_t nbBins, uint16_t size, uint32_t * scratch)
{
    arm_histogr_common_u8_mve(pSrc, pMask, NULL, 0, 0, nbBins, 0, pHistog, size, scratch,
                              arm_subhistogr_split_basic_mask);
}

static
void arm_histogr_nonuniform_u8_mve(const channel_uint8_t * pSrc,
                                   const arm_cv_hist_bounds_ctx * bounds, uint16_t nbBins,
                                   uint16_t * pHistog, uint32_t size, uint32_t * scratch)
{
    arm_histogr_common_u8_mve(pSrc, NULL, bounds->phistIntervLUT, bounds->min, bounds->max, nbBins,
                              0, pHistog, size, scratch, arm_subhistogr_split_non_uniform);
}

static
void arm_histogr_nonuniform_u8_mask_mve(const channel_uint8_t * pSrc,
                                        const arm_cv_hist_bounds_ctx * bounds,
                                        const channel_uint8_t * pMask, uint16_t nbBins,
                                        uint16_t * pHistog, uint32_t size, uint32_t * scratch)
{
    arm_histogr_common_u8_mve(pSrc, pMask, bounds->phistIntervLUT, bounds->min, bounds->max, nbBins,
                              0, pHistog, size, scratch, arm_subhistogr_split_non_uniform_mask);
}



static
void arm_histogr_buckets_u8_mve(const channel_uint8_t * pSrc,
                                uint16_t min, uint16_t max, uint16_t nbBins, uint32_t invHistWidth,
                                uint16_t * pHistog, uint32_t size, uint32_t * scratch)
{
    arm_histogr_common_u8_mve(pSrc, NULL, NULL, min, max, nbBins,
                              invHistWidth, pHistog, size, scratch, arm_subhistogr_split_buckets);
}



static
void arm_histogr_buckets_u8_mask_mve(const channel_uint8_t * pSrc,
                                     const channel_uint8_t * pMask, uint16_t min, uint16_t max,
                                     uint16_t nbBins, uint32_t invHistWidth, uint16_t * pHistog,
                                     uint32_t size, uint32_t * scratch)
{
    arm_histogr_common_u8_mve(pSrc, pMask, NULL, min, max, nbBins,
                              invHistWidth, pHistog, size, scratch,
                              arm_subhistogr_split_buckets_mask);
}


static void arm_min_max_no_idx_u8(const uint8_t * pSrc, uint32_t size, uint8_t * min, uint8_t * max)
{
    int32_t         loopCnt = size;
    uint8x16_t      curMaxVec = vdupq_n_u8(0);
    uint8_t         maxValue = 0;
    uint8x16_t      curMinVec = vdupq_n_u8(255);
    uint8_t         minValue = 255;

    do {
        mve_pred16_t    p = vctp32q(loopCnt);
        uint8x16_t      vec;

        vec = vld1q_z(pSrc, p);
        curMaxVec = vmaxq_x(vec, curMaxVec, p);
        curMinVec = vminq_x(vec, curMinVec, p);
        pSrc += 16;
        loopCnt -= 16;
    }
    while (loopCnt > 0);

    /*
     * Get max value across the vector
     */
    maxValue = vmaxvq(maxValue, curMaxVec);
    minValue = vminvq(minValue, curMinVec);
    *max = maxValue;
    *min = minValue;
}


#else


static void arm_min_max_no_idx_u8(const uint8_t * pSrc, uint32_t size, uint8_t * min, uint8_t * max)
{
    uint8_t         cur, minOut, maxOut;
    uint32_t        blkCnt;     /* loop counter */

    /* Load first input value that act as reference value for comparision */
    minOut = *pSrc;
    maxOut = *pSrc;
    pSrc++;

    blkCnt = (size - 1U);


    while (blkCnt > 0U) {
        /* Initialize minVal to the next consecutive values one by one */
        cur = *pSrc++;

        /* compare for the minimum value */
        if (minOut > cur) {
            /* Update the minimum value */
            minOut = cur;
        }
        /* compare for the maximum value */
        if (maxOut < cur) {
            /* Update the maximum value */
            maxOut = cur;
        }
        /* Decrement the loop counter */
        blkCnt--;
    }

    /* Store the minimum value into destination pointer */
    *min = minOut;
    *max = maxOut;
}

static uint32_t arm_offset_filter_with_mask_u8(const channel_uint8_t * pSrc,
                                               const channel_uint8_t * pMask, uint32_t min,
                                               uint32_t max, channel_uint8_t * pDst, uint32_t size)
{
    uint32_t        filteredCnt = 0;

    while (size > 0U) {
        channel_uint8_t in = *pSrc++;
        channel_uint8_t mask = *pMask++;

        if (in >= min && in < max && mask) {
            *pDst++ = in - min;
            filteredCnt++;
        }

        size--;
    }
    return filteredCnt;
}

static uint32_t arm_offset_filter_u8(const channel_uint8_t * pSrc,
                                     uint32_t min, uint32_t max, channel_uint8_t * pDst,
                                     uint32_t size)
{
    uint32_t        filteredCnt = 0;

    while (size > 0U) {
        channel_uint8_t in = *pSrc++;

        if (in >= min && in < max) {
            *pDst++ = in - min;
            filteredCnt++;
        }

        size--;
    }
    return filteredCnt;
}



static void arm_scale_u8(const channel_uint8_t * pSrc,
                         uint32_t invHistWidth, uint32_t nbBins, channel_uint8_t * pDst,
                         uint32_t size)
{
    uint32_t        blkCnt;     /* Loop counter */
    uint32_t        in, out;    /* Temporary variables */


    /* Initialize blkCnt with number of samples */
    blkCnt = size;

    while (blkCnt > 0U) {
        /* C = A * scale */

        /* Scale input and store result in destination buffer. */
        in = *pSrc++;
        in = in * nbBins;
        out = ((q63_t) in * invHistWidth) >> 32;

        *pDst++ = (channel_uint8_t) (__USAT(out, 8));

        /* Decrement loop counter */
        blkCnt--;
    }

}


static
void arm_histogr_buckets_u8(const arm_cv_image_gray8_t * pImageIn,
                            uint16_t min, uint16_t max, uint16_t nbBins, uint32_t invHistWidth,
                            uint16_t * pHistog, uint32_t * scratch)
{
    const channel_uint8_t *pSrc = pImageIn->pData;
    uint32_t        blockSize;
    for (uint32_t rows = 0; rows < pImageIn->height; rows++) {
        /* remove offset and filter out of bound items */
        blockSize =
            arm_offset_filter_u8(pSrc, min, max, (channel_uint8_t *) scratch, pImageIn->width);

        /* convert values into bucket index */
        arm_scale_u8((channel_uint8_t *) scratch, invHistWidth, nbBins,
                     (channel_uint8_t *) scratch, blockSize);

        /* build histogram */
        arm_histogr_core_u8((channel_uint8_t *) scratch, pHistog, blockSize);

        pSrc += pImageIn->width;
    }
}

static
void arm_histogr_buckets_u8_mask(const arm_cv_image_gray8_t * pImageIn,
                                 const channel_uint8_t * pMask, uint16_t min, uint16_t max,
                                 uint16_t nbBins, uint32_t invHistWidth, uint16_t * pHistog,
                                 uint32_t * scratch)
{
    const channel_uint8_t *pSrc = pImageIn->pData;
    uint32_t        blockSize;
    for (uint32_t rows = 0; rows < pImageIn->height; rows++) {
        /* remove offset and filter out of bound items */

        blockSize =
            arm_offset_filter_with_mask_u8(pSrc, pMask, min, max,
                                           (channel_uint8_t *) scratch, pImageIn->width);
        pMask += pImageIn->width;

        /* convert values into bucket index */
        arm_scale_u8((channel_uint8_t *) scratch, invHistWidth, nbBins,
                     (channel_uint8_t *) scratch, blockSize);

        /* build histogram */
        arm_histogr_core_u8((channel_uint8_t *) scratch, pHistog, blockSize);

        pSrc += pImageIn->width;
    }
}

#endif




static uint8_t arm_histogram_get_interval_idx(uint8_t * histIntervLUT, int value)
{
    return histIntervLUT[value];
}

static
void arm_histogr_nonuniform_u8(const channel_uint8_t * pIn, const arm_cv_hist_bounds_ctx * bounds,
                               uint16_t * pHistog, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        channel_uint8_t in = *pIn++;

        if (in < bounds->min || in > bounds->max) {
            continue;
        }

        pHistog[arm_histogram_get_interval_idx(bounds->phistIntervLUT, in)]++;
    }
}



static
void arm_histogr_nonuniform_u8_mask(const channel_uint8_t * pIn,
                                    const arm_cv_hist_bounds_ctx * bounds,
                                    const channel_uint8_t * pMask, uint16_t * pHistog,
                                    uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        channel_uint8_t in = pIn[i];
        channel_uint8_t mask = pMask[i];

        if (in < bounds->min || in > bounds->max) {
            continue;
        }

        if (mask)
            pHistog[arm_histogram_get_interval_idx(bounds->phistIntervLUT, in)]++;
    }
}






/**
 * @brief      Returns gray8 histogram scratch size
 *
 * @param[in]     pImageIn        The input image
 * @param[in]     nbBins          The histogram dimension
 * @param[in]     bounds          Structure describing histograms min/max and cache for non-uniform histograms
 * @return        Scratch size array in bytes
 */

uint16_t arm_histogram_gray8_get_scratch_size(const arm_cv_image_gray8_t * pImageIn,
                                              uint16_t nbBins,
                                              const arm_cv_hist_bounds_ctx * bounds)
{
    uint16_t        scratchSize = 0;

#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))
    (void) pImageIn;
    (void) bounds;

    /* need room for 4 sub-histograms */
    scratchSize += nbBins * 4 * sizeof(uint16_t);
#else
    if ((bounds && (bounds->min != 0) && (bounds->max != 255)) || (nbBins != 256))
        scratchSize += pImageIn->width;
#endif
    return scratchSize;
}



/**
 * @brief      Returns Gray8 image non-uniform histogram LUT
 *
 * @param[in]     sortedBoundaries  Sorted histogram boundaries (edges)
 * @param[in]     BoundariesSize    nb of histogram boundaries (edges)
 * @param[in]     bounds          Structure describing histograms min/max and cache for non-uniform histograms
 */


void arm_histogram_set_nonuniform_bound(channel_uint8_t * sortedBoundaries,
                                        uint8_t sortedBoundaries_size, arm_cv_hist_bounds_ctx * ctx)
{
    int             curInterval = 0;

    ctx->min = sortedBoundaries[0];
    ctx->max = sortedBoundaries[sortedBoundaries_size];

    for (int value = ctx->min; value <= ctx->max; value++) {
        if (value >= sortedBoundaries[curInterval + 1]) {
            curInterval++;
        }
        ctx->phistIntervLUT[value] = curInterval;
    }
}




/**
  @ingroup imageAnalysis
 */

/**
 * @brief      Gray8 image histogram
 *
 * @param[in]     pImageIn        The input image
 * @param[in]     pMask           The optional input mask
 * @param[in,out] pHistogVal      The resulting histogram
 * @param[in]     nbBins          The histogram dimension
 * @param[in]     bounds          Structure describing histograms min/max and cache for non-uniform histograms
 * @param[in]     uniform         Flag indicating whether the histogram is uniform or not
 * @param[in]     accumulate      Accumulation flag. If set, the histogram is not cleared in the beginning when it is allocated.
 * @param[in]     scratch         Internal storage
 * @return        error status
 *
 * @par  scratch buffer sizing:
 *
 * Size of temporary buffers:
 *   - given by arm_histogram_gray8_get_scratch_size
 */

arm_cv_status arm_histogram_gray8(const arm_cv_image_gray8_t * pImageIn,
                                  const arm_cv_image_gray8_t * pMask, uint16_t * pHistogVal,
                                  uint16_t nbBins,
                                  const arm_cv_hist_bounds_ctx * bounds, uint32_t uniform,
                                  uint32_t accumulate, uint32_t * scratch)
{
    uint32_t        blockSize = pImageIn->height * pImageIn->width;
    channel_uint8_t *pSrc = pImageIn->pData;
    channel_uint8_t *pMaskSrc = NULL;


    if (pMask)
        pMaskSrc = pMask->pData;


    if (!accumulate)
        /* clear histogram  */
        for (int i = 0; i < nbBins; i++)
            pHistogVal[i] = 0;

    if (!(uniform & 1)) {

        /*
         * non-uniform histograms :
         * bounds contains histogram LUT returned by arm_histogram_set_nonuniform_bound
         */

        if (bounds == NULL)
            return ARM_CV_ARGUMENT_ERROR;

        if (bounds->phistIntervLUT == NULL)
            return ARM_CV_ARGUMENT_ERROR;


        if (pMaskSrc) {
#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))
            if (blockSize > nbBins)
                arm_histogr_nonuniform_u8_mask_mve(pSrc, bounds, pMaskSrc, nbBins, pHistogVal,
                                                   blockSize, scratch);
            else
#endif
                arm_histogr_nonuniform_u8_mask(pSrc, bounds, pMaskSrc, pHistogVal, blockSize);

        } else {
#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))
            if (blockSize > nbBins) {
                arm_histogr_nonuniform_u8_mve(pSrc, bounds, nbBins, pHistogVal, blockSize, scratch);
            } else
#endif
                arm_histogr_nonuniform_u8(pSrc, bounds, pHistogVal, blockSize);
        }


    } else if ((bounds && (bounds->min != 0) && (bounds->max != 255)) || (nbBins != 256)) {
        /* bucketting required */
        uint32_t        min = 0, max = 256;

        if (scratch == NULL)
            return ARM_CV_ARGUMENT_ERROR;

        if (bounds) {
            /* user provided sortedBoundaries */
            min = bounds->min;
            max = bounds->max + 1;

            /* could potentially be relaxed */
            if (max < min || nbBins > (max - min)) {
                return ARM_CV_ARGUMENT_ERROR;
            }
        } else {
            /* auto detected boundaries */
            uint8_t         min8, max8;

            arm_min_max_no_idx_u8(pSrc, blockSize, &min8, &max8);

            min = min8;
            max = max8 + 1;
        }

        /* compute nbBins / (max - min)  */
        uint32_t        den = max - min;
        uint32_t        invHistWidth;

        invHistWidth = 0xffffffff / den;
        /* rounding */
        invHistWidth += (den / 2);

#if defined(ARM_MATH_MVEI)
        if (!pMaskSrc)
            arm_histogr_buckets_u8_mve(pSrc, min, max, nbBins, invHistWidth, pHistogVal, blockSize,
                                       scratch);
        else
            arm_histogr_buckets_u8_mask_mve(pSrc, pMaskSrc, min, max, nbBins, invHistWidth,
                                            pHistogVal, blockSize, scratch);
#else
        if (!pMaskSrc)
            arm_histogr_buckets_u8(pImageIn, min, max, nbBins, invHistWidth, pHistogVal, scratch);
        else
            arm_histogr_buckets_u8_mask(pImageIn, pMaskSrc, min, max, nbBins, invHistWidth,
                                        pHistogVal, scratch);


#endif
    } else {
        /* no bucketting needed */
        if (pMaskSrc) {
#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))
            /* do not use MVE for degenerated cases */
            if (blockSize > nbBins)
                arm_histogr_core_u8_mask_mve(pSrc, pMaskSrc, pHistogVal, nbBins, blockSize,
                                             scratch);
            else
#endif
                arm_histogr_core_u8_mask(pSrc, pMaskSrc, pHistogVal, blockSize);
        } else {
#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))
            /* do not use MVE for degenerated cases */
            if (blockSize > nbBins)
                arm_histogr_core_u8_mve(pSrc, pHistogVal, nbBins, blockSize, scratch);
            else
#endif
                arm_histogr_core_u8(pSrc, pHistogVal, blockSize);

        }
    }


#if 0
    if (pHistogEdgesVal && pHistogEdgesScal) {
        /* build edge list */
        q31_t           den = nbBins;
        q31_t           inv;
        uint32_t        signB = arm_recip_q31(den, &inv, armRecipTableQ31);
        q63_t           div = (q63_t) (max - min) * (q63_t) inv;
        q31_t           invHistWidth, norm;

        arm_norm_64_to_32u(div, &invHistWidth, &norm);

        invHistWidth = invHistWidth >> 8;
        int32_t         scal = -(32 + norm - signB - 1) + 8;
        scal = 31 - scal;
        *pHistogEdgesScal = scal;

        for (int i = 0; i < nbBins; i++)
            pHistogEdgesVal[i] = (min << scal) + i * invHistWidth;
    }
#endif
    return ARM_CV_SUCCESS;
}
