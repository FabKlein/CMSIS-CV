/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        cannysobel.c
 * Description:  Last steps of the canny edge detector (after the gaussian filter)
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

#include <stdio.h>

#define dump_buf(a, buf_sz, wrap, format )                  \
{                                                           \
    printf("%s:\n", #a);                                    \
    for (int i = 0; i < buf_sz; i++)                        \
        printf(i % wrap == wrap - 1 ? format",\n":format", ", a[i]);  \
    printf("\n");                                           \
}


//#if  1                          //((!defined(ARM_MATH_MVEI)) ||(defined(FORCE_SCALAR)))
static uint32_t arm_offset_filter_with_mask_u8(const channel_uint8_t * pSrc,
                                               const channel_uint8_t * pMask, uint32_t min,
                                               uint32_t max, channel_uint8_t * pDst,
                                               uint32_t blockSize)
{
    uint32_t        filteredCnt = 0;

    while (blockSize > 0U) {
        channel_uint8_t in = *pSrc++;
        channel_uint8_t mask = *pMask++;

        if (in >= min && in < max && mask) {
            *pDst++ = in - min;
            filteredCnt++;
        }

        blockSize--;
    }
    return filteredCnt;
}

static uint32_t arm_offset_filter_u8(const channel_uint8_t * pSrc,
                                     uint32_t min, uint32_t max, channel_uint8_t * pDst,
                                     uint32_t blockSize)
{
    uint32_t        filteredCnt = 0;

    while (blockSize > 0U) {
        channel_uint8_t in = *pSrc++;

        if (in >= min && in < max) {
            *pDst++ = in - min;
            filteredCnt++;
        }

        blockSize--;
    }
    return filteredCnt;
}

static void arm_min_max_no_idx_u8(const uint8_t * pSrc, uint32_t blockSize, uint8_t * min,
                                  uint8_t * max)
{
    uint8_t         cur, minOut, maxOut;
    uint32_t        blkCnt;     /* loop counter */

    /* Load first input value that act as reference value for comparision */
    minOut = *pSrc;
    maxOut = *pSrc;
    pSrc++;

    blkCnt = (blockSize - 1U);


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



static void arm_scale_u8(const channel_uint8_t * pSrc,
                         uint32_t invHistWidth, uint32_t nbBins, channel_uint8_t * pDst,
                         uint32_t blockSize)
{
    uint32_t        blkCnt;     /* Loop counter */
    uint32_t        in, out;    /* Temporary variables */



    /* Initialize blkCnt with number of samples */
    blkCnt = blockSize;

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
void arm_histogr_core_u8(const channel_uint8_t * pIn, uint16_t * pHistog, uint32_t size)
{

    for (uint32_t height = 0; height < size; height++)
        pHistog[*pIn++] += 1;
}

static
void arm_histogr_u8_core_mask(const channel_uint8_t * pIn, const channel_uint8_t * pMask,
                              uint16_t * pHistog, uint32_t size)
{
    for (uint32_t height = 0; height < size; height++) {
        channel_uint8_t in = *pIn++;
        channel_uint8_t mask = *pMask++;

        if (mask)
            pHistog[in] += 1;
    }
}


uint16_t arm_histogram_gray8_get_scratch_size(const arm_cv_image_gray8_t * pImageIn,
                                              uint16_t nbBins, const arm_cv_hist_bounds_ctx * bounds)
{
    uint16_t        scratchSize = 0;
    if ((bounds && (bounds->min != 0) && (bounds->max != 255)) || (nbBins != 256))
        scratchSize = pImageIn->width;
    return scratchSize;
}






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


__STATIC_INLINE uint8_t arm_histogram_get_interval_idx(uint8_t * histIntervLUT, int value)
{
    return histIntervLUT[value];
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
                               uint32_t accumulate, channel_uint8_t * scratch)
{
    uint32_t        min = 0, max = 256;
    uint16_t        blockSize = pImageIn->height * pImageIn->width;
    channel_uint8_t *pSrc = pImageIn->pData;
    channel_uint8_t *pMaskSrc = NULL;


    if (pMask)
        pMaskSrc = pMask->pData;


    if (!accumulate)
        /* clear histogram  */
        for (int i = 0; i < nbBins; i++)
            pHistogVal[i] = 0;

    if (!(uniform & 1)) {

        if (bounds == NULL)
            return ARM_CV_ARGUMENT_ERROR;
        if (bounds->phistIntervLUT == NULL)
            return ARM_CV_ARGUMENT_ERROR;

        /* non-uniform histograms
         *
         * bounds contains histogram LUT returned by arm_histogram_set_nonuniform_bound
         */
        for (int i = 0; i < blockSize; i++) {
            channel_uint8_t value = pSrc[i];

            //printf("value %d idx %d\n", value, arm_histogram_get_interval_idx(bounds->phistIntervLUT, value));
            if (value < bounds->min || value > bounds->max) {
                continue;
            }
            if (pMaskSrc) {
                if (!pMaskSrc[i])
                    continue;
            }
            pHistogVal[arm_histogram_get_interval_idx(bounds->phistIntervLUT, value)]++;
        }
    } else if ((bounds && (bounds->min != 0) && (bounds->max != 255)) || (nbBins != 256)) {
        /* bucketting required */

        if (scratch == NULL)
            return ARM_CV_ARGUMENT_ERROR;

        if (bounds) {
            /* user provided sortedBoundaries */
            min = bounds->min;
            max = bounds->max + 1;

            if (max < min || nbBins > (max - min)) {

                return ARM_CV_ARGUMENT_ERROR;
            }
        } else {
            /* auto detected sortedBoundaries */
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


        /* remove offset and filter */
        for (uint32_t rows = 0; rows < pImageIn->height; rows++) {
            if (pMaskSrc) {
                blockSize =
                    arm_offset_filter_with_mask_u8(pSrc, pMaskSrc, min, max, scratch,
                                                   pImageIn->width);
                pMaskSrc += pImageIn->width;
            } else
                blockSize = arm_offset_filter_u8(pSrc, min, max, scratch, pImageIn->width);

            /* convert values into bucket index */
            arm_scale_u8(scratch, invHistWidth, nbBins, scratch, blockSize);

            /* build histogram */
            arm_histogr_core_u8(scratch, pHistogVal, blockSize);

            //dump_buf(pHistogVal, nbBins, 64, "%d");

            pSrc += pImageIn->width;
        }

    } else {
        /* no bucketting needed */
        if (pMaskSrc) {
            arm_histogr_u8_core_mask(pSrc, pMaskSrc, pHistogVal, blockSize);
        } else
            arm_histogr_core_u8(pSrc, pHistogVal, blockSize);
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


