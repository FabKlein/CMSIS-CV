/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_hist_equalize_gray8.c
 * Description:  Gray8 histogram equalization
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
#include "cv/image_analysis.h"



typedef struct {
#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))
    /* need room for 4 sub-histograms for MVE implementation */
    uint16_t        histogScratch[256 * 4];
#endif
    /* histogram storage */
    uint16_t        histo[256];
    /* cdf : cumulative distribution function */
    uint8_t         cdf[256];
} arm_hist_equalize_scratch_t;



/**
 * @brief         Returns gray8 histogram equalization scratch size
 *
 * @return        Scratch size array in bytes
 */

uint16_t arm_hist_equalize_gray8_get_scratch_size(void)
{
    return sizeof(arm_hist_equalize_scratch_t);
}

#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))


static void arm_hist_equalize_apply_normalization(const uint8_t * pDataIn, uint8_t * pDataOut,
                                              uint32_t image_size, uint8_t * cdf8_fx)
{
    int32_t         loopCnt = image_size;

    do {

        mve_pred16_t    p = vctp8q(loopCnt);
        /* pixel values used as gather load index in the CDF table */
        uint8x16_t      inOffs = vld1q_z(pDataIn, p);

        uint8x16_t      vIn = vldrbq_gather_offset_z(cdf8_fx, inOffs, p);

        vst1q_p(pDataOut, vIn, p);
        pDataOut += 16;
        pDataIn += 16;
        loopCnt -= 16;
    }
    while (loopCnt > 0);
}

static void arm_hist_equalize_set_mapping(const uint16_t * histo, uint32_t image_size,
                                      uint8_t * cdf8_fx)
{
    /* divisor in Q8.24 */
    uint32_t        div = 255UL * 16777216UL / image_size;
    uint32_t        cdfprev;
    int             i;
    uint32x4_t      vcdf = vuninitializedq_u32();

    cdfprev = histo[0];
    cdf8_fx[0] = ((uint64_t) (cdfprev << 8) * (uint64_t) div + (uint64_t) (1UL << 31)) >> 32;

    for (i = 0; i < 256 / 4; i++) {

        cdfprev = cdfprev + histo[4 * i + 1];
        vcdf = vsetq_lane_u32(cdfprev, vcdf, 0);

        cdfprev = cdfprev + histo[4 * i + 2];
        vcdf = vsetq_lane_u32(cdfprev, vcdf, 1);

        cdfprev = cdfprev + histo[4 * i + 3];
        vcdf = vsetq_lane_u32(cdfprev, vcdf, 2);

        cdfprev = cdfprev + histo[4 * i + 4];
         vcdf = vsetq_lane_u32(cdfprev, vcdf, 3);

        vcdf = vrmulhq_u32(vcdf << 8, vdupq_n_u32(div));
        vstrbq_u32(&cdf8_fx[4 * i + 1], vcdf);
    }

    cdfprev = cdfprev + histo[4 * i + 1];
    vcdf = vsetq_lane_u32(cdfprev, vcdf, 0);

    cdfprev = cdfprev + histo[4 * i + 2];
    vcdf = vsetq_lane_u32(cdfprev, vcdf, 1);

    cdfprev = cdfprev + histo[4 * i + 3];
    vcdf = vsetq_lane_u32(cdfprev, vcdf, 2);

    vcdf = vrmulhq_u32(vcdf << 8, vdupq_n_u32(div));
    vstrbq_p_u32(&cdf8_fx[4 * i + 1], vcdf, vctp32q(3));

}

#else

static void arm_hist_equalize_set_mapping(const uint16_t * histo, uint32_t image_size,
                                      uint8_t * cdf8_fx)
{
    /* divisor in Q8.24 */
    uint32_t        div = 255UL * 16777216UL / image_size;
    uint32_t        cdfprev;

    cdfprev = histo[0];
    cdf8_fx[0] = ((uint64_t) (cdfprev << 8) * (uint64_t) div + (uint64_t) (1UL << 31)) >> 32;

    for (int i = 1; i < 256; ++i) {
        cdfprev = cdfprev + histo[i];
        cdf8_fx[i] = ((uint64_t) (cdfprev << 8) * (uint64_t) div + (uint64_t) (1UL << 31)) >> 32;
    }
}

static void arm_hist_equalize_apply_normalization(const uint8_t * pDataIn, uint8_t * pDataOut,
                                              uint32_t image_size, uint8_t * cdf8_fx)
{
    for (int i = 0; i < (int) image_size; ++i) {
        pDataOut[i] = cdf8_fx[pDataIn[i]];
    }
}

#endif




/**
 * @brief      Gray8 image histogram equalization
 *
 * @param[in]     pImageIn        The input image
 * @param[out]    pImageOut       The output equalized image
 * @param[in]     scratch         Internal storage
 *
 * @par  scratch buffer sizing:
 *
 * Size of temporary buffers:
 *   - given by arm_hist_equalize_gray8_get_scratch_size
 */

void arm_hist_equalize_gray8(const arm_cv_image_gray8_t * pImageIn,
                                 arm_cv_image_gray8_t * pImageOut, uint32_t * scratch)
{

    arm_hist_equalize_scratch_t *pScratch = (arm_hist_equalize_scratch_t *) scratch;
    uint16_t       *histo = pScratch->histo;
    uint8_t        *cdf8_fx = pScratch->cdf;
    uint32_t        image_size = pImageIn->height * pImageIn->width;


   arm_histogram_gray8(pImageIn,
                         NULL, histo,
                         256,
                         NULL, 1, 0,
#if (defined(ARM_MATH_MVEI) && !defined(FORCE_SCALAR))
                         (uint32_t *) pScratch->histogScratch
#else
                         NULL
#endif
                         );


    arm_hist_equalize_set_mapping(histo, image_size, cdf8_fx);


    arm_hist_equalize_apply_normalization(pImageIn->pData, pImageOut->pData, image_size, cdf8_fx);

}
