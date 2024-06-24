/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        image_transforms.h
 * Description:  Image transformations for CMSIS-CV
 *
 *
 * Target Processor: Cortex-M and Cortex-A cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2014 ARM Limited or its affiliates. All rights reserved.
 * Copyright (C) 2024 Himax Technologies, Inc. or its affiliates. All rights reserved.
 *
 * (Copyright details are available in each C source file for each function)
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

#ifndef ARM_CV_IMAGE_ANALYSIS_H
#define ARM_CV_IMAGE_ANALYSIS_H


//this include contain the specific types of the library
#include "arm_cv_types.h"

#ifdef   __cplusplus
extern          "C" {
#endif


typedef struct _arm_cv_hist_bounds_ctx {
    channel_uint8_t     min;
    channel_uint8_t     max;
    channel_uint8_t    *phistIntervLUT;
} arm_cv_hist_bounds_ctx;


/**
 * @brief      Returns gray8 histogram scratch size
 *
 * @param[in]     pImageIn        The input image
 * @param[in]     nbBins          The histogram dimension
 * @param[in]     bounds          Structure describing histograms min/max and cache for non-uniform histograms
 */

extern uint16_t arm_histogram_gray8_get_scratch_size(const arm_cv_image_gray8_t * pImageIn,
                                                     uint16_t nbBins,
                                                     const arm_cv_hist_bounds_ctx * bounds);

/**
 * @brief      Returns Gray8 image non-uniform histogram LUT
 *
 * @param[in]     sortedBoundaries  Sorted histogram boundaries (edges)
 * @param[in]     BoundariesSize    nb of histogram boundaries (edges)
 * @param[in]     bounds          Structure describing histograms min/max and cache for non-uniform histograms
 */

extern void arm_histogram_set_nonuniform_bound(channel_uint8_t * sortedBoundaries,
                                        uint8_t sortedBoundariesSize, arm_cv_hist_bounds_ctx * ctx);

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
 */

extern arm_cv_status arm_histogram_gray8(const arm_cv_image_gray8_t * pImageIn,
                                      const arm_cv_image_gray8_t * pMask, uint16_t * pHistogVal,
                                      uint16_t nbBins,
                                      const arm_cv_hist_bounds_ctx * bounds, uint32_t uniform,
                                      uint32_t accumulate, channel_uint8_t * scratch);



#ifdef   __cplusplus
}
#endif
#endif
