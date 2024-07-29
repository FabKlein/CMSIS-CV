/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        non_linear_filters.h
 * Description:  Non-Linear 2D filters for CMSIS-CV
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
#ifndef ARM_CV_NON_LINEAR_FILTER_H
#define ARM_CV_NON_LINEAR_FILTER_H


#include "arm_cv_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint16_t arm_median_filter_gray8_get_scratch_size(uint8_t kernDim);

    /**
     * @brief      Gray8 median filter
     *
     * @param[in]  ImageIn   The input image
     * @param      ImageOut  The output image
     */
    arm_cv_status arm_median_filter_gray8(const arm_cv_image_gray8_t *ImageIn, arm_cv_image_gray8_t *ImageOut,
                                          uint8_t kern_dim, uint64_t *scratch);

#ifdef __cplusplus
}
#endif

#endif
