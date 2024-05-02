/* ----------------------------------------------------------------------
 * Project:      CMSIS CV Library
 * Title:        arm_cv_types.h
 * Description:  Datatypes CMSIS-CV
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
#ifndef ARM_CV_TYPES_H
#define ARM_CV_TYPES_H

#include "arm_math_types.h"

#ifdef   __cplusplus
extern "C"
{
#endif


/**
 * @brief   Structure for a grayscale 8
 * @param   width     image width in pixels
 * @param   height    image height in pixels
 * @param   pData     pointer to the array containing the data for the pixels
 * @return 
 *
 */
typedef struct _arm_cv_image_gray8 {
  uint8_t* pData;
  uint16_t width;
  uint16_t height;
} arm_cv_image_gray8_t;

/**
 * @brief   Structure for a q15 image
 * @param   width     image width in pixels
 * @param   height    image height in pixels
 * @param   pData     pointer to the array containing the data for the pixels
 * @return 
 *
 */
typedef struct _arm_cv_image_q15 {
  q15_t* pData;
  uint16_t width;
  uint16_t height;
} arm_cv_image_q15_t;

/**
 * @brief   Gradient in q15
 * @param   x     first component of the gradient
 * @param   y     second component of the gradient
 * @return 
 *
 */
typedef struct _arm_cv_gradient_q15 {
  q15_t x;
  q15_t y;
} arm_cv_gradient_q15_t;
/**
 * @brief   Image of gradients in q15 
 * @param   width     image width in pixels
 * @param   height    image height in pixels
 * @param   pData     pointeur to the array containing the data for the gradient
 * @return 
 *
 */
typedef struct _arm_cv_image_grad_q15 {
  arm_cv_gradient_q15_t* pData;
  uint16_t width;
  uint16_t height;
} arm_cv_image_gradient_q15_t;



#ifdef   __cplusplus
}
#endif

#endif

