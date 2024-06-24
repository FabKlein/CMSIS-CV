#include "common.h"
#include "load.hpp"
#include "test_config.h"
#include <vector>
#include <stdio.h>
#include <cassert>

extern          "C" {
#include "cv/image_analysis.h"
}
#if defined(TESTGROUP3)

#define ARR(...) __VA_ARGS__

#define GRAY8_HISTOGR_UT(NB_BINS, NUM_BOUNDS_EDGES, BOUNDS_EDGES, UNIFORM, WITH_MASKS, WITH_ACCUM) \
    {                                                       \
        uint8_t         bounds[NUM_BOUNDS_EDGES] =          \
          ARR(BOUNDS_EDGES);                                \
        ;                                                   \
                                                            \
        test_gray8_histogr(inputs, wbuf, total_bytes, testid, cycles, \
                                                NB_BINS, bounds, UNIFORM, WITH_MASKS, WITH_ACCUM); \
    }



#define dump_buf(a, buf_sz, wrap, format )                  \
{                                                           \
    printf("%s:\n", #a);                                    \
    for (int i = 0; i < buf_sz; i++)                        \
        printf(i % wrap == wrap - 1 ? format",\n":format", ", a[i]);  \
    printf("\n");                                           \
}


static arm_cv_status test_gray8_histogr(const unsigned char *inputs,
                                                             unsigned char *&outputs,
                                                             uint32_t & total_bytes,
                                                             uint32_t testid, long &cycles,
                                                             uint16_t histSize,
                                                             channel_uint8_t * boundsArr,
                                                             bool uniform, bool useMask,
                                                             bool useAccumulation)
{
    long            start, end;
    uint32_t        width, height;
    int             bufid = TENSOR_START + testid;

    uint32_t        histAccum = useAccumulation;
    get_img_dims(inputs, bufid, &width, &height);

    arm_cv_hist_bounds_ctx ctx;
    uint8_t        *lut = NULL;
    uint8_t        *mask = NULL;


    if (!uniform) {
        /* look up table caching */
        uint32_t        lutSize = boundsArr[histSize] - boundsArr[0];
        lut = (uint8_t *) malloc(lutSize);
        ctx.phistIntervLUT = lut;
        arm_histogram_set_nonuniform_bound(boundsArr, histSize, &ctx);

    } else {
        ctx.min = boundsArr[0];
        ctx.max = boundsArr[1];
        ctx.phistIntervLUT = NULL;
    }

    std::vector < BufferDescription > desc = {
        BufferDescription(Shape(histSize, 1)
                          , kIMG_NUMPY_TYPE_UINT16)
    };


    outputs = create_write_buffer(desc, total_bytes);

    const uint8_t  *src = Buffer < uint8_t >::read(inputs, bufid);
    uint16_t       *dst = Buffer < uint16_t >::write(outputs, 0);

    const arm_cv_image_gray8_t input = { (uint16_t) width,
        (uint16_t) height,
        (uint8_t *) src
    };

    const arm_cv_image_gray8_t *pmaskInput = NULL;
    arm_cv_image_gray8_t maskInput;

    if (useMask) {
        /* create artifical mask based on input image LSBit */
        mask = (uint8_t *) malloc(width * height);
        assert(mask != NULL);

        for (uint32_t i = 0; i < width * height; i++)
            mask[i] = src[i] & 1;

        maskInput.width = (uint16_t) width;
        maskInput.height = (uint16_t) height;
        maskInput.pData = (uint8_t *) mask;

        pmaskInput = &maskInput;
    }


    uint16_t        scratchSize = arm_histogram_gray8_get_scratch_size(&input, histSize, &ctx);
    uint8_t        *scratch = (uint8_t *) malloc(scratchSize);


    if (useAccumulation) {
        /* add artificial 1 offset when accumulation enabled */
        std::fill(dst, dst + 256, static_cast < uint16_t > (1));
    }

    start = time_in_cycles();


    arm_cv_status      status = arm_histogram_gray8(&input,
                                                 pmaskInput, dst,
                                                 histSize,
                                                 &ctx, uniform, histAccum,
                                                 scratch);

    end = time_in_cycles();


    free(scratch);
    if (lut)
        free(lut);

    if (mask)
        free(mask);

    //dump_buf(dst, histSize, 32, "%d");
    cycles = end - start;
    return status;
}




void run_test(const unsigned char *inputs,
              const uint32_t testid,
              const uint32_t funcid, unsigned char *&wbuf, uint32_t & total_bytes, long &cycles)
{
    wbuf = nullptr;
    switch (funcid) {

      case 0:
          GRAY8_HISTOGR_UT(256, 2, ARR({0, 256 - 1}), true, false, false);
          break;

      case 1:
          GRAY8_HISTOGR_UT(256, 2, ARR({0, 256 - 1}), true, false, true);
          break;

      case 2:
          GRAY8_HISTOGR_UT(128, 2, ARR({0, 256 - 1}), true, false, false);
          break;

      case 3:
          GRAY8_HISTOGR_UT(10, 2, ARR({0, 256 - 1}), true, false, false);
          break;

      case 4:
          GRAY8_HISTOGR_UT(33, 2, ARR({0, 256 - 1}), true, false, false);
          break;

      case 5:
          GRAY8_HISTOGR_UT(55, 2, ARR({0, 256 - 1}), true, false, false);
          break;

      case 6:
          GRAY8_HISTOGR_UT(64, 2, ARR({0, 256 - 1}), true, false, false);
          break;

      case 7:
          GRAY8_HISTOGR_UT(200, 2, ARR({0, 256 - 1}), true, false, false);
          break;

      case 8:
          GRAY8_HISTOGR_UT(128, 2, ARR({10, 246 - 1}), true, false, false);
          break;

      case 9:
          GRAY8_HISTOGR_UT(10, 2, ARR({100, 200 - 1}), true, false, false);
          break;

      case 10:
          GRAY8_HISTOGR_UT(33, 2, ARR({33, 99 - 1}), true, false, false);
          break;

      case 11:
          GRAY8_HISTOGR_UT(6, 7, ARR({0, 10, 20, 40, 80, 160, 200}), false, false, false);
          break;

      case 12:
          GRAY8_HISTOGR_UT(256, 2, ARR({0, 256 - 1}), true, true, false);
          break;

      case 13:
           GRAY8_HISTOGR_UT(33, 2, ARR({33, 99 - 1}), true, true, false);
          break;

      case 14:
          GRAY8_HISTOGR_UT(6, 7, ARR({0, 10, 20, 40, 80, 160, 200}), false, true, false);
          break;


    }

}

#endif
