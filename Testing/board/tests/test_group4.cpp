#include "common.h"
#include "load.hpp"
#include "test_config.h"
#include <stdio.h>
#include <vector>
#undef NDEBUG
#include <cassert>

extern "C"
{
#include "cv/non_linear_filters.h"
}
#if defined(TESTGROUP4)


#define SCRATCH_HEADROOM 8
#define SCRATCH_FILL_PAT 0xa5

static arm_cv_status test_gray8_medfiltr(const unsigned char *inputs, unsigned char *&outputs,
                                         uint32_t &total_bytes, uint32_t testid, long &cycles)
{
    long start, end;
    int bufid = TENSOR_START + testid;

    uint32_t width, height;

    get_img_dims(inputs, bufid, &width, &height);
    std::vector<BufferDescription> desc = {BufferDescription(Shape(height, width), kIMG_GRAY8_TYPE)};

    outputs                             = create_write_buffer(desc, total_bytes);

    const uint8_t *src                  = Buffer<uint8_t>::read(inputs, bufid);
    uint8_t *dst                        = Buffer<uint8_t>::write(outputs, 0);

    /* alternate 3,5,7 kernel size */
    uint8_t kernDim = 3 + (testid % 3) * 2;

    uint16_t scratchSize = arm_median_filter_gray8_get_scratch_size(kernDim);

    scratchSize += SCRATCH_HEADROOM;
    uint64_t *scratch = (uint64_t *)malloc(scratchSize);

    uint8_t *scratch8 = (uint8_t *)scratch;
    memset(scratch8, SCRATCH_FILL_PAT, scratchSize);

    // The test to run is executed with some timing code.
    start                            = time_in_cycles();

    const arm_cv_image_gray8_t input = {(uint16_t)width, (uint16_t)height, (uint8_t *)src};
    arm_cv_image_gray8_t ouput       = {(uint16_t)width, (uint16_t)height, (uint8_t *)dst};

    arm_median_filter_gray8(&input, &ouput, kernDim, scratch);
    // memcpy(dst,src,get_buffer_length(inputs,bufid));

    end    = time_in_cycles();
    cycles = end - start;

    for (int i = 0; i < SCRATCH_HEADROOM; i++)
        if (scratch8[scratchSize - 1 - i] != SCRATCH_FILL_PAT)
            printf("scratch overrun %d %x \n", i, scratch8[scratchSize - 1 - i]);

    free(scratch);

    return ARM_CV_SUCCESS;
}


void run_test(const unsigned char *inputs, const uint32_t testid, const uint32_t funcid, unsigned char *&wbuf,
              uint32_t &total_bytes, long &cycles)
{
    wbuf                 = nullptr;
    (void)funcid;

    test_gray8_medfiltr(inputs, wbuf, total_bytes, testid, cycles);
}

#endif
