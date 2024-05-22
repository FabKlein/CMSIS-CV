#include "common.h"
#include "load.hpp"
#include "test_config.h"
#include <vector>

#if defined(TESTGROUP1)

void test_rgb(const unsigned char* inputs,
                 unsigned char* &outputs,
                 uint32_t &total_bytes,
                 uint32_t testid)
{
    uint32_t width,height;
    int bufid = TENSOR_START + 2;
    if (testid == 3)
        bufid = TENSOR_START + 3 ;
    
    get_img_dims(inputs,bufid,&width,&height);
    std::vector<BufferDescription> desc = {BufferDescription(Shape(height,width))};

    outputs = create_write_buffer(desc,total_bytes);

    const uint8_t *src = Buffer<uint8_t>::read(inputs,bufid);
    uint8_t *dst = Buffer<uint8_t>::write(outputs,0);
    memcpy(dst,src,get_buffer_length(inputs,bufid));
}

void test_gray8(const unsigned char* inputs,
                 unsigned char* &outputs,
                 uint32_t &total_bytes,
                 uint32_t testid)
{
    uint32_t nb_dims,dim0,dim1,dim2,dim3;;
    int bufid = TENSOR_START + 0;
    if (testid == 1)
        bufid = TENSOR_START + 1 ;

    get_buffer_shape(inputs,bufid,&nb_dims,&dim0,&dim1,&dim2,&dim3);
    std::vector<BufferDescription> desc = {BufferDescription(Shape(dim0,dim1)
                                                            ,kIMG_GRAY8_TYPE)
                                          };

    outputs = create_write_buffer(desc,total_bytes);

    const uint8_t *src = Buffer<uint8_t>::read(inputs,bufid);
    uint8_t *dst = Buffer<uint8_t>::write(outputs,0);
    memcpy(dst,src,get_buffer_length(inputs,bufid));
}

void run_test(const unsigned char* inputs,
              const uint32_t testid,
              const uint32_t funcid,
              unsigned char* &wbuf,
              uint32_t &total_bytes)
{

    wbuf = nullptr;
    switch(funcid)
    {
        case 0:
            test_rgb(inputs,wbuf,total_bytes,testid);
            break;
        case 1:
            test_gray8(inputs,wbuf,total_bytes,testid);
            break;
    }

}

#endif