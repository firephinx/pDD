#include <settings.h>

#ifdef USE_CUDA
Class CudaIntegralImage{
    //int *data;
    int *device_data;
    int rows;
    int cols;
    CudaIntegralImage(cv::Mat);
    ~CudaIntegralImage();
};
#endif
