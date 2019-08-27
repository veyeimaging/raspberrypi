#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "D_mipicam.h"

#include <opencv2/opencv.hpp>

#define VCOS_ALIGN_DOWN(p,n) (((ptrdiff_t)(p)) & ~((n)-1))
#define VCOS_ALIGN_UP(p,n) VCOS_ALIGN_DOWN((ptrdiff_t)(p)+(n)-1,(n))


#define LOG(fmt, args...) fprintf(stderr, fmt "\n", ##args)

// #define SOFTWARE_AE_AWB
int frame_count = 0;
cv::Mat *get_image(CAMERA_INSTANCE camera_instance, int width, int height) {
    IMAGE_FORMAT fmt = {IMAGE_ENCODING_I420, 50};
    BUFFER *buffer = D_capture(camera_instance, &fmt, 3000);
    if (!buffer) 
        return NULL;
    
    // The actual width and height of the IMAGE_ENCODING_RAW_BAYER format and the IMAGE_ENCODING_I420 format are aligned, 
    // width 32 bytes aligned, and height 16 byte aligned.
    width = VCOS_ALIGN_UP(width, 32);
    height = VCOS_ALIGN_UP(height, 16);
    cv::Mat *image = new cv::Mat(cv::Size(width,(int)(height * 1.5)), CV_8UC1, buffer->data);
    cv::cvtColor(*image, *image, cv::COLOR_YUV2BGR_I420);
    D_release_buffer(camera_instance,buffer);
    return image;
}

int main(int argc, char **argv) {
    CAMERA_INSTANCE camera_instance;
    int width = 1920, height = 1080;
    char file_name[100];
    struct camera_interface cam_interface;
    cam_interface.camera_num = 0;
    LOG("Open camera...");
    int res = D_init_camera(&camera_instance,cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }

    while(1){
        cv::Mat *image = get_image(camera_instance, width, height);
        if(!image)
            continue;
        cv::imshow("Dimage", *image);
        cv::waitKey(10);
        delete image;
    }

    LOG("Close camera...");
    res = D_close_camera(camera_instance);
    if (res) {
        LOG("close camera status = %d", res);
    }
    return 0;
}


