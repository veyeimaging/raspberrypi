#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "D_mipicam.h"



#define LOG(fmt, args...) fprintf(stderr, fmt "\n", ##args)


void save_image(CAMERA_INSTANCE camera_instance, const char *name) {
    IMAGE_FORMAT fmt = {IMAGE_ENCODING_JPEG, 50};
    // The actual width and height of the IMAGE_ENCODING_RAW_BAYER format and the IMAGE_ENCODING_I420 format are aligned, 
    // width 32 bytes aligned, and height 16 byte aligned.
    BUFFER *buffer = D_capture(camera_instance, &fmt, 3000);
    if (!buffer) {
        LOG("capture timeout.");
        return;
    }
    FILE *file = fopen(name, "wb");
    fwrite(buffer->data, buffer->length, 1, file);
    fclose(file);
    D_release_buffer(camera_instance,buffer);
}

int capture(int camera_num){
    CAMERA_INSTANCE camera_instance;
    int width = 0, height = 0;
    char file_name[100];
    struct camera_interface cam_interface;
    cam_interface.camera_num = camera_num;
    LOG("Open camera...");

    int res =  D_init_camera(&camera_instance,cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }

    width = 1920;
    height = 1080;
    sprintf(file_name, "Dimage%d-%dx%d.jpg",camera_num, width, height);
    LOG("Capture image %s...", file_name);
    save_image(camera_instance, file_name);


    LOG("Close camera...");
    res = D_close_camera(camera_instance);
    if (res) {
        LOG("close camera status = %d", res);
    }
    return 0;
}
int main(int argc, char **argv) {
   
    capture(0);
    capture(1);
    return 0;
}
