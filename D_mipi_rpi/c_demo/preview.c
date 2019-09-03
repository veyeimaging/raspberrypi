#include "D_mipicam.h"
#include <linux/v4l2-controls.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define LOG(fmt, args...) fprintf(stderr, fmt "\n", ##args)

int main(int argc, char **argv) {
    CAMERA_INSTANCE camera_instance;
    int width = 0, height = 0;
    
    struct camera_interface cam_interface;
    cam_interface.camera_num = -1; //0or1 for CM
    LOG("Open camera...");
    int res = D_init_camera(&camera_instance,cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }
    width = 1920;
    height = 1080;

  
    LOG("Start preview...");
    PREVIEW_PARAMS preview_params = {
        .fullscreen = 0,             // 0 is use previewRect, non-zero to use full screen
        .opacity = 255,              // Opacity of window - 0 = transparent, 255 = opaque
        .window = {0, 0, 1280, 720}, // Destination rectangle for the preview window.
    };
    res = D_start_preview(camera_instance, &preview_params);
    if (res) {
        LOG("start preview status = %d", res);
        return -1;
    }
   
    usleep(1000 * 1000 * 10);
    LOG("Stop preview...");
    res = D_stop_preview(camera_instance);
    if (res) {
        LOG("stop preview status = %d", res);
    }

    LOG("Close camera...");
    res = D_close_camera(camera_instance);
    if (res) {
        LOG("close camera status = %d", res);
    }
    return 0;
}