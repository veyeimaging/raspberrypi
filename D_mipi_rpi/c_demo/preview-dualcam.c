#include "D_mipicam.h"
#include <linux/v4l2-controls.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define LOG(fmt, args...) fprintf(stderr, fmt "\n", ##args)

int main(int argc, char **argv) {
    CAMERA_INSTANCE camera_instance, camera_instance2;
    int width = 0, height = 0;
    LOG("Open camera...");
    // more information: https://www.raspberrypi.org/documentation/hardware/computemodule/cmio-camera.md
    struct camera_interface cam_interface = {
        .i2c_bus = 0,           // /dev/i2c-0  or /dev/i2c-1   
        .camera_num = 0,        // mipi interface num
    };
    int res = D_init_camera(&camera_instance, cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }

    LOG("Start preview...");
    PREVIEW_PARAMS preview_params = {
        .fullscreen = 0,             // 0 is use previewRect, non-zero to use full screen
        .opacity = 255,              // Opacity of window - 0 = transparent, 255 = opaque
        .window = {0, 0, 640, 480}, // Destination rectangle for the preview window.
    };
    res = D_start_preview(camera_instance, &preview_params);
    if (res) {
        LOG("start preview status = %d", res);
        return -1;
    }

    // Important: Using init_camera will invalidate the read/write I2C of another camera_instance, 
    // ie the control command is invalid and the resolution cannot be switched(autoexposure is also unusable).
    // This problem will be fixed in the future.
    LOG("Open camera...");
    cam_interface.camera_num = 1;
    res = D_init_camera(&camera_instance2, cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }
  
    LOG("Start preview...");
    PREVIEW_PARAMS preview_params2 = {
        .fullscreen = 0,             // 0 is use previewRect, non-zero to use full screen
        .opacity = 255,              // Opacity of window - 0 = transparent, 255 = opaque
        .window = {0, 480, 640, 480}, // Destination rectangle for the preview window.
    };
    res = D_start_preview(camera_instance2, &preview_params2);
    if (res) {
        LOG("start preview status = %d", res);
        return -1;
    }


    usleep(1000 * 1000 * 20);
    LOG("Stop preview...");
    res = D_stop_preview(camera_instance);
    if (res) {
        LOG("stop preview status = %d", res);
    }
    LOG("Stop preview...");
    res = D_stop_preview(camera_instance2);
    if (res) {
        LOG("stop preview status = %d", res);
    }
    LOG("Close camera...");
    res = D_close_camera(camera_instance);
    if (res) {
        LOG("close camera status = %d", res);
    }
    res = D_close_camera(camera_instance2);
    if (res) {
        LOG("close camera status = %d", res);
    }
    return 0;
}