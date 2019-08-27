#include "D_mipicam.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define LOG(fmt, args...) fprintf(stderr, fmt "\n", ##args)

FILE *fd;
int frame_count = 0;
int yuv_callback(BUFFER *buffer) {
    // This buffer will be automatically released after the callback function ends.
    // If you do a time-consuming operation here, it will affect other parts,
    // such as preview, video encoding(This issue may be fixed in the future).
    if (buffer->length) {
        if (TIME_UNKNOWN == buffer->pts) {
            // Frame data in the second half
        }
         LOG("buffer length = %d, pts = %llu, flags = 0x%X", buffer->length, buffer->pts, buffer->flags);
        frame_count++;
    }
    return 0;
}

int main(int argc, char **argv) {
    CAMERA_INSTANCE camera_instance;
    int count = 0;
    int width = 0, height = 0;
    struct camera_interface cam_interface;
    cam_interface.camera_num = 0;
    LOG("Open camera...");
    int res = D_init_camera(&camera_instance,cam_interface);
    if (res) {
        LOG("init camera status = %d", res);
        return -1;
    }

    width = 1920;
    height = 1080;

    // start raw callback
    LOG("Start yuv data callback...");
    res = D_start_yuv_stream(camera_instance, yuv_callback, NULL);
    if (res) {
        LOG("Failed to start raw data callback.");
        return -1;
    }

    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
    usleep(1000 * 1000 * 10);
    clock_gettime(CLOCK_REALTIME, &end);

    double timeElapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1000000000.0;
    LOG("Total frame count = %d", frame_count);
    LOG("TimeElapsed = %f", timeElapsed);
    // stop raw data callback
    LOG("Stop raw data callback...");
    D_stop_yuv_stream(camera_instance);

    LOG("Close camera...");
    res = D_close_camera(camera_instance);
    if (res) {
        LOG("close camera status = %d", res);
    }
    return 0;
}
