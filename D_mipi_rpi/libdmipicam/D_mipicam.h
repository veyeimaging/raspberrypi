#ifndef _D_MIPI_CAMERA_H__
#define _D_MIPI_CAMERA_H__

#ifdef __cplusplus
extern "C" {
#endif
/*
    video streaming、 yuv streaming 、capture、preview ,
    Now these four functions can only use one at a time.
    [will be fixed later,by using splitter component]
*/

#include <stdint.h>
#define FOURCC(a, b, c, d) ((a) | (b << 8) | (c << 16) | (d << 24))

#define IMAGE_ENCODING_I420 FOURCC('I', '4', '2', '0')
#define IMAGE_ENCODING_I422 FOURCC('I', '4', '2', '2') //reserved
#define IMAGE_ENCODING_JPEG FOURCC('J', 'P', 'E', 'G')
#define IMAGE_ENCODING_BMP FOURCC('B', 'M', 'P','')
#define IMAGE_ENCODING_PNG FOURCC('P', 'N', 'G','')
#define VIDEO_ENCODING_H264 FOURCC('H', '2', '6', '4')


#define OUTPUT_FLAG_KEEP_BUFFER_REQUIREMENTS 0x08
#define OUTPUT_FLAG_BUFFER_ALLOCATION_USE_MMAL_CORE 0x10

#define VIDEO_LEVEL_H264_4 0x1C
#define VIDEO_LEVEL_H264_41 0x1D
#define VIDEO_LEVEL_H264_42 0x1E

#define VIDEO_PROFILE_H264_BASELINE 0x19
#define VIDEO_PROFILE_H264_MAIN 0x1A
#define VIDEO_PROFILE_H264_HIGH 0x1C

#define VIDEO_INTRA_REFRESH_CYCLIC 0x00
#define VIDEO_INTRA_REFRESH_ADAPTIVE 0x01
#define VIDEO_INTRA_REFRESH_BOTH 0x02
#define VIDEO_INTRA_REFRESH_CYCLIC_MROWS 0x7F000001
/** \name Special Unknown Time Value
 * Timestamps in MMAL are defined as signed 64 bits integer values representing microseconds.
 * However a pre-defined special value is used to signal that a timestamp is not known. */
/* @{ */
#define TIME_UNKNOWN (INT64_C(1) << 63) /**< Special value signalling that time is not known */
/* @} */

/** \name Buffer header flags
 * \anchor bufferheaderflags
 * The following flags describe properties of a buffer header */
/* @{ */
#ifndef MMAL_H
/** Signals that the current payload is the end of the stream of data */
#define MMAL_BUFFER_HEADER_FLAG_EOS (1 << 0)
/** Signals that the start of the current payload starts a frame */
#define MMAL_BUFFER_HEADER_FLAG_FRAME_START (1 << 1)
/** Signals that the end of the current payload ends a frame */
#define MMAL_BUFFER_HEADER_FLAG_FRAME_END (1 << 2)
/** Signals that the current payload contains only complete frames (1 or more) */
#define MMAL_BUFFER_HEADER_FLAG_FRAME (MMAL_BUFFER_HEADER_FLAG_FRAME_START | MMAL_BUFFER_HEADER_FLAG_FRAME_END)
/** Signals that the current payload is a keyframe (i.e. self decodable) */
#define MMAL_BUFFER_HEADER_FLAG_KEYFRAME (1 << 3)
/** Signals a discontinuity in the stream of data (e.g. after a seek).
 * Can be used for instance by a decoder to reset its state */
#define MMAL_BUFFER_HEADER_FLAG_DISCONTINUITY (1 << 4)
/** Signals a buffer containing some kind of config data for the component
 * (e.g. codec config data) */
#define MMAL_BUFFER_HEADER_FLAG_CONFIG (1 << 5)
/** Signals an encrypted payload */
#define MMAL_BUFFER_HEADER_FLAG_ENCRYPTED (1 << 6)
/** Signals a buffer containing side information */
#define MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO (1 << 7)
/** Signals a buffer which is the snapshot/postview image from a stills capture */
#define MMAL_BUFFER_HEADER_FLAGS_SNAPSHOT (1 << 8)
/** Signals a buffer which contains data known to be corrupted */
#define MMAL_BUFFER_HEADER_FLAG_CORRUPTED (1 << 9)
/** Signals that a buffer failed to be transmitted */
#define MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED (1 << 10)
/** Signals the output buffer won't be used, just update reference frames */
#define MMAL_BUFFER_HEADER_FLAG_DECODEONLY (1 << 11)
/** Signals that the end of the current payload ends a NAL */
#define MMAL_BUFFER_HEADER_FLAG_NAL_END (1 << 12)
#endif
/* @} */

typedef struct {
    uint32_t encoding;
    int quality; // JPEG quality setting (1-100)
} IMAGE_FORMAT;

/** Describes a rectangle */
typedef struct {
    int32_t x;      /**< x coordinate (from left) */
    int32_t y;      /**< y coordinate (from top) */
    int32_t width;  /**< width */
    int32_t height; /**< height */
} RECTANGLE;

typedef struct {
    int fullscreen;   // 0 is use previewRect, non-zero to use full screen
    int opacity;      // Opacity of window - 0 = transparent, 255 = opaque
    RECTANGLE window; // Destination rectangle for the preview window.
} PREVIEW_PARAMS;

typedef struct
{
    uint32_t encoding;         /// Requested codec video encoding (MJPEG or H264)
    int bitrate;               /// Requested bitrate
    int intraperiod;           /// Intra-refresh period (key frame rate)
    int quantisationParameter; /// Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
    int bInlineHeaders;        /// Insert inline headers to stream (SPS, PPS)
    int immutableInput;        /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
                               /// the camera output or the encoder output (with compression artifacts)
    int profile;               /// H264 profile to use for encoding
    int level;                 /// H264 level to use for encoding

    int inlineMotionVectors; /// Encoder outputs inline Motion Vectors
    int intra_refresh_type;  /// What intra refresh type to use. -1 to not set.
    int addSPSTiming;        /// 0 or 1
    int slices;              /// Horizontal slices per frame. Default 1 (off)
} VIDEO_ENCODER_STATE;

typedef struct {
    void *priv; /**< This is private data, please don't change it. */
    uint8_t *data;
    uint32_t alloc_size; /**< Allocated size in bytes of payload buffer */
    uint32_t length;     /**< Number of bytes currently used in the payload buffer (starting
                                   from offset) */
    uint32_t flags;      /**< Flags describing properties of a buffer header (see
                                   \ref bufferheaderflags "Buffer header flags") */

    int64_t pts;    /**< Presentation timestamp in microseconds. \ref TIME_UNKNOWN
                                   is used when the pts is unknown. */
    void *userdata; /**< Field reserved for use by the client */
} BUFFER;

struct format {
    int width;
    int height;
    int framerate; 
};
// i2c is reserved,because we use shell script to do do param setting
struct camera_interface {
    int i2c_bus;        // /dev/i2c-0  or /dev/i2c-1
    int camera_num;     // mipi interface num,-1 for mode B, 0,1 for mode CM the only used param in this struct
    int sda_pins[2];    // enable sda_pins[camera_num], disable sda_pins[camera_num ? 0 : 1]
    int scl_pins[2];    // enable scl_pins[camera_num], disable scl_pins[camera_num ? 0 : 1]

};

// note The buffer will be automatically released after the callback function ends.
typedef int (*OUTPUT_CALLBACK)(BUFFER *buffer);

typedef void *CAMERA_INSTANCE;


/**
 * init camera.
 * @param camera_instance Pointer of type CAMERA_INSTANCE, use to obtain CAMERA_INSTANCE.
 * @param camera_num Camera interface num.
 * @return error code , 0 success, !0 error.
 * 
 * example:
 @code
    CAMERA_INStANCE camera_instance;
    D_init_camera(&camera_instance, 0);
 @endcode
 @note Some boards have multiple camera interfaces.
 * */
int D_init_camera(CAMERA_INSTANCE *camera_instance, struct camera_interface cam_interface);

/**
 * init camera.
 * @param camera_instance Pointer of type CAMERA_INSTANCE, use to obtain CAMERA_INSTANCE.
 * @param camera_num Camera interface num.
 * @return error code , 0 success, !0 error.
 * 
 * example:
 @code
    CAMERA_INStANCE camera_instance;
    D_init_camera(&camera_instance, 0,&videofmt);
 @endcode
 @note Some boards have multiple camera interfaces.
 * */
int D_init_camera_ex(CAMERA_INSTANCE *camera_instance, struct camera_interface cam_interface,struct format* pvideofmt);

/**
 * Start video stream, H264
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @param encoder_state Used to specify encoding parameters. Use default parameters if NULL.
 * @param callback Callback method, this method will be called when there is data return.
 * @param userdata Userdata, which will be a member of the buffer parameter in the callback function.
 * @return error code , 0 success, !0 error.
 @endcode
 @note The buffer will be automatically released after the callback function ends.
 * */
int D_start_video_stream(CAMERA_INSTANCE camera_instance, VIDEO_ENCODER_STATE *encoder_state, OUTPUT_CALLBACK callback, void *userdata);

/**
 * Stop video stream, H264
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @return error code , 0 success, !0 error.
 @endcode
 @note
 * */
int D_stop_video_stream(CAMERA_INSTANCE camera_instance);

/**
 * Start YUV stream, yuv420
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @param callback Callback method, this method will be called when there is data return.
 * @param userdata Userdata, which will be a member of the buffer parameter in the callback function.
 * @return error code , 0 success, !0 error.
 @endcode
 @note If you do a time-consuming operation in the callback, it will affect other parts, such as preview, video encoding(This issue may be fixed in the future).
 @note The buffer will be automatically released after the callback function ends.
 * */
int D_start_yuv_stream(CAMERA_INSTANCE camera_instance, OUTPUT_CALLBACK callback, void *userdata);

/**
 * Stop yuv stream
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @return error code , 0 success, !0 error.
 @endcode
 @note
 * */
int D_stop_yuv_stream(CAMERA_INSTANCE camera_instance);

/**
 * Get single frame data.
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @param format The data format to be obtained.
 * @param timeout This method will return NULL if no data is obtained at this time. ms
 * @return BUFFER structure pointer containing image data.
 @note Currently supported image formats are:
    IMAGE_ENCODING_JPEG, IMAGE_ENCODING_I420,IMAGE_ENCODING_BMP,IMAGE_ENCODING_PNG
 @note When the buffer is used, you need to call the D_release_buffer function to release it.
 @note The actual width and height of the raw bayer format and the yuv420 format are aligned, width 32 bytes aligned, and height 16 byte aligned.
 * */
BUFFER *D_capture(CAMERA_INSTANCE camera_instance, IMAGE_FORMAT *format, int timeout);

/**
 * Used to release the memory occupied by the buffer.
 * 
 * @param buffer The buffer to be released.
 * */
void D_release_buffer(CAMERA_INSTANCE camera_instance,BUFFER *buffer);

/**
 * Turn on image preview
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @param preview_params Preview parameter,Use default parameters if NULL.
 * @return error code , 0 success, !0 error.
 * */
int D_start_preview(CAMERA_INSTANCE camera_instance, PREVIEW_PARAMS *preview_params);

/**
 * Turn off image preview
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @return error code , 0 success, !0 error.
 * */
int D_stop_preview(CAMERA_INSTANCE camera_instance);

/**
 * Release all resources and turn off the camera.
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @return error code , 0 success, !0 error.
 * */
int D_close_camera(CAMERA_INSTANCE camera_instance);

/**
 * Get the resolution supported by the current camera
 * 
 * @param camera_instance Type CAMERA_INSTANCE, Obtained from D_init_camera function.
 * @param fmt Used to return resolution parameters.
 * @param index Format list index.
 * @return error code , 0 success, !0 error.
 * */
int D_get_support_formats(CAMERA_INSTANCE camera_instance, struct format *fmt, int index);

#ifdef __cplusplus
}
#endif

#endif
