'''
This script is a wrapper for the libdmipicam.so dynamic library. 
To use this script you need to pre-install libdmipicam.so
'''
from ctypes import *
import numpy as np
import sys

try:
    camera_lib = cdll.LoadLibrary("libdmipicam.so")
except:
    print("Load libdmipicam fail.")
    sys.exit(0)


def FOURCC(a, b, c, d):
    return (ord(a) | (ord(b) << 8) | (ord(c) << 16) | (ord(d) << 24))

IMAGE_ENCODING_I420 = FOURCC('I', '4', '2', '0')
IMAGE_ENCODING_I422 = FOURCC('I', '4', '2', '2')      #reserved
IMAGE_ENCODING_JPEG = FOURCC('J', 'P', 'E', 'G')
IMAGE_ENCODING_RAW_BAYER = FOURCC('R', 'A', 'W', ' ')
IMAGE_ENCODING_BMP = FOURCC('B', 'M', 'P',' ')
IMAGE_ENCODING_PNG = FOURCC('P', 'N', 'G',' ')
VIDEO_ENCODING_H264 = FOURCC('H', '2', '6', '4')


image_encodings = {
    'i420' : IMAGE_ENCODING_I420,
    'i422' : IMAGE_ENCODING_I422,
    'jpeg' : IMAGE_ENCODING_JPEG,
    'raw' : IMAGE_ENCODING_RAW_BAYER,
    'bmp' : IMAGE_ENCODING_BMP,
    'png' : IMAGE_ENCODING_PNG,
}
video_encodings = {
    'h264' : VIDEO_ENCODING_H264,
}


# H264 level
VIDEO_LEVEL_H264_4                  = 0x1C
VIDEO_LEVEL_H264_41                 = 0x1D
VIDEO_LEVEL_H264_42                 = 0x1E
# H264 profile
VIDEO_PROFILE_H264_BASELINE         = 0x19
VIDEO_PROFILE_H264_MAIN             = 0x1A
VIDEO_PROFILE_H264_HIGH             = 0x1C

VIDEO_INTRA_REFRESH_CYCLIC          = 0x00
VIDEO_INTRA_REFRESH_ADAPTIVE        = 0x01
VIDEO_INTRA_REFRESH_BOTH            = 0x02
VIDEO_INTRA_REFRESH_CYCLIC_MROWS    = 0x7F000001

MMAL_TIME_UNKNOWN = c_int64(1<<63).value

MMAL_BUFFER_HEADER_FLAG_EOS                    = (1<<0)
MMAL_BUFFER_HEADER_FLAG_FRAME_START            = (1<<1)
MMAL_BUFFER_HEADER_FLAG_FRAME_END              = (1<<2)
MMAL_BUFFER_HEADER_FLAG_FRAME                  = (MMAL_BUFFER_HEADER_FLAG_FRAME_START|MMAL_BUFFER_HEADER_FLAG_FRAME_END)
MMAL_BUFFER_HEADER_FLAG_KEYFRAME               = (1<<3)
MMAL_BUFFER_HEADER_FLAG_DISCONTINUITY          = (1<<4)
MMAL_BUFFER_HEADER_FLAG_CONFIG                 = (1<<5)
MMAL_BUFFER_HEADER_FLAG_ENCRYPTED              = (1<<6)
MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO          = (1<<7)
MMAL_BUFFER_HEADER_FLAGS_SNAPSHOT              = (1<<8)
MMAL_BUFFER_HEADER_FLAG_CORRUPTED              = (1<<9)
MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED    = (1<<10)
MMAL_BUFFER_HEADER_FLAG_DECODEONLY             = (1<<11)
MMAL_BUFFER_HEADER_FLAG_NAL_END                = (1<<12)   

class FORMAT(Structure):
    _fields_ = [
        ("width",c_int),
        ("height",c_int),
		("maxframrate",c_int),
    ]

class IMAGE_FORMAT(Structure):
    _fields_ = [
        ("encoding",c_uint32),
        ("quality",c_int),
    ]

class RECTANGLE(Structure):
    _fields_ = [
        ("x",c_int32),
        ("y",c_int32),
        ("width",c_int32),
        ("height",c_int32),
    ]

class PREVIEW_PARAMS(Structure):
    _fields_ = [
        ("fullscreen",c_int),
        ("opacity",c_int),
        ("window",RECTANGLE),
    ]

class VIDEO_ENCODER_STATE(Structure):
    _fields_ = [
        ("encoding",c_uint32),              # Requested codec video encoding (MJPEG or H264)  
        ("bitrate",c_int),                  # Requested bitrate
        ("intraperiod",c_int),              # Intra-refresh period (key frame rate)
        ("quantisationParameter",c_int),    # Quantisation parameter - quality. Set bitrate 0 and set this for variable bitrate
        ("bInlineHeaders",c_int),           # Insert inline headers to stream (SPS, PPS)
        ("immutableInput",c_int),           # Not working               
        ("profile",c_int),                  # H264 profile to use for encoding
        ("level",c_int),                    # H264 level to use for encoding
        ("inlineMotionVectors",c_int),      # Encoder outputs inline Motion Vectors
        ("intra_refresh_type",c_int),       # What intra refresh type to use. -1 to not set.
        ("addSPSTiming",c_int),             # 0 or 1
        ("slices",c_int),                   # Horizontal slices per frame. Default 1 (off)
    ]

class BUFFER(Structure):
    _fields_ = [
        ("priv",c_void_p),
        ("data",POINTER(c_ubyte)),
        ("alloc_size",c_uint32),
        ("length",c_uint32),
        ("flags",c_uint32),
        ("pts",c_uint64),
        ("userdata",c_void_p),
    ]

class FORMAT(Structure):
    _fields_ = [
        ("width",c_int),
        ("height",c_int),
        ("framerate",c_uint32),
    ]

class CAMERA_INTERFACE(Structure):
    _fields_ = [
        ("i2c_bus",c_int),
        ("camera_num",c_int),
        ("sda_pins",c_int * 2),
        ("scl_pins",c_int * 2),
    ]


OUTPUT_CALLBACK = CFUNCTYPE(c_int, POINTER(BUFFER))

D_init_camera = camera_lib.D_init_camera
D_init_camera.argtypes = [POINTER(c_void_p), CAMERA_INTERFACE]
D_init_camera.restype = c_int

D_init_camera_ex = camera_lib.D_init_camera_ex
D_init_camera_ex.argtypes = [POINTER(c_void_p), CAMERA_INTERFACE, POINTER(FORMAT)]
D_init_camera_ex.restype = c_int

D_start_preview = camera_lib.D_start_preview
D_start_preview.argtypes = [c_void_p, POINTER(PREVIEW_PARAMS)]
D_start_preview.restype = c_int

D_stop_preview = camera_lib.D_stop_preview
D_start_preview.argtypes = [c_void_p]
D_start_preview.restype = c_int

D_capture = camera_lib.D_capture
D_capture.argtypes = [c_void_p, POINTER(IMAGE_FORMAT), c_int]
D_capture.restype = POINTER(BUFFER)

D_release_buffer = camera_lib.D_release_buffer
D_release_buffer.argtypes = [c_void_p,POINTER(BUFFER)]
D_release_buffer.restype = None

D_start_yuv_stream = camera_lib.D_start_yuv_stream
D_start_yuv_stream.argtypes = [c_void_p, OUTPUT_CALLBACK, c_void_p]
D_start_yuv_stream.restype = c_int

D_stop_yuv_stream = camera_lib.D_stop_yuv_stream
D_stop_yuv_stream.argtypes = [c_void_p]
D_stop_yuv_stream.restype = c_int

D_start_video_stream = camera_lib.D_start_video_stream
D_start_video_stream.argtypes = [c_void_p, POINTER(VIDEO_ENCODER_STATE), OUTPUT_CALLBACK, c_void_p]
D_start_video_stream.restype = c_int

D_stop_video_stream = camera_lib.D_stop_video_stream
D_stop_video_stream.argtypes = [c_void_p]
D_stop_video_stream.restype = c_int

D_get_support_formats = camera_lib.D_get_support_formats
D_get_support_formats.argtypes = [c_void_p, POINTER(FORMAT), c_int]
D_get_support_formats.restype = c_int

D_close_camera = camera_lib.D_close_camera
D_close_camera.argtypes = [c_void_p]
D_close_camera.restype = c_int

def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

class buffer(object):
    buffer_ptr = None
    def __init__(self, buff):
        if not isinstance(buff, POINTER(BUFFER)):
            raise TypeError("Expected parameter type is POINTER(BUFFER).")
        self.buffer_ptr = buff
        
    @property
    def as_array(self):
        return np.ctypeslib.as_array(self.buffer_ptr[0].data, shape=(self.length,))
    
    @property
    def data(self):
        return string_at(self.buffer_ptr[0].data,self.length)

    @property
    def length(self):
        return self.buffer_ptr[0].length
    @length.setter
    def length(self, value):
        self.buffer_ptr[0].length = value

    @property
    def alloc_size(self):
        return self.buffer_ptr[0].alloc_size
    @alloc_size.setter
    def alloc_size(self, value):
        self.buffer_ptr[0].alloc_size = value

    @property
    def flags(self):
        return self.buffer_ptr[0].flags
    @flags.setter
    def flags(self, value):
        self.buffer_ptr[0].flags = value

    @property
    def pts(self):
        return self.buffer_ptr[0].pts
    @pts.setter
    def pts(self, value):
        self.buffer_ptr[0].pts = value

    @property
    def userdata(self):
        if self.buffer_ptr[0].userdata == None:
            return None
        return cast(self.buffer_ptr[0].userdata, POINTER(py_object))[0]
        
    def release(self):
        return None
        #will release the buffer automaticly in SDK for stream 
        #will release the buffer manually in python for capture
        #D_release_buffer(self.buffer_ptr)

    def __del__(self):
        self.release()

def check_status(status,func_name):
    if status != 0:
        raise RuntimeError("{}: Unexpected result.".format(func_name))

class mipi_camera(object):
    
    def __init__(self):
        self.camera_instance = c_void_p(0)
    def init_camera(self, camera_interface=None):
        #//0or1 for CM
        cam_infe = CAMERA_INTERFACE(0,-1,(0,0),(0,0));
        if camera_interface is not None:
            try:
                cam_infe.i2c_bus, cam_infe.camera_num, (cam_infe.sda_pins[0], cam_infe.sda_pins[1]), (cam_infe.scl_pins[0], cam_infe.scl_pins[1]) = camera_interface
            except (TypeError, ValueError) as e:
                raise TypeError(
                    "Invalid camera_interface " )
        check_status(
            D_init_camera(byref(self.camera_instance), cam_infe),
            sys._getframe().f_code.co_name
        )
    def start_preview(self, fullscreen = True, opacity = 255, window = None):
        rect = RECTANGLE(0, 0, 640, 480)
        if window is not None:
            try:
                rect.x, rect.y, rect.width, rect.height = window
            except (TypeError, ValueError) as e:
                raise TypeError(
                    "Invalid window rectangle (x, y, w, h) tuple: %s" % window)
        preview_params = PREVIEW_PARAMS(int(fullscreen), opacity, rect)
        check_status(
            D_start_preview(self.camera_instance, byref(preview_params) if preview_params!=None else None),
            sys._getframe().f_code.co_name
        )
        
    def stop_preview(self):
        check_status(
            D_stop_preview(self.camera_instance),
            sys._getframe().f_code.co_name
        )
    def capture(self, time_out = 3000, encoding = 'jpeg', quality = 90):
        if image_encodings[encoding] == None:
            raise TypeError("Unknown image encoding type.")
        image_format = IMAGE_FORMAT(image_encodings[encoding], quality)
        return buffer(D_capture(self.camera_instance, byref(image_format), time_out))
        
    def release_buffer(self, buffer):
        #return buffer.release_buff()
        return D_release_buffer(self.camera_instance, buffer.buffer_ptr) 
    def start_yuv_stream(self, func = None, userdata = None):
        '''
        Important:You need to keep a reference to userdata. 
                  If userdata is released, using userdata in 
                  the callback function will crash the program.
        '''
        if userdata != None and not isinstance(userdata, py_object):
            raise TypeError("Userdata must be of type py_object")
        cfunc = OUTPUT_CALLBACK(func) if func != None else cast(None, OUTPUT_CALLBACK)
        check_status(
            D_start_yuv_stream(self.camera_instance, cfunc, byref(userdata) if userdata != None else None),
            sys._getframe().f_code.co_name
        )
    def stop_yuv_stream(self):
        check_status(
            D_stop_yuv_stream(self.camera_instance),
            sys._getframe().f_code.co_name
        )
    def start_video_stream(self, func = None, userdata = None, **kwargs):    
        '''
        Important:You need to keep a reference to userdata. 
                  If userdata is released, using userdata in 
                  the callback function will crash the program.
        '''
        if userdata != None and not isinstance(userdata, py_object):
            raise TypeError("Userdata must be of type py_object")

        cfunc = OUTPUT_CALLBACK(func) if func != None else cast(None, OUTPUT_CALLBACK)

        options = {
            'encoding': VIDEO_ENCODING_H264,
            'bitrate': 17000000,
            'intraperiod': -1,
            'quantisationParameter': 0,
            'bInlineHeaders': 0,
            'immutableInput': 1,
            'profile': VIDEO_PROFILE_H264_HIGH,
            'level': VIDEO_LEVEL_H264_4,
            'inlineMotionVectors': 0,
            'intra_refresh_type': -1,
            'addSPSTiming': 1,
            'slices':1,
            }
        for arg_name in options:
            options[arg_name] = kwargs.pop(arg_name, options[arg_name])
        video_state = VIDEO_ENCODER_STATE()

        for arg_name, _ in video_state._fields_:
            setattr(video_state, arg_name, options[arg_name])
        check_status(
            D_start_video_stream(
                self.camera_instance, byref(video_state), 
                cfunc, byref(userdata) if userdata != None else None
            ),
            sys._getframe().f_code.co_name
        )
    def stop_video_stream(self):
        check_status(
            D_stop_video_stream(self.camera_instance),
            sys._getframe().f_code.co_name
        )

    def get_support_formats(self):
        fmt = FORMAT()
        fmts = []
        i = 0
        while D_get_support_formats(self.camera_instance, byref(fmt), i) == 0:
            i += 1
            fmts.append((fmt.width, fmt.height))
        return fmts

    def close_camera(self):
        check_status(
            D_close_camera(self.camera_instance),
            sys._getframe().f_code.co_name
        )

pass
