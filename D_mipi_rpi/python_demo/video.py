import D_mipicamera as Dcam
import time
import ctypes
import numpy as np

def callback(data):
    buff = Dcam.buffer(data)
    print("one frame len %d" % buff.length)
    file = buff.userdata
    buff.as_array.tofile(file)
    return 0

if __name__ == "__main__":
    try:
        camera = Dcam.mipi_camera()
        print("Open camera...")
        camera.init_camera()
        file = open("test.h264", "wb")
        # Need keep py_object reference
        file_obj = ctypes.py_object(file)
        camera.start_video_stream(callback, file_obj)
        time.sleep(10)
        camera.stop_video_stream()
        file.close()
        print("Close camera...")
        camera.close_camera()
    except Exception as e:
        print(e)
