import D_mipicamera as Dcam
import time
import cv2 #sudo apt-get install python-opencv

def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

if __name__ == "__main__":
    try:
        camera = Dcam.mipi_camera()
        print("Open camera...")
        camera.init_camera()
        while cv2.waitKey(10) != 27:
            frame = camera.capture(encoding = 'i420')
            height = int(align_up(1080, 16))
            width = int(align_up(1920, 32))
            image = frame.as_array.reshape(int(height * 1.5), width)
            image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
            cv2.imshow("D-Cam", image)
            # Release memory
            camera.release_buffer(frame)
            del frame
        
        print("Close camera...")
        camera.close_camera()
    except Exception as e:
        print(e)
