import D_mipicamera as Dcam
import time

if __name__ == "__main__":
    try:
        camera = Dcam.mipi_camera()
        print("Open camera...")
        camera.init_camera()
        # print("Start preview...")
        # camera.start_preview(fullscreen = False, window = (0, 0, 1280, 720))
        
        frame = camera.capture(encoding = 'jpeg')
        frame.as_array.tofile("test.jpg")
        # Release memory
        camera.release_buffer(frame)
        del frame
        # print("Stop preview...")
        # camera.stop_preview()
        print("Close camera...")
        camera.close_camera()
    except Exception as e:
        print(e)
