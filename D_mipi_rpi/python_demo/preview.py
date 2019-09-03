import D_mipicamera as Dcam
import time

if __name__ == "__main__":
    try:
        camera = Dcam.mipi_camera()
        print("Open camera...")
        camera.init_camera(camera_interface = (0,-1,(0,0),(0,0)))
        print("Start preview...")
        camera.start_preview(fullscreen = False, window = (0, 0, 1280, 720))
        time.sleep(100)
        print("Stop preview...")
        camera.stop_preview()
        print("Close camera...")
        camera.close_camera()
    except Exception as e:
        print(e)

