This is a SDK for veye mipi camera module. It include sdk、C demo、Python demo.

It contains three parts: C language SDK(libdmipicam.so),C language sample,Python language sample.

### Install support package
```
sudo apt-get update && sudo apt-get install libopencv-dev

sudo apt-get install python-opencv
```
### D-SDK
- interface：
opensource，D_mipicam.h

- compile：
```
./build
```
- install：
```
sudo install -m 644 ./libdmipicam.so /usr/lib/
```
### C sample
- preview

Dispaly real-time video to HDMI output

- preview-dualcam

Dispaly real-time video to HDMI output for dual cameras（RPI CM）

- video

H.264 format encoding and record to file.

- capture

Capture one jpeg image.

- video2stdout
H.264 format encoding and send to stdout,which could be used by pipe. It's like veye_raspivid -o -.
```
./video2stdout | nc -l -p 5000
```
- capture_yuv

Capture one yuv image.

- capture-dualcam

Capture jpeg images for dual cameras.

- yuv_stream

Shows how to get yuv stream.

- capture2opencv

Shows how to get yuv stream,transfer to opencv format and display it.

- qrcode_detection

Shows how to get yuv stream,transfer to opencv format and display it and detect QR code.

### Python sample
- preview.py

Dispaly real-time video to HDMI output

- capture.py

Capture one jpeg image.

- capture_yuv.py

capture_yuv

- video.py

H.264 format encoding and record to file.

- capture2opencv.py

Shows how to get yuv stream,transfer to opencv format and display it.
