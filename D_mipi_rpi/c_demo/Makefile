CROSS_COMPILE 	?=
CROSS_PREFIX	?=
CC		:= $(CROSS_COMPILE)gcc
CXX		:= $(CROSS_COMPILE)g++
CFLAGS 	?= -I. -I../libdmipicam  -g -O0 -std=gnu11
CXXFLAGS?= -I. -I../libdmipicam -g -std=gnu++11 `pkg-config --cflags --libs opencv`
LDFLAGS	?=
LIBS	:= -L ../libdmipicam/ -ldmipicam
OLIB	:= ../libdmipicam
examples:= preview capture capture_yuv video list_format yuv_stream  preview-dualcam capture-dualcam video2stdout capture2opencv qrcode_detection  
%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $<

all: $(examples)

preview: preview.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

capture: capture.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)
    
capture_yuv: capture_yuv.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)
    
video: video.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

list_format: list_format.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

yuv_stream: yuv_stream.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

video2stdout : video2stdout.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

capture2opencv : capture2opencv.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

qrcode_detection: qrcode_detection.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS) -lzbar

preview-dualcam : preview-dualcam.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

capture-dualcam : capture-dualcam.c
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ $^ $(LIBS)

clean:
	-rm -f *.o
	-rm -f $(examples)

.PHONY: install

install: 
	sudo install -m 644 $(OLIB)/libdmipicam.so /usr/lib/

