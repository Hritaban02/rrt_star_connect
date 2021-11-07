CFLAGS = -I/usr/include -L/usr/lib/ -g
LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_stitching -lopencv_videoio -lopencv_imgcodecs
CC = g++ -std=c++11
% : %.cpp
	$(CC) $(CFLAGS) -o $@ $< $(LIBS)
