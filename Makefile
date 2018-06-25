CC = g++
CFLAGS = -ggdb -std=c++11
GRAB_SRCS = grab.cpp
VIEWER_SRCS = viewer.cpp
COMMON_SRCS = camera.cpp

PROG = grab

OPENCV = `pkg-config opencv --cflags --libs`
#LIBS = $(OPENCV)

all: $(PROG)

$(PROG): $(GRAB_SRCS) $(COMMON_SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(GRAB_SRCS) $(COMMON_SRCS)  -lv4l2

viewer: $(VIEWER_SRCS) $(COMMON_SRCS)
	$(CC) $(CFLAGS) -o viewer $(VIEWER_SRCS) $(COMMON_SRCS) $(OPENCV)  -lv4l2

clean:
	$(RM) *.o
	$(RM) $(PROG)
	$(RM) viewer
