CC = g++
CFLAGS = -ggdb -std=c++11
SRCS = grab.cpp camera.cpp
PROG = grab

#OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

all: $(PROG)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)  -lv4l2

clean:
	$(RM) *.o
	$(RM) $(PROG)
