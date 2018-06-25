#include <iostream>
#include <fstream>

#include "camera.h"

#define XRES 1280
#define YRES 1024

using namespace std;

int main(int argc, char **argv)
{

    Camera camera("/dev/video1", XRES, YRES);
    auto frame = camera.captureFrame();

    ofstream image;
    image.open("frame.ppm");
    image << "P6\n"
          << XRES << " " << YRES << " 255\n";
    image.write((char *)frame.data, frame.size);
    image.close();

    return 0;
}
