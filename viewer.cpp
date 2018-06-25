#include <iostream>
#include <fstream>
#include <cstdio>
#include <ctime>
#include <chrono>
#include <thread>

#include "camera.h"
#include <opencv2/opencv.hpp>

#define XRES 1280
#define YRES 1024

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    std::clock_t start;
    double duration1;  
    double duration2;  
    double duration3;    

    Camera camera("/dev/video1", XRES, YRES);

    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Image image;

    int wKey = -1;
    while(wKey == -1 )
    {
        start = std::clock();
        image = camera.captureFrame();
        duration1 = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        Mat img(YRES, XRES, CV_8UC3, image.data);
        duration2 = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        imshow("Display window", img);
        duration3 = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        cout<<"capture time: "<< duration1;
        cout<<" openCV Mat: "<< duration2;
        cout<<" display time: "<< duration3 << endl;
        wKey =  waitKey(1);
        
    }

    return 0;

}
