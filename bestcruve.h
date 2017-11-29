#ifndef BESTCRUVE_H
#define BESTCRUVE_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;

#define IMGSIZE 600

struct Car{
    Point center;
    int direction;
    int radius;
    int RS;
};

void getLine(Mat& img, double& length, int& angle, Point& car, int car_direction, int radius, Point& aim, bool RS);

#endif // BESTCRUVE_H

