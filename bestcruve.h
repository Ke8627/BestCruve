#ifndef BESTCRUVE_H
#define BESTCRUVE_H

#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

#define IMGSIZE 600

#define DISPLAYMSG
#define a2r(A) A*M_PI/180.0
#define r2a(R) R*180.0/M_PI
#define STEP 1
//#define SHOWALLIMGS //Only support Reeds-Sheep mode
                    //Severely reduce fluency!

struct Car{
    Point center;
    int direction;
    int radius;
    int RS;
};

struct Path{
    double length; int angle; Mat img; int cu;
};

int CL_FL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int CL_FR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int CL_BL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int CL_BR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int Dubins(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim, bool showall);

int CCL_FL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int CCL_FR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int CCL_BL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int CCL_BR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim);
int ReedsSheep(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim, bool showall);

void getLine(Mat& img, double& length, int& angle, Point car, int car_direction, int radius, Point aim, bool RS, bool showall);

#endif // BESTCRUVE_H

