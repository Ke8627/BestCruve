/*==========================================================
 *  Filename:   demo1.cpp
 *  Author:     Weiqi Chen (https://wiki77777.github.io/)
 *  Version:    1.0
 *  Date:       2017-11-27
 *  Description:
 *      A demo of bestcurve. Provide sliders to change the
 *  position of car and aim. The car's direction and minimal
 *  radius are also support to change by the other sliders.
 *  Environment:
 *      C++11， OpenCV 2.4.13
 ===========================================================*/
#include "bestcruve.h"

static Car car;
static Point aim;
int SHOWALLIMGS;

void on_Trackbar(int, void *){
    double length; int angle;
    Mat img = Mat(IMGSIZE, IMGSIZE, CV_8UC3, Scalar::all(255));
    getLine(img,length,angle,car.center,car.direction,car.radius,aim,car.RS,SHOWALLIMGS);
    imshow("Result",img);
}

void sliders(void){
    car.center = Point(IMGSIZE/6,IMGSIZE/6);
    car.direction = 240;
    car.radius = IMGSIZE/10;
    car.RS = 1;
    SHOWALLIMGS = 0;
    namedWindow("Options", CV_WINDOW_FREERATIO|CV_GUI_NORMAL);
    namedWindow("Result", CV_WINDOW_AUTOSIZE|CV_GUI_NORMAL);
    aim = Point(IMGSIZE/2,IMGSIZE/2);
    createTrackbar("Aim.x", "Options", &(aim.x), IMGSIZE, on_Trackbar);
    createTrackbar("Aim.y", "Options", &(aim.y), IMGSIZE, on_Trackbar);
    createTrackbar("Car.direction", "Options", &car.direction, 360, on_Trackbar);
    createTrackbar("Car.x", "Options", &car.center.x, IMGSIZE, on_Trackbar);
    createTrackbar("Car.y", "Options", &car.center.y, IMGSIZE, on_Trackbar);
    createTrackbar("Car.radius", "Options", &car.radius, IMGSIZE, on_Trackbar);
    createTrackbar("Allow Retreat", "Options", &car.RS, 1, on_Trackbar);
    createTrackbar("Show All (UNRECOMMEND)", "Options", &SHOWALLIMGS, 1, on_Trackbar);
    on_Trackbar(0, 0);
    moveWindow("Options",IMGSIZE+10,0);
    moveWindow("Result",0,0);
    waitKey();
}
