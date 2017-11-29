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
#include <iostream>
#include <random>

#define AIM_NUM 5

using namespace std;

static Car car;
static struct aims{
    Point center; int radius;
}cirs[AIM_NUM];

void continuous(void){
    random_device rd;
    car.center = Point(rd()%600,rd()%600);
    car.direction = rd()%360;
    car.radius = IMGSIZE/10;
    car.RS = 1;

    double length; int angle;
    Mat img = Mat(IMGSIZE, IMGSIZE, CV_8UC3, Scalar::all(255));
    for(int i=0;i<AIM_NUM;i++){
        cirs[i].center = Point(rd()%600,rd()%600);
        cirs[i].radius = rd()%20+10;
        getLine(img,length,angle,car.center,car.direction,car.radius,cirs[i].center,car.RS);
        car.center = cirs[i].center;
        car.direction = angle;
    }

    imshow("Result",img);

    waitKey();
}
