/*==========================================================
 *  Filename:   bestcruve.cpp
 *  Author:     Weiqi Chen (https://wiki77777.github.io/)
 *  Version:    1.1
 *  Date:       2017-11-27
 *  Description:
 *      Generated the best cruve for a Reeds-Shepp car in
 *  no pose required situation.
 *  Environment:
 *      C++11， OpenCV 2.4.13
 ===========================================================*/

#include "bestcruve.h"

void arrow(Mat& img, Point p, int alpha, int len, const Scalar& color, int thickness = 1, int lineType = 8){
    Point pStart, pEnd;
    double angle = a2r(alpha);
    pStart.x = p.x + len*cos(angle);
    pStart.y = p.y + len*sin(angle);
    pEnd.x = p.x - len*cos(angle);
    pEnd.y = p.y - len*sin(angle);
    line(img,pStart,pEnd,color,thickness,lineType);
    if(alpha>45)
        line(img,pEnd,Point(pEnd.x + len*cos(abs(a2r(45)-angle))/2, pEnd.y + len*sin(abs(a2r(45)-angle))/2),
             color,thickness,lineType);
    else
        line(img,pEnd,Point(pEnd.x + len*cos(abs(a2r(45)-angle))/2, pEnd.y - len*sin(abs(a2r(45)-angle))/2),
             color,thickness,lineType);
    line(img,pEnd,Point(pEnd.x + len*cos(abs(a2r(-45)-angle))/2, pEnd.y + len*sin(abs(a2r(-45)-angle))/2),
         color,thickness,lineType);
}

void getLine(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim, bool RS = true, bool showall = true){
    Rect aimR;
    aimR.x = aim.x - IMGSIZE/50;   aimR.y = aim.y - IMGSIZE/50;
    aimR.width = aimR.height = IMGSIZE/25;
    rectangle(img,aimR,Scalar(0,0,255),2);
    arrow(img,carP,car_direction,IMGSIZE/20,Scalar(200,200,50),2);
    if(RS == false)
        Dubins(img,length,angle,carP,car_direction,radius,aim,showall);
    else
        ReedsSheep(img,length,angle,carP,car_direction,radius,aim,showall);
}
