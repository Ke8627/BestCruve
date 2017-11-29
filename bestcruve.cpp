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
#include <sstream>

#define DISPLAYMSG
#define a2r(A) A*M_PI/180.0
#define r2a(R) R*180.0/M_PI
#define STEP 1
//#define SHOWALLIMGS //Only support Reeds-Sheep mode
                    //Severely reduce fluency!

using namespace std;

void arrow(Mat& img, Point p, int alpha, int len, const Scalar& color, int thickness = 1, int lineType = 8){
    Point pStart, pEnd;
    double angle = a2r(alpha);
    pStart.x = p.x - len*cos(angle);
    pStart.y = p.y - len*sin(angle);
    pEnd.x = p.x + len*cos(angle);
    pEnd.y = p.y + len*sin(angle);
    line(img,pStart,pEnd,color,thickness,lineType);
    if(alpha>45)
        line(img,pEnd,Point(pEnd.x - len*cos(abs(a2r(45)-angle))/2, pEnd.y - len*sin(abs(a2r(45)-angle))/2),
             color,thickness,lineType);
    else
        line(img,pEnd,Point(pEnd.x - len*cos(abs(a2r(45)-angle))/2, pEnd.y + len*sin(abs(a2r(45)-angle))/2),
             color,thickness,lineType);
    line(img,pEnd,Point(pEnd.x - len*cos(abs(a2r(-45)-angle))/2, pEnd.y - len*sin(abs(a2r(-45)-angle))/2),
         color,thickness,lineType);
}

int Dubins(Mat& img, double& length, int& angle, Point& carP, int car_direction, int radius, Point&aim, bool C0L = false){
    int ret;
    Mat leftMat,rightMat;
    int leftLength=0, rightLength=0, leftRot = 0, rightRot = 0;
    img.copyTo(leftMat); img.copyTo(rightMat);
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    for(int i=-1;i<=1;i+=2){
        int direction = car_direction, x = carP.x, y = carP.y;
        Point2f centerTurn;
        centerTurn.x = x + i*radius * cos(a2r(90)-a2r(direction));
        centerTurn.y = y - i*radius * sin(a2r(90)-a2r(direction));
        circle(i==-1?leftMat:rightMat,centerTurn,3,Scalar(0),-1);
        line(i==-1?leftMat:rightMat,Point(x,y),centerTurn,Scalar(0),1);
        double beita = theta - direction;
        int cu = 0;
        while(1){
            if(abs(beita) < STEP) break;
            ellipse(i==-1?leftMat:rightMat,centerTurn,Size(radius,radius),direction+i*90,0,-i*STEP,Scalar(0),2);
            direction -= C0L?-i:i*STEP;
            if(direction >= 360)  direction = 0;
            if(direction <= -1)  direction = 359;
            x = centerTurn.x + radius * cos(a2r(direction)+i*a2r(90));
            y = centerTurn.y + radius * sin(a2r(direction)+i*a2r(90));
            theta = r2a(atan2(aim.y-y,aim.x-x));
            if(theta<0) theta+=360;
            beita = theta - direction;
            if(cu++>360/STEP) break;
        }
        if(cu++<270/STEP) ret = cu*STEP;
        (i==-1?leftRot:rightRot) = direction;
        (i==-1?leftLength:rightLength) = (M_PI * radius)*cu*STEP/180.0 + sqrt(pow(y - aim.y,2) + pow(x - aim.x,2));
        line(i==-1?leftMat:rightMat,Point(x,y),centerTurn,Scalar(0),1);
        line(i==-1?leftMat:rightMat,Point(x,y),aim,Scalar(0),2);
    }
    if(leftLength<rightLength){
        length = leftLength; angle = leftRot;
        leftMat.copyTo(img);
    }
    else{
        length = rightLength; angle = rightRot;
        rightMat.copyTo(img);
    }
#ifdef DISPLAYMSG
    stringstream ss;    ss << "MinRadius:" << radius << " ";
    string lengthStr;   ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
    ss << "Length:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
#endif
    return ret;
}

void ReedsSheep(Mat& img, double& length, int& angle, Point& carP, int car_direction, int radius, Point&aim){
    struct path{
        double length; int angle; Mat img;
    };
#ifdef SHOWALLIMGS
    path FF,RR,RF,FR;   //F=forward,R=retreat
    img.copyTo(FF.img); img.copyTo(RR.img);
    img.copyTo(RF.img); img.copyTo(FR.img);
    img = Mat(IMGSIZE*2,IMGSIZE*2,img.type());

    Dubins(FF.img,FF.length,FF.angle,carP,car_direction,radius,aim);
    Dubins(RR.img,RR.length,RR.angle,carP,car_direction>180?car_direction-180:car_direction+180,radius,aim);
    Dubins(RF.img,RF.length,RF.angle,carP,car_direction,radius,aim,true);
    Dubins(FR.img,FR.length,FR.angle,carP,car_direction>180?car_direction-180:car_direction+180,radius,aim,true);

    length = FF.length; angle = FF.angle;
    int bestCurve = 0;
    double *lengths[3] = {&RR.length,&RF.length,&FR.length};
    int *angles[3] = {&RR.angle,&RF.angle,&FR.angle};
    for(int i=0;i<3;i++){
        if(*lengths[i]<length){
            bestCurve = i+1; length = *lengths[i]; angle = *angles[i];
        }
    }

    putText(FF.img,"1.FF",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 0?255:0),IMGSIZE/300.0);
    Mat imageROI= img(Rect(0,0,IMGSIZE,IMGSIZE));
    FF.img.copyTo(imageROI);

    putText(RR.img,"2.RR",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 1?255:0),IMGSIZE/300.0);
    imageROI= img(Rect(IMGSIZE,0,IMGSIZE,IMGSIZE));
    RR.img.copyTo(imageROI);

    putText(RF.img,"3.RF",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 2?255:0),IMGSIZE/300.0);
    imageROI= img(Rect(0,IMGSIZE,IMGSIZE,IMGSIZE));
    RF.img.copyTo(imageROI);

    putText(FR.img,"4.FR",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 3?255:0),IMGSIZE/300.0);
    imageROI= img(Rect(IMGSIZE,IMGSIZE,IMGSIZE,IMGSIZE));
    FR.img.copyTo(imageROI);

    line(img,Point(0,IMGSIZE),Point(IMGSIZE*2,IMGSIZE),Scalar(0),2);
    line(img,Point(IMGSIZE,0),Point(IMGSIZE,IMGSIZE*2),Scalar(0),2);
    resize(img,img,Size(IMGSIZE,IMGSIZE));
#else
    path F,R;
    img.copyTo(F.img); img.copyTo(R.img);
    if(Dubins(F.img,F.length,F.angle,carP,car_direction,radius,aim) > 270){
        img.copyTo(F.img);
        Dubins(F.img,F.length,F.angle,carP,car_direction,radius,aim,true);
    }
    if(Dubins(R.img,R.length,R.angle,carP,car_direction>180?car_direction-180:car_direction+180,radius,aim) > 270){
        img.copyTo(R.img);
        Dubins(R.img,R.length,R.angle,carP,car_direction>180?car_direction-180:car_direction+180,radius,aim,true);
    }
    R.angle += 180;
    if(R.angle>360) R.angle -= 360;
    if(F.length < R.length){
        length = F.length; angle = F.angle; F.img.copyTo(img);
    }
    else{
        length = R.length; angle = R.angle; R.img.copyTo(img);
    }
#endif
}

void getLine(Mat& img, double& length, int& angle, Point& car, int car_direction, int radius, Point&aim, bool RS = true){
    Rect aimR;
    aimR.x = aim.x - IMGSIZE/50;   aimR.y = aim.y - IMGSIZE/50;
    aimR.width = aimR.height = IMGSIZE/25;
    rectangle(img,aimR,Scalar(0,0,255),2);
    arrow(img,car,car_direction,IMGSIZE/20,Scalar(200,200,50),2);
    if(RS == false)
        Dubins(img,length,angle,car,car_direction,radius,aim);
    else
        ReedsSheep(img,length,angle,car,car_direction,radius,aim);
}
