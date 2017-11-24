/*==========================================================
 *  Filename:   bestcruve.cpp
 *  Author:     Weiqi Chen (https://wiki77777.github.io/)
 *  Version:    1.0
 *  Date:       2017-11-24
 *  Description:
 *      Generated the best cruve for a Reeds-Shepp car in
 *  no pose required situation.
 *  Environment:
 *      C++11， OpenCV 2.4.13
 ===========================================================*/
#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

#define IMGSIZE 600
#define MIN_RADIUS 60
#define a2r(A) A*M_PI/180.0
#define r2a(R) R*180.0/M_PI
#define STEP 1
#define AIM_NUM 1
//#define SHOWALLIMGS //Only support Reeds-Sheep mode
                    //Severely reduce fluency!

struct Car{
    Point center;
    int rotation;
    int radius;
    int RS;
}car;
Point aim;

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

int Dubins(Mat& img, double& length, Point& carP, int car_rotation, int radius, Point&aim, bool C0L = false){
    int ret = 0;
    Mat leftMat,rightMat;
    int leftLength=0, rightLength=0;
    img.copyTo(leftMat); img.copyTo(rightMat);
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    for(int i=-1;i<=1;i+=2){
        double rotation = car_rotation, x = carP.x, y = carP.y;
        Point2f centerTurn;
        centerTurn.x = x + i*radius * cos(a2r(90)-a2r(rotation));
        centerTurn.y = y - i*radius * sin(a2r(90)-a2r(rotation));
        circle(i==-1?leftMat:rightMat,centerTurn,3,Scalar(0),-1);
        line(i==-1?leftMat:rightMat,Point(x,y),centerTurn,Scalar(0),1);
        int cu = 0;
        double beita = theta - rotation;
        while(1){
            if(abs(beita) < STEP) break;
            ellipse(i==-1?leftMat:rightMat,centerTurn,Size(radius,radius),rotation+i*90,0,-i*STEP,Scalar(0),2);
            rotation -= C0L?-i:i*STEP;
            if(rotation >= 360)  rotation = 0;
            if(rotation <= -1)  rotation = 359;
            x = centerTurn.x + radius * cos(a2r(rotation)+i*a2r(90));
            y = centerTurn.y + radius * sin(a2r(rotation)+i*a2r(90));
            theta = r2a(atan2(aim.y-y,aim.x-x));
            if(theta<0) theta+=360;
            beita = theta - rotation;
            if(cu++>360/STEP) break;
        }
        if(cu<270/STEP) ret = cu;
        (i==-1?leftLength:rightLength) = (M_PI * radius)*STEP*cu/180.0 + \
                sqrt(pow(y - aim.y,2) + pow(x - aim.x,2));
        line(i==-1?leftMat:rightMat,Point(x,y),centerTurn,Scalar(0),1);
        line(i==-1?leftMat:rightMat,Point(x,y),aim,Scalar(0),2);
    }
    if(leftLength<rightLength){
        length = leftLength;
        leftMat.copyTo(img);
    }
    else{
        length = rightLength;
        rightMat.copyTo(img);
    }
    flip(img,img,0);
    stringstream ss;    ss << "MinRadius:" << radius << " ";
    string lengthStr;   ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
    ss << "Length:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
    return ret;
}

void ReedsSheep(Mat& img, double& length, Point& carP, int car_rotation, int radius, Point&aim){
    struct path{
        double length; Mat img;
    };
#ifdef SHOWALLIMGS
    path FF,RR,RF,FR;   //F=forward,R=retreat
    img.copyTo(FF.img); img.copyTo(RR.img);
    img.copyTo(RF.img); img.copyTo(FR.img);
    img = Mat(IMGSIZE*2,IMGSIZE*2,img.type());

    Dubins(FF.img,FF.length,carP,car_rotation,radius,aim);
    Dubins(RR.img,RR.length,carP,car_rotation>180?car_rotation-180:car_rotation+180,radius,aim);
    Dubins(RF.img,RF.length,carP,car_rotation,radius,aim,true);
    Dubins(FR.img,FR.length,carP,car_rotation>180?car_rotation-180:car_rotation+180,radius,aim,true);

    length = FF.length; int bestCurve = 0;
    double *lengths[3] = {&RR.length,&RF.length,&FR.length};
    for(int i=0;i<3;i++){
        if(*lengths[i]<length){
            bestCurve = i+1; length = *lengths[i];
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
    if(Dubins(F.img,F.length,carP,car_rotation,radius,aim) == 0){
        img.copyTo(F.img);
        Dubins(F.img,F.length,carP,car_rotation,radius,aim,true);
    }
    if(Dubins(R.img,R.length,carP,car_rotation>180?car_rotation-180:car_rotation+180,radius,aim) == 0){
        img.copyTo(R.img);
        Dubins(R.img,R.length,carP,car_rotation>180?car_rotation-180:car_rotation+180,radius,aim,true);
    }
    if(F.length < R.length){
        length = F.length; F.img.copyTo(img);
    }
    else{
        length = R.length; R.img.copyTo(img);
    }
#endif
}

void getLine(Mat& img, double& length, Point& car, int car_rotation, int radius, Point&aim, bool RS = false){
    if(RS == false)
        Dubins(img,length,car,car_rotation,radius,aim);
    else
        ReedsSheep(img,length,car,car_rotation,radius,aim);
}

void on_Trackbar(int, void *){
    double length;
    Mat img = Mat(IMGSIZE, IMGSIZE, CV_8UC3, Scalar::all(255));
    Rect aimR;
    aimR.x = aim.x - IMGSIZE/50;   aimR.y = aim.y - IMGSIZE/50;
    aimR.width = aimR.height = IMGSIZE/25;
    rectangle(img,aimR,Scalar(0,0,255),2);
    arrow(img,car.center,car.rotation,IMGSIZE/20,Scalar(200,200,50),2);
    getLine(img,length,car.center,car.rotation,car.radius,aim,car.RS);
    cout << "The best curve's length: " << length << endl;
    imshow("Result",img);
}

//Example
int main(){
    car.center = Point(IMGSIZE/6,IMGSIZE/6);
    car.rotation = 160;
    car.radius = IMGSIZE/10;
    car.RS = 1;
    namedWindow("Options", CV_WINDOW_FREERATIO|CV_GUI_NORMAL);
    namedWindow("Result", CV_WINDOW_AUTOSIZE|CV_GUI_NORMAL);
    aim = Point(IMGSIZE/2,IMGSIZE/2);
    createTrackbar("Aim.x", "Options", &(aim.x), IMGSIZE, on_Trackbar);
    createTrackbar("Aim.y", "Options", &(aim.y), IMGSIZE, on_Trackbar);
    createTrackbar("Car.rotation", "Options", &car.rotation, 360, on_Trackbar);
    createTrackbar("Car.x", "Options", &car.center.x, IMGSIZE, on_Trackbar);
    createTrackbar("Car.y", "Options", &car.center.y, IMGSIZE, on_Trackbar);
    createTrackbar("Car.radius", "Options", &car.radius, IMGSIZE, on_Trackbar);
    createTrackbar("Allow Retreat", "Options", &car.RS, 1, on_Trackbar);
    on_Trackbar(0, 0);
    moveWindow("Options",IMGSIZE+10,0);
    moveWindow("Result",0,0);
    waitKey();
    return 0;
}
