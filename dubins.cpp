#include "bestcruve.h"

int CL_FL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    int direction = car_direction>180?car_direction-180:car_direction+180;
    Point2f centerTurn;
    centerTurn.x = carP.x - radius * cos(a2r(90)-a2r(car_direction));
    centerTurn.y = carP.y + radius * sin(a2r(90)-a2r(car_direction));
    if(pow(centerTurn.x-aim.x,2)+pow(centerTurn.y-aim.y,2) < pow(radius,2))
        return CL_FR(img,length,angle,carP,car_direction,radius,aim);
    int x = carP.x, y = carP.y;
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    circle(img,centerTurn,3,Scalar(0),-1);
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    double beita = theta - direction;
    int cu = 0;
    while(1){
        if(abs(beita) < STEP) break;
        ellipse(img,centerTurn,Size(radius,radius),direction+90,0,STEP,Scalar(0),2);
        direction -= STEP;
        if(direction >= 360)  direction = 0;
        if(direction <= -1)  direction = 359;
        x = centerTurn.x + radius * cos(a2r(direction)+a2r(90));
        y = centerTurn.y + radius * sin(a2r(direction)+a2r(90));
        theta = r2a(atan2(aim.y-y,aim.x-x));
        if(theta<0) theta+=360;
        beita = theta - direction;
        if(cu++>360/STEP) break;
    }
    angle = direction>180?direction-180:direction+180;
    length = (M_PI * radius)*cu*STEP/180.0 + sqrt(pow(y - aim.y,2) + pow(x - aim.x,2));
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    line(img,Point(x,y),aim,Scalar(0),2);
#ifdef DISPLAYMSG
    stringstream ss;    ss << "MinRadius:" << radius << " ";
    string lengthStr;   ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
    ss << "Length:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
#endif
    return cu*STEP;
}

int CL_FR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    int direction = car_direction>180?car_direction-180:car_direction+180;
    Point2f centerTurn;
    centerTurn.x = carP.x - radius * cos(a2r(90)-a2r(direction));
    centerTurn.y = carP.y + radius * sin(a2r(90)-a2r(direction));
    if(pow(centerTurn.x-aim.x,2)+pow(centerTurn.y-aim.y,2) < pow(radius,2))
        return CL_FL(img,length,angle,carP,car_direction,radius,aim);
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    int x = carP.x, y = carP.y;
    circle(img,centerTurn,3,Scalar(0),-1);
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    double beita = theta - direction;
    int cu = 0;
    while(1){
        if(abs(beita) < STEP) break;
        ellipse(img,centerTurn,Size(radius,radius),direction-90,0,STEP,Scalar(0),2);
        direction += STEP;
        if(direction >= 360)  direction = 0;
        if(direction <= -1)  direction = 359;
        x = centerTurn.x + radius * cos(a2r(direction)-a2r(90));
        y = centerTurn.y + radius * sin(a2r(direction)-a2r(90));
        theta = r2a(atan2(aim.y-y,aim.x-x));
        if(theta<0) theta+=360;
        beita = theta - direction;
        if(cu++>360/STEP) break;
    }
    angle = direction>180?direction-180:direction+180;
    length = (M_PI * radius)*cu*STEP/180.0 + sqrt(pow(y - aim.y,2) + pow(x - aim.x,2));
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    line(img,Point(x,y),aim,Scalar(0),2);
#ifdef DISPLAYMSG
    stringstream ss;    ss << "MinRadius:" << radius << " ";
    string lengthStr;   ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
    ss << "Length:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/20,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
#endif
    return cu*STEP;
}

int Dubins(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim, bool showall=true){
    double theta = r2a(atan2(carP.y-aim.y,carP.x-aim.x));
    if(theta<0) theta+=360;
    int bestCurve;
    int incAngle = car_direction-theta;
    if(incAngle<0) incAngle+=360;
    if(incAngle>180) bestCurve = 2;
    else bestCurve = 1;
    if(showall){
        Path FL,FR;   //F=forward,R=retreat
        img.copyTo(FL.img); img.copyTo(FR.img);
        img = Mat(IMGSIZE,IMGSIZE*2,img.type());

        FL.cu = CL_FL(FL.img,FL.length,FL.angle,carP,car_direction,radius,aim);
        FR.cu = CL_FR(FR.img,FR.length,FR.angle,carP,car_direction,radius,aim);

        putText(FL.img,"1.FL",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 1?255:0),IMGSIZE/300.0);
        Mat imageROI= img(Rect(0,0,IMGSIZE,IMGSIZE));
        FL.img.copyTo(imageROI);

        putText(FR.img,"2.FR",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 2?255:0),IMGSIZE/300.0);
        imageROI= img(Rect(IMGSIZE,0,IMGSIZE,IMGSIZE));
        FR.img.copyTo(imageROI);

        line(img,Point(0,IMGSIZE),Point(IMGSIZE*2,IMGSIZE),Scalar(0),2);
        line(img,Point(IMGSIZE,0),Point(IMGSIZE,IMGSIZE*2),Scalar(0),2);
        resize(img,img,Size(IMGSIZE,IMGSIZE/2));

        switch(bestCurve){
        case 1: angle = FL.angle; length = FL.length; return FL.cu;
        case 2: angle = FR.angle; length = FR.length; return FR.cu;
        default: break;
        }
        return -1;
    }
    else{
        Path bestpath;
        img.copyTo(bestpath.img);
        switch(bestCurve){
        case 1: bestpath.cu = CL_FL(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim); break;
        case 2: bestpath.cu = CL_FR(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim); break;
        default: break;
        }
        bestpath.img.copyTo(img);
        angle = bestpath.angle; length = bestpath.length;
        return bestpath.cu;
    }
}
