#include "bestcruve.h"

int CL_BL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    Point2f centerTurn;
    centerTurn.x = carP.x - radius * cos(a2r(90)-a2r(car_direction));
    centerTurn.y = carP.y + radius * sin(a2r(90)-a2r(car_direction));
    if(pow(centerTurn.x-aim.x,2)+pow(centerTurn.y-aim.y,2) < pow(radius,2))
        return CL_BR(img,length,angle,carP,car_direction,radius,aim);
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    int direction = car_direction, x = carP.x, y = carP.y;
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
    angle = direction;
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

int CL_BR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    Point2f centerTurn;
    centerTurn.x = carP.x + radius * cos(a2r(90)-a2r(car_direction));
    centerTurn.y = carP.y - radius * sin(a2r(90)-a2r(car_direction));
    if(pow(centerTurn.x-aim.x,2)+pow(centerTurn.y-aim.y,2) < pow(radius,2))
        return CL_BL(img,length,angle,carP,car_direction,radius,aim);
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    int direction = car_direction, x = carP.x, y = carP.y;
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
    angle = direction;
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

int CCL_FL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    int direction = car_direction>180?car_direction-180:car_direction+180, x = carP.x, y = carP.y;
    Point2f centerTurn;
    centerTurn.x = x + radius * cos(a2r(90)-a2r(direction));
    centerTurn.y = y - radius * sin(a2r(90)-a2r(direction));
    circle(img,centerTurn,3,Scalar(0),-1);
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    int cu = 0;
    while(1){
        Mat img_tmp; img.copyTo(img_tmp);
        if(CL_FL(img_tmp,length,angle,Point(x,y),direction,radius,aim) < 270){
            img_tmp.copyTo(img); break;
        }
        ellipse(img,centerTurn,Size(radius,radius),direction+90,0,STEP,Scalar(0),2);
        direction -= STEP;
        if(direction >= 360)  direction = 0;
        if(direction <= -1)  direction = 359;
        x = centerTurn.x + radius * cos(a2r(direction)+a2r(90));
        y = centerTurn.y + radius * sin(a2r(direction)+a2r(90));
        theta = r2a(atan2(aim.y-y,aim.x-x));
        if(theta<0) theta+=360;
        if(cu++>90/STEP) break;
    }
    angle = direction>180?direction-180:direction+180;
    length += (M_PI * radius)*cu*STEP/180.0;
    line(img,Point(x,y),centerTurn,Scalar(0),1);
#ifdef DISPLAYMSG
    stringstream ss;
    string lengthStr;
    ss << "Total:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/2,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
#endif
    return cu*STEP;
}

int CCL_FR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    int direction = car_direction>180?car_direction-180:car_direction+180, x = carP.x, y = carP.y;
    Point2f centerTurn;
    centerTurn.x = x - radius * cos(a2r(90)-a2r(direction));
    centerTurn.y = y + radius * sin(a2r(90)-a2r(direction));
    circle(img,centerTurn,3,Scalar(0),-1);
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    int cu = 0;
    while(1){
        Mat img_tmp; img.copyTo(img_tmp);
        if(CL_FR(img_tmp,length,angle,Point(x,y),direction,radius,aim) < 270){
            img_tmp.copyTo(img); break;
        }
        ellipse(img,centerTurn,Size(radius,radius),direction-90,0,STEP,Scalar(0),2);
        direction += STEP;
        if(direction >= 360)  direction = 0;
        if(direction <= -1)  direction = 359;
        x = centerTurn.x + radius * cos(a2r(direction)-a2r(90));
        y = centerTurn.y + radius * sin(a2r(direction)-a2r(90));
        theta = r2a(atan2(aim.y-y,aim.x-x));
        if(theta<0) theta+=360;
        if(cu++>90/STEP) break;
    }
    angle = direction>180?direction-180:direction+180;
    length += (M_PI * radius)*cu*STEP/180.0;
    line(img,Point(x,y),centerTurn,Scalar(0),1);
#ifdef DISPLAYMSG
    stringstream ss;
    string lengthStr;
    ss << "Total:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/2,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
#endif
    return cu*STEP;
}

int CCL_BL(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    int direction = car_direction, x = carP.x, y = carP.y;
    Point2f centerTurn;
    centerTurn.x = x - radius * cos(a2r(90)-a2r(direction));
    centerTurn.y = y + radius * sin(a2r(90)-a2r(direction));
    circle(img,centerTurn,3,Scalar(0),-1);
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    int cu = 0;
    while(1){
        Mat img_tmp; img.copyTo(img_tmp);
        if(CL_FR(img_tmp,length,angle,Point(x,y),direction,radius,aim) < 270){
            img_tmp.copyTo(img); break;
        }
        ellipse(img,centerTurn,Size(radius,radius),direction-90,0,STEP,Scalar(0),2);
        direction += STEP;
        if(direction >= 360)  direction = 0;
        if(direction <= -1)  direction = 359;
        x = centerTurn.x + radius * cos(a2r(direction)-a2r(90));
        y = centerTurn.y + radius * sin(a2r(direction)-a2r(90));
        theta = r2a(atan2(aim.y-y,aim.x-x));
        if(theta<0) theta+=360;
        if(cu++>90/STEP) break;
    }
    angle = direction;
    length += (M_PI * radius)*cu*STEP/180.0;
    line(img,Point(x,y),centerTurn,Scalar(0),1);
#ifdef DISPLAYMSG
    stringstream ss;
    string lengthStr;
    ss << "Total:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/2,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
#endif
    return cu*STEP;
}

int CCL_BR(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim){
    double theta = r2a(atan2(aim.y-carP.y,aim.x-carP.x));
    if(theta<0) theta+=360;
    int direction = car_direction, x = carP.x, y = carP.y;
    Point2f centerTurn;
    centerTurn.x = x + radius * cos(a2r(90)-a2r(direction));
    centerTurn.y = y - radius * sin(a2r(90)-a2r(direction));
    circle(img,centerTurn,3,Scalar(0),-1);
    line(img,Point(x,y),centerTurn,Scalar(0),1);
    int cu = 0;
    while(1){
        Mat img_tmp; img.copyTo(img_tmp);
        if(CL_FL(img_tmp,length,angle,Point(x,y),direction,radius,aim) < 270){
            img_tmp.copyTo(img); break;
        }
        ellipse(img,centerTurn,Size(radius,radius),direction+90,0,STEP,Scalar(0),2);
        direction -= STEP;
        if(direction >= 360)  direction = 0;
        if(direction <= -1)  direction = 359;
        x = centerTurn.x + radius * cos(a2r(direction)+a2r(90));
        y = centerTurn.y + radius * sin(a2r(direction)+a2r(90));
        theta = r2a(atan2(aim.y-y,aim.x-x));
        if(theta<0) theta+=360;
        if(cu++>90/STEP) break;
    }
    angle = direction;
    length += (M_PI * radius)*cu*STEP/180.0;
    line(img,Point(x,y),centerTurn,Scalar(0),1);
#ifdef DISPLAYMSG
    stringstream ss;
    string lengthStr;
    ss << "Total:" << length; ss >> lengthStr;
    putText(img,lengthStr,Point(IMGSIZE/2,IMGSIZE-IMGSIZE/20*2),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0),IMGSIZE/300.0);
#endif
    return cu*STEP;
}

int ReedsSheep(Mat& img, double& length, int& angle, Point carP, int car_direction, int radius, Point aim, bool showall=true){
    double theta = r2a(atan2(carP.y-aim.y,carP.x-aim.x));
    if(theta<0) theta+=360;
    int bestCurve;
    int incAngle = car_direction-theta;
    if(incAngle<0) incAngle+=360;
    if(incAngle>270) bestCurve = 2;
    else if(incAngle>180) bestCurve = 4;
    else if(incAngle>90) bestCurve = 3;
    else bestCurve = 1;
    if(showall){
        Path FL,FR,BL,BR;   //F=forward,R=retreat
        img.copyTo(FL.img); img.copyTo(FR.img);
        img.copyTo(BL.img); img.copyTo(BR.img);

        FL.cu = CL_FL(FL.img,FL.length,FL.angle,carP,car_direction,radius,aim);
        if(FL.cu>270){
            img.copyTo(FL.img);
            FL.cu = CCL_BR(FL.img,FL.length,FL.angle,carP,car_direction,radius,aim);
        }
        FR.cu = CL_FR(FR.img,FR.length,FR.angle,carP,car_direction,radius,aim);
        if(FR.cu>270){
            img.copyTo(FR.img);
            FR.cu = CCL_BL(FR.img,FR.length,FR.angle,carP,car_direction,radius,aim);
        }
        BL.cu = CL_BL(BL.img,BL.length,BL.angle,carP,car_direction,radius,aim);
        if(BL.cu>270){
            img.copyTo(BL.img);
            BL.cu = CCL_FR(BL.img,BL.length,BL.angle,carP,car_direction,radius,aim);
        }
        BR.cu = CL_BR(BR.img,BR.length,BR.angle,carP,car_direction,radius,aim);
        if(BR.cu>270){
            img.copyTo(BR.img);
            BR.cu = CCL_FL(BR.img,BR.length,BR.angle,carP,car_direction,radius,aim);
        }
        img = Mat(IMGSIZE*2,IMGSIZE*2,img.type());
        putText(FL.img,"1.FL",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 1?255:0),IMGSIZE/300.0);
        Mat imageROI= img(Rect(0,0,IMGSIZE,IMGSIZE));
        FL.img.copyTo(imageROI);

        putText(FR.img,"2.FR",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 2?255:0),IMGSIZE/300.0);
        imageROI= img(Rect(IMGSIZE,0,IMGSIZE,IMGSIZE));
        FR.img.copyTo(imageROI);

        putText(BL.img,"3.BL",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 3?255:0),IMGSIZE/300.0);
        imageROI= img(Rect(0,IMGSIZE,IMGSIZE,IMGSIZE));
        BL.img.copyTo(imageROI);

        putText(BR.img,"4.BR",Point(IMGSIZE/20,IMGSIZE/20),FONT_HERSHEY_COMPLEX,IMGSIZE/600.0,Scalar(0,0,bestCurve == 4?255:0),IMGSIZE/300.0);
        imageROI= img(Rect(IMGSIZE,IMGSIZE,IMGSIZE,IMGSIZE));
        BR.img.copyTo(imageROI);

        line(img,Point(0,IMGSIZE),Point(IMGSIZE*2,IMGSIZE),Scalar(0),2);
        line(img,Point(IMGSIZE,0),Point(IMGSIZE,IMGSIZE*2),Scalar(0),2);
        resize(img,img,Size(IMGSIZE,IMGSIZE));

        switch(bestCurve){
        case 1: angle = FL.angle; length = FL.length; return FL.cu;
        case 2: angle = FR.angle; length = FR.length; return FR.cu;
        case 3: angle = BL.angle; length = BL.length; return BL.cu;
        case 4: angle = BR.angle; length = BR.length; return BR.cu;
        default: break;
        }
        return -1;
    }
    else{
        Path bestpath;
        img.copyTo(bestpath.img);
        switch(bestCurve){
        case 1:
            bestpath.cu = CL_FL(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            if(bestpath.cu>270){
                img.copyTo(bestpath.img);
                bestpath.cu = CCL_BR(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            }
            break;
        case 2:
            bestpath.cu = CL_FR(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            if(bestpath.cu>270){
                img.copyTo(bestpath.img);
                bestpath.cu = CCL_BL(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            }
            break;
        case 3:
            bestpath.cu = CL_BL(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            if(bestpath.cu>270){
                img.copyTo(bestpath.img);
                bestpath.cu = CCL_FR(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            }
            break;
        case 4:
            bestpath.cu = CL_BR(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            if(bestpath.cu>270){
                img.copyTo(bestpath.img);
                bestpath.cu = CCL_FL(bestpath.img,bestpath.length,bestpath.angle,carP,car_direction,radius,aim);
            }
            break;
        default: break;
        }
        bestpath.img.copyTo(img);
        angle = bestpath.angle; length = bestpath.length;
        return bestpath.cu;
    }
}
