#include "stdafx.h"
#include <sstream>
#include "mycamera.hpp"

using namespace std;

#define DOUT( s )             \
{                              \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}

MyCamera::MyCamera() 
	: referencePosX(0)
	, referencePosZ(0)
	, distanceToOptera(0)
{
};

MyCamera::~MyCamera()
{
};

MyCamera::MyCamera(double h, double v, double vangle, int cameratype, const float& minAngMoveDist, const float& minGndMoveDist)
	: referencePosX(0)
	, referencePosZ(0)
	, distanceToOptera(0)
{
    cameraType = cameratype;
    hfov = h;
	vfov = v;
	tilt0 = vangle;
	if (minAngMoveDist)	minAngularMoveDistance = minAngMoveDist;
	if (minGndMoveDist) minGroundMoveDistance = minGndMoveDist;

};

double MyCamera::getY(double tilt) {
	return (1 -  tan((tilt - tilt0) * PI / 180) / tan(vfov / 2.0 * PI / 180)) / 2;
}

double MyCamera::getTilt(double y, double frontfacepercent) {
    double overlapping = 0.2;
    double tiltAngle = tilt0;
    if (cameraType == 270) {
        if (y < frontfacepercent + overlapping) {
            y = 1 - (frontfacepercent + overlapping - y) / (frontfacepercent + overlapping);
            tiltAngle = tilt0;
        }
        else {
            y = (y - frontfacepercent) / (1 - frontfacepercent) / 2 - 0.166;
            tiltAngle = 0.1;
        }
    }
    double a = atan((1 - 2 * y) * tan(vfov / 2.0 * PI / 180));
    double b = a / PI * 180.0;
    double tiltS = (b + tiltAngle);
    tiltS = tiltS < 90 ? tiltS : 89.90;
    return tiltS;
}

double MyCamera::getPan(double x) {
    double f = 1.0 / 6;
    double a = 0;
    double aoffset = 0;
    double xoffset = 0;
    if (x < 2 * f) { // -135 ~ -45 degree
        x -= 0;
        aoffset = 90;
    }
    else if (x < 4 * f) {
        x -= 2 * f;
        aoffset = 0;
    }
    else if (x < 6 * f) {
        x -= 4 * f;
        aoffset = -90;
    }
    a = -(atan((f - x) / f) * 180 / PI + aoffset);
    return a;
}

MyCameraOptera::MyCameraOptera() {
}

MyCameraOptera::MyCameraOptera(double hfov, double vfov, double vangle, int cameratype, float minAngularMoveDist, float minGroundMoveDist)
	: MyCamera(hfov, vfov, vangle, cameratype, minAngularMoveDist, minGroundMoveDist) {
}

double MyCameraOptera::getTilt(double y, double frontfacepercent) {
	double tiltAngle, clapping, a, b, tiltS, overlapping;
	if (cameraType == 360) {
		tiltAngle = tilt0;
		clapping = 0.16;
		a = 0;

		if (y <= 0.5) {
			a = atan((1 - 2 * y) * tan(vfov / 2.0 * PI / 180));
		}
		else {
			tiltAngle = tilt0 - vfov / 2;
			a = atan((1 + clapping - y) / (0.5 + clapping) * tan(vfov / 2 * PI / 180));
		}

		b = a / PI * 180.0;
		tiltS = (b + tiltAngle);
		tiltS = tiltS < 90 ? tiltS : 89.90;
	}
	else if (cameraType == 270) {
		overlapping = 0.16; //top sensor cropping.
		tiltAngle = tilt0;
		if (y < frontfacepercent) {
			y = 1 - (frontfacepercent + overlapping - y) / (frontfacepercent + overlapping);
			tiltAngle = tilt0;
		}
		else {
			y = (y - frontfacepercent) / (1 - frontfacepercent) / 2 - 0.166; //0.166 if bottom overlapping correction
			tiltAngle = 2;
		}
		a = atan((1 - 2 * y) * tan(vfov / 2.0 * PI / 180));
		b = a / PI * 180.0;
		tiltS = (b + tiltAngle);
		tiltS = tiltS < 90 ? tiltS : 89.90;
	}

    return tiltS;
}

double MyCameraOptera::getPan(double x) {
	double f, a;
	if (cameraType == 360) {
		f = 1.0 / 8;
		double aoffset[] = { 135, 45, -45, -135 };
		int index = (int)(x / (2 * f));
		// divide to 4 parts, calcultae angle from -45 to 45 plus offset
		x -= (index * (2 * f));
		a = -(atan((f - x) / f) * 180 / PI + aoffset[index]);
	}
	else if (cameraType == 270) {
		f = 1.0 / 6;
		a = 0;
		double aoffset = 0;
		if (x < 2 * f) { // -135 ~ -45 degree
			x -= 0;
			aoffset = 90;
		}
		else if (x < 4 * f) {
			x -= 2 * f;
			aoffset = 0;
		}
		else if (x < 6 * f) {
			x -= 4 * f;
			aoffset = -90;
		}
		a = -(atan((f - x) / f) * 180 / PI + aoffset);
	}

    return a;
}