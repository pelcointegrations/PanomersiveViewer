#include <math.h>

#ifndef __Camera__
#define __Camera__

const double PI = 3.14159265358979323846;
const double PI_2 = 1.57079632679489661923;
const double PI_4 = 0.785398163397448309616;

// convert radians to degrees and vice-versa
#define RAD2DEG(x) ((x) * 180.0 / PI)
#define DEG2RAD(x) ((x) * PI / 180.0)

class MyCamera {
protected:
	bool   separatedCameras;        // true if optera and spectra are not located at the same location
    int    cameraType;              // 0=spectra, 180=optera 180, 360=optera 360, 270=optera 270
    double installHeight;	        // install height (feet)
    double vfov;			        // vertical field of view (degrees)
    double hfov;                    // horizontal fov (degrees)
    double tilt0;			        // tilt angle (degree)
	double referencePosX;		    // x position in reference space (feet)
	double referencePosZ;		    // y position in reference space (feet)
	double referenceTiltX;			// tilt about x-axis relative to reference space (radians)
	double referenceTiltZ;			// tilt about z-axis relative to reference space (radians)
	double referencePan;            // pan offset relative to reference space X-axis (radians)
	double distanceToOptera;        // distance to optera (feet)
	double minAngularMoveDistance;  // minimum angular move distance (radians)
	double minGroundMoveDistance;   // minimum ground move distance (feet)
	double coneError;               // amount of cone offset (radians)

public:
    MyCamera();
	virtual ~MyCamera();
	MyCamera(double hfov, double vfov, double vangle, int cameratype, const float& minAngMoveDist = NULL, const float& minGndMoveDist = NULL);
    double getY(double tilt);
    virtual double getTilt(double y, double frontfacepercent);
	virtual double getPan(double x);
    double getVFov(void) { return vfov; }
	void setVFov(double v) { vfov = v; }
	double getHFov(void) { return hfov; }
	void setHFov(double h) { hfov = h; }
	double getVAngle(void) { return tilt0; }
	void setVAngle(double v) { tilt0 = v; }
	int getCameraType(void) { return cameraType; }
	void setCameraType(int type) { cameraType = type; }
	double getInstallHeight() {	return installHeight; }
	void setInstallHeight(double h) { installHeight = h; }
	double getConeError() { return coneError; }
	void setConeError(double h) { coneError = h; }
	double getReferencePosX() { return referencePosX; }
	void setReferencePosX(double x) { referencePosX = x; }
	double getReferencePosZ() { return referencePosZ; }
	void setReferencePosZ(double z) { referencePosZ = z; }
	double getReferenceTiltX() { return referenceTiltX; }
	void setReferenceTiltX(double x) { referenceTiltX = x; }
	double getReferenceTiltZ() { return referenceTiltZ; }
	void setReferenceTiltZ(double z) { referenceTiltZ = z; }
	double getReferencePan() { return referencePan; }
	void setReferencePan(double p) { referencePan = p; }
	double getDistanceToOptera() { return distanceToOptera; }
	void setDistanceToOptera(double d) { distanceToOptera = d; }
	bool getSeparatedCameras() { return separatedCameras; }
	void setSeparatedCameras(bool c) { separatedCameras = c; }
	double getMinAngularMoveDistance() { return minAngularMoveDistance; }
	void setMinAngularMoveDistance(double d) { minAngularMoveDistance = d; }
	double getMinGroundMoveDistance() { return minGroundMoveDistance; }
	void setMinGroundMoveDistance(double d) { minGroundMoveDistance = d; }
};

class MyCameraOptera : public MyCamera {
protected:

public:
	MyCameraOptera();
	MyCameraOptera(double hfov, double vfov, double vangle, int cameratype, float minAngMoveDist, float minGndMoveDist);
    double getTilt(double y, double frontfacepercent);
	double getPan(double x);
};

#endif
