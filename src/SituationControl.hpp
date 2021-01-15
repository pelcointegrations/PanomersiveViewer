//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_SITUATION_CONTROL_HPP__
#define __PELCO_IMEXTK_SITUATION_CONTROL_HPP__

#include <memory>
#include <vector>
#include <pelco/imextk/ViewGenerator.h>
#include "MosaicToPanTilt.hpp"

//
// control ptz camera based on input mode: auto, point, calibration and follow
//
class SituationControl {
public:
    SituationControl(std::shared_ptr<SituationTracking> _tracking);
    virtual ~SituationControl();

    // situation awareness mode
    enum SAControlMode {
        SA_OFF,
        SA_AUTO,
        SA_POINT_LOCATION,                          // point to current mouse location
		SA_CALIBRATE_PTZ_PAN_OLD,                   // point Optera and Spectra at each other to find PTZ pan correction and X/Z location. Requires manual heights and distance.
		SA_CALIBRATE_PTZ_PAN,                       // point Optera and Spectra at each other to find PTZ pan correction
		SA_CALIBRATE_PTZ_PAN_POSITION,              // point Optera and Spectra at each other and 1 common point to find PTZ pan correction, height, and X/Z location
		SA_CALIBRATE_PTZ_PAN_POSITION_BLIND,        // point Optera and Spectra at 2 common points to find PTZ pan correction, height, and X/Z location
		SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS,     // point Optera and Spectra at 4 common points to find PTZ height, PTZ X/Z location, and camera tilts (for Optera 270, 360 cal)
		SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS, // point Optera and Spectra at 4 common points to find PTZ pan offset, height, PTZ X/Z location, and camera tilts (for Optera 180 cal)
        SA_FOLLOW_OBJECT,                           // click to follow
        SA_ZOOM_TO_BOX                              // draw a box on optera and zoom to it on spectra
    };
    void setControlMode(SAControlMode mode);
    SAControlMode getControlMode(void) {
        return _saMode;
    }
    void setViewGenerator(PelcoImextk_ViewGen viewGenerator, PelcoImextk_Context imextkContext);
	bool onLMouseButtonUp(int mx, int my, int xAnchor, int yAnchor, float rawPtzPanNorm, float rawPtzTiltNorm);
	void getPtzLocation(float &panNorm, float &tiltNorm, float &zoomNorm);
	enum CalibrationState {
		CAL_RESET_STATE = 0,
		CAL_PAN_STATE,
		CAL_POINT_1_STATE,
		CAL_POINT_2_STATE,
		CAL_POINT_3_STATE,
		CAL_POINT_4_STATE,
		CAL_DONE_STATE
	};
	bool getCamerasSeparated();
	CalibrationState getCalibrationState();
	void setCalibrationState(CalibrationState state);
private:
    // calibrate ptz camera with optera
	bool calibratePtz(int x, int y, float rawPtzPanNorm, float rawPtzTiltNorm);
	bool calibratePtzPan(int viewX, int viewY, float rawPtzPanNorm, bool updatePtzLocation);
	bool calibratePtzPosition();
	bool calibratePtzPanAndPosition();
	bool SituationControl::calibratePtzPositionCameraTilts();
	bool SituationControl::calibratePtzPanPositionCameraTilts();
	void savePointMeasurement(int viewX, int viewY, float rawPtzPanNorm, float rawPtzTiltNorm, int pointNum, bool usePtzPanCorrection);
	void calcInsideAngle(float absAngle1Rad, float absAngle2Rad, float &insideAngleMagRad, int &insideAngleDir);
    void getViewDimension(void);
    bool getViewPointRadians(int x, int y, float &panRadians, float &tiltRadians);
    // click to point where mx and my is point in video screen (not window screen or mouse coordinate)
	bool ptzFollowOpteraClick(int mx, int my);
	bool getSlavePanTiltFromAnalytic(int viewX, int viewY, float targetHeight, float &spectraPanNorm, float &spectraTiltNorm, bool separatedCameras);
    // move ptz camera for calibration
    void movePtzCamera(int mx, int my);
    // set ptz camera to no zoom and fixed tilt angle
	void startCalibration(SAControlMode mode);
	void calcTriangleAngles(float sideA, float sideB, float sideC, float angleA, float &angleB, float &angleC);
    // zome to rectangle on optera
    void zoomToBox(int mx, int my, int xAnchor, int yAnchor);

    std::shared_ptr<SituationTracking> _tracking;
    SAControlMode        _saMode;
    PelcoImextk_ViewGen _viewGenerator;
    // cache view data
    int                  _viewHeight;
    int                  _viewWidth;
	float                _targetHeight;
	CalibrationState     _calibrationState;
	struct CalibrationPoint {
		float opteraPanRad;
		float opteraTiltRad;
		float spectraPanRad;
		float spectraTiltRad;
	};
	CalibrationPoint     _calibrationPoint[5];  // 1st entry is for the pan cal, the rest are for points 1-4.
};

#endif // __PELCO_IMEXTK_SITUATION_CONTROL_HPP__
