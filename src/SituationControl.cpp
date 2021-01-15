//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include <assert.h>
#include <algorithm>
#include <thread>
#include <chrono>
#include <ctime>
#include "SituationControl.hpp"
#include "numeric_calibrate.h"

using namespace std;

//#define _DEBUG_OUTPUT
#ifdef _DEBUG_OUTPUT
#define DOUT( s )                            \
{                                            \
   std::wostringstream os_;                  \
   os_ << s;                                 \
   OutputDebugStringW( os_.str().c_str() );  \
}
#else
#define DOUT( s )
#endif

#define MAX_ZOOM_FACTOR 4.0

//////////////////////////////////////////////////////////////////////////////////
void waitForCameraIdle(PtzControl& onvifPtz) {
    // wait for camera setting done, maximum 10 seconds
    for (int i = 0; i < 100; ++i) {
        // wait a little bit
        DOUT("wait 100ms\n");
        this_thread::sleep_for(chrono::milliseconds(100));
        if (onvifPtz.isCameraIdle()) {
            break;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
SituationControl::SituationControl(std::shared_ptr<SituationTracking> tracking)
    : _tracking(tracking)
    , _saMode(SA_OFF)
    , _viewGenerator(PelcoImextk_kInvalidViewGen)
    , _viewWidth(1)
    , _viewHeight(1)
    , _targetHeight(0)
	, _calibrationState(CAL_RESET_STATE) {
}

//////////////////////////////////////////////////////////////////////////////////
SituationControl::~SituationControl() {
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::onLMouseButtonUp(int mx, int my, int xAnchor, int yAnchor, float rawPtzPanNorm, float rawPtzTiltNorm) {
    getViewDimension();
	switch (_saMode) {
		case SA_CALIBRATE_PTZ_PAN_OLD:
		case SA_CALIBRATE_PTZ_PAN:
		case SA_CALIBRATE_PTZ_PAN_POSITION:
		case SA_CALIBRATE_PTZ_PAN_POSITION_BLIND:
		case SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS:
		case SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS:
			calibratePtz(mx, my, rawPtzPanNorm, rawPtzTiltNorm);
			break;
        case SA_POINT_LOCATION:
            ptzFollowOpteraClick(mx, my);
            break;
        case SA_FOLLOW_OBJECT:
            {
                _tracking->_ptzControl.setPtzEnable(false);
                // point ptz to the location
                if (ptzFollowOpteraClick(mx, my)) {
                    float panRadians;
                    float tiltRadians;
                    if (getViewPointRadians(mx, my, panRadians, tiltRadians)) {
                        _tracking->_mosaicToPanTilt.mapAnalyticPanTilt(panRadians, tiltRadians);
                        _tracking->followObject(panRadians, tiltRadians);
                    }
                }
                // set ptz auto enable
                _tracking->_ptzControl.setPtzEnable(true);
            }
            break;
        case SA_ZOOM_TO_BOX:
            {
				// no box drawn, defer to Point to Click
				if ((mx == xAnchor) && (my == yAnchor)) {
					ptzFollowOpteraClick(mx, my);
				}
				else {
					zoomToBox(mx, my, xAnchor, yAnchor);
				}
            }
            break;
        default:
            break;
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::getPtzLocation(float &panNorm, float &tiltNorm, float &zoomNorm) {
	waitForCameraIdle(_tracking->_ptzControl);
	_tracking->_ptzControl.getCameraLocation(panNorm, tiltNorm, zoomNorm);
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::getCamerasSeparated() {
	return _tracking->_cameraInfos._cameraSpectra.getSeparatedCameras();
}

//////////////////////////////////////////////////////////////////////////////////
SituationControl::CalibrationState SituationControl::getCalibrationState() {
	return _calibrationState;
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::setCalibrationState(CalibrationState state) {
	_calibrationState = state;
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::calcTriangleAngles(float sideA, float sideB, float sideC, float angleA, float &angleB, float &angleC) {

	// Given 3 sides and one angle, calc the remaining 2 angles.
	// Angle A is opposite side A, angle B is opposite side B, etc.

	// use the law of cosines to solve for the 1st unknown angle
	angleB = acos((sideA * sideA + sideC * sideC - sideB * sideB) / (2 * sideA * sideC));

	// solve for the 2nd unknown angle knowing that the sum of the 3 angles = 180 
	angleC = (float)PI - angleA - angleB;

	return;
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::startCalibration(SAControlMode mode) {
	float x = 0, y = 0, z = 0;
	float ptzReferencePositionX = 0, ptzReferencePositionZ = 0, opteraPanToSpectraRad = 0, spectraPanToOpteraRad = 0;
    _tracking->_ptzControl.getCameraLocation(x, y, z);
    z = 0;
	switch (mode) {
	case SA_CALIBRATE_PTZ_PAN_OLD:
	case SA_CALIBRATE_PTZ_PAN:
	case SA_CALIBRATE_PTZ_PAN_POSITION:
		switch (_tracking->_cameraInfos._cameraType) {
		case CameraInfos::CAMERA_360:
			// Point at Optera
			// calc pan angle from optera to spectra
			ptzReferencePositionX = (float)_tracking->_cameraInfos._cameraSpectra.getReferencePosX();
			ptzReferencePositionZ = (float)_tracking->_cameraInfos._cameraSpectra.getReferencePosZ();
			opteraPanToSpectraRad = atan2(ptzReferencePositionZ, ptzReferencePositionX);
			// calc spectra pan angle looking back at optera (180deg off)
			spectraPanToOpteraRad = opteraPanToSpectraRad - (float)PI;
			spectraPanToOpteraRad += (float)_tracking->_cameraInfos._cameraSpectra.getReferencePan();
			x = foldOverBoundary(spectraPanToOpteraRad, -(float)PI, (float)PI) / (float)PI;
			y = (float) 0.85;
			break;
		case CameraInfos::CAMERA_270:
			y = (float) 0.65;
			break;
		default:
			y = (float) 0.9;
			break;
		}
		break;
	case SA_CALIBRATE_PTZ_PAN_POSITION_BLIND:
	case SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS:
	case SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS:
		break;
	}
	_tracking->_ptzControl.manualSetAbsoluteLocation(x, y, &z);
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::calibratePtzPan(int viewX, int viewY, float rawPtzPanNorm, bool updatePtzLocation) {

	// Point Optera and Spectra at each other to find PTZ pan correction and X/Z location.

	// This is the original calibration method. It required the PTZ height and distance from Optera.
	// Just the pan calibration can be performed by setting updatePtzLocation = false.

	float rawPtzPanRad = rawPtzPanNorm * (float)PI;
	// get equivalent ptz pan/tilt as if ptz was co-located with optera
	float targetHeight = 0;

	float opteraPanRad, opteraTiltRad;
	if (getViewPointRadians(viewX, viewY, opteraPanRad, opteraTiltRad)) {             // cube radians
		_tracking->_mosaicToPanTilt.mapAnalyticPanTilt(opteraPanRad, opteraTiltRad);  // optera radians

		float distanceToOptera = (float)(_tracking->_cameraInfos._cameraSpectra.getDistanceToOptera());
		bool separatedCameras = _tracking->_cameraInfos._cameraSpectra.getSeparatedCameras();
		float ptzReferencePositionX, ptzReferencePositionZ, ptzReferencePanRad;
		if (!separatedCameras) {
			// co-located camera case
			ptzReferencePanRad = (rawPtzPanRad - opteraPanRad);
			// ptz global position same as optera's
			ptzReferencePositionX = 0.0;
			ptzReferencePositionZ = 0.0;
		}
		else {
			// separated camera case - optera and PTZ are looking at each other (opposite point)
			float ptzPanToOpteraRad = opteraPanRad - (float)PI; // the pan direction for PTZ looking at optera is the opposite direction optera is looking
			ptzReferencePanRad = rawPtzPanRad - ptzPanToOpteraRad;
			ptzReferencePositionX = distanceToOptera * cos(opteraPanRad);
			ptzReferencePositionZ = distanceToOptera * sin(opteraPanRad);
		}

		// save results
		_tracking->_cameraInfos._cameraSpectra.setReferencePan(ptzReferencePanRad);
		if (updatePtzLocation) {
			// save pan and location results
			_tracking->_cameraInfos._cameraSpectra.setReferencePosX(ptzReferencePositionX);
			_tracking->_cameraInfos._cameraSpectra.setReferencePosZ(ptzReferencePositionZ);
			_tracking->_options.updateCalibrationParams(&ptzReferencePanRad, &ptzReferencePositionX, &ptzReferencePositionZ, NULL, NULL, NULL, NULL, NULL, NULL);
		}
		else {
			// just save pan results
			_tracking->_options.updateCalibrationParams(&ptzReferencePanRad, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
		}

		DOUT(" calibratePtzPan: ptzPanRad (uncal'd): " << rawPtzPanRad << " opteraPanRad: " << opteraPanRad << " ptzPanRefRad: " << ptzReferencePanRad << " GlobalPosX: " << distanceToOptera * cos(opteraPanRad) << " GlobalPosZ: " << distanceToOptera * sin(opteraPanRad) << endl);
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::calibratePtzPosition() {

	// Point Optera and Spectra at 1 common point to find PTZ height, and X/Z location.

	// This routine calibrates the position and height of the PTZ relative to Optera by pointing the PTZ and Optera to
	// 1 common point on the ground plane. This assumes that the PTZ pan calibration has already been performed.
	// The method used is solving geometric equations.		

	// calc optera to point dist
	float opteraToPointBaseDist = (float)_tracking->_cameraInfos._cameraOptera.getInstallHeight() / tan(-_calibrationPoint[1].opteraTiltRad);
	// calc spectra to point dist
	float opteraPointToSpectraAngle = abs(_calibrationPoint[1].opteraPanRad - _calibrationPoint[0].opteraPanRad);
	if (opteraPointToSpectraAngle > PI) { opteraPointToSpectraAngle = (float)(2 * PI - opteraPointToSpectraAngle); } 	// 180 wrap correction
	float spectraPointToOpteraAngle = abs(_calibrationPoint[1].spectraPanRad - _calibrationPoint[0].spectraPanRad);
	if (spectraPointToOpteraAngle > PI) { spectraPointToOpteraAngle = (float)(2 * PI - spectraPointToOpteraAngle); } 	// 180 wrap correction
	float spectraToPointBaseDist = opteraToPointBaseDist * sin(opteraPointToSpectraAngle) / sin(spectraPointToOpteraAngle);
	// calc spectra height
	float spectraHeight = spectraToPointBaseDist * tan(-_calibrationPoint[1].spectraTiltRad);
	// calc optera to spectra dist
	float pointSpectraToOpteraAngle = (float)PI - opteraPointToSpectraAngle - spectraPointToOpteraAngle;
	float opteraToSpectraDist = spectraToPointBaseDist * sin(pointSpectraToOpteraAngle) / sin(opteraPointToSpectraAngle);
	// calc spectra x,z location
	float spectraPositionX = opteraToSpectraDist * cos(_calibrationPoint[0].opteraPanRad);
	float spectraPositionZ = opteraToSpectraDist * sin(_calibrationPoint[0].opteraPanRad);
	// save results
	_tracking->_cameraInfos._cameraSpectra.setDistanceToOptera(opteraToSpectraDist);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosX(spectraPositionX);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosZ(spectraPositionZ);
	_tracking->_cameraInfos._cameraSpectra.setInstallHeight(spectraHeight);
	_tracking->_options.updateCalibrationParams(NULL, &spectraPositionX, &spectraPositionZ, &opteraToSpectraDist, &spectraHeight, NULL, NULL, NULL, NULL);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::calibratePtzPanAndPosition() {

	// Point Optera and Spectra at 2 common points to find PTZ pan correction, height, and X/Z location.

	// This routine is used for the case where the PTZ and Optera can not see each other and therefore the normal PTZ
	// pan calibration cannot be performed. This routine will calibrate the PTZ pan offset, height, and position by
	// pointing the PTZ and Optera to 2 common points on the ground plane. The method used is solving geometric equations.

	// Simple variable names are used for readibility. Refer to 2-point calibration document diagram for definitions.
	//   Optera - Point 1 - Point 2 Triangle:
	//		a = optera to point 1 distance
	//		b = optera to point 2 distance
	//		p = point 1 to point 2 distance
	//   Spectra - Point 1 - Point 2 Triangle
	//		e = spectra to point 1 distance
	//		f = spectra to point 2 distance

	// calc dists and angles of optera - point 1 - point 2 triangle
	// calc optera to point dists
	float a = (float)_tracking->_cameraInfos._cameraOptera.getInstallHeight() / tan(-_calibrationPoint[1].opteraTiltRad);
	float b = (float)_tracking->_cameraInfos._cameraOptera.getInstallHeight() / tan(-_calibrationPoint[2].opteraTiltRad);
	// calc point 1 to point 2 dist
	float opteraPoint1ToPoint2Angle; int opteraPoint1ToPoint2Dir;
	calcInsideAngle(_calibrationPoint[1].opteraPanRad, _calibrationPoint[2].opteraPanRad, opteraPoint1ToPoint2Angle, opteraPoint1ToPoint2Dir);
	float p = sqrt(a * a + b * b - 2 * a * b * cos(opteraPoint1ToPoint2Angle));
	// calc angles across from sides a & b
	float point1OpteraToPoint2Angle, point2OpteraToPoint1Angle;
	calcTriangleAngles(p, a, b, opteraPoint1ToPoint2Angle, point2OpteraToPoint1Angle, point1OpteraToPoint2Angle);

	// calc dists and angles of spectra - point 1 - point 2 triangle
	float spectraPoint1ToPoint2Angle; int spectraPoint1ToPoint2Dir;
	calcInsideAngle(_calibrationPoint[1].spectraPanRad, _calibrationPoint[2].spectraPanRad, spectraPoint1ToPoint2Angle, spectraPoint1ToPoint2Dir);
	float eNorm = 1 / tan(-_calibrationPoint[1].spectraTiltRad);  // normalized to spectra height, Hs
	float fNorm = 1 / tan(-_calibrationPoint[2].spectraTiltRad);
	float spectraHeight = sqrt((p * p) / (eNorm * eNorm + fNorm * fNorm - 2 * eNorm * fNorm * cos(spectraPoint1ToPoint2Angle)));
	float e = eNorm * spectraHeight;
	float f = fNorm * spectraHeight;
	float point1SpectraToPoint2Angle, point2SpectraToPoint1Angle;
	calcTriangleAngles(p, e, f, spectraPoint1ToPoint2Angle, point2SpectraToPoint1Angle, point1SpectraToPoint2Angle);

	// calc dists and angles for optera - spectra - point 1 triangle
	float point1OpteraToSpectraAngle;
	if (opteraPoint1ToPoint2Dir != spectraPoint1ToPoint2Dir) {
		// point1 to point2 line is between optera and spectra
		point1OpteraToSpectraAngle = point1OpteraToPoint2Angle + point1SpectraToPoint2Angle;
	}
	else {
		// point1 to point2 line is not between optera and spectra
		point1OpteraToSpectraAngle = abs(point1OpteraToPoint2Angle - point1SpectraToPoint2Angle);
	}
	if (point1OpteraToSpectraAngle > (float)PI) { point1OpteraToSpectraAngle = (float)(2 * PI - point1OpteraToSpectraAngle); } 	// 180 wrap correction
	float spectraToOpteraDist = sqrt(a * a + e * e - 2 * a * e * cos(point1OpteraToSpectraAngle));
	float opteraSpectraToPoint1Angle, spectraOpteraToPoint1Angle;
	calcTriangleAngles(spectraToOpteraDist, a, e, point1OpteraToSpectraAngle, spectraOpteraToPoint1Angle, opteraSpectraToPoint1Angle);

	// Calc optera to spectra pan angle
	// The calc of this angle varies depending on whether points 1 & 2 are between spectra and optera and whether 
	// they are both on the same side of the optera-spectra line or not.
	int opteraPoint1ToSpectraDir;
	if (opteraPoint1ToPoint2Dir != spectraPoint1ToPoint2Dir) {
		// point1 to point2 line is between optera and spectra
		// determine if both points are on the same side of the optera to spectra line or not
		if (opteraPoint1ToPoint2Dir == 0) {
			opteraPoint1ToSpectraDir = (point1OpteraToPoint2Angle + point1SpectraToPoint2Angle < (float)PI) ? 0 : 1;
		}
		else {
			opteraPoint1ToSpectraDir = (point1OpteraToPoint2Angle + point1SpectraToPoint2Angle < (float)PI) ? 1 : 0;
		}
	}
	else {
		// point1 to point2 line is not between optera and spectra
		// determine if both points are on the same side of the optera to spectra line or not
		if (opteraPoint1ToPoint2Dir == 0) {
			opteraPoint1ToSpectraDir = (point1OpteraToPoint2Angle > point1SpectraToPoint2Angle) ? 0 : 1;
		}
		else {
			opteraPoint1ToSpectraDir = (point1OpteraToPoint2Angle > point1SpectraToPoint2Angle) ? 1 : 0;
		}
	}
	float opteraPanToSpectra;
	if (opteraPoint1ToSpectraDir == 1) {
		opteraPanToSpectra = _calibrationPoint[1].opteraPanRad + opteraSpectraToPoint1Angle;
	}
	else {
		opteraPanToSpectra = _calibrationPoint[1].opteraPanRad - opteraSpectraToPoint1Angle;
	}

	// Calc spectra x,z location
	float spectraPositionX = spectraToOpteraDist * cos(opteraPanToSpectra);
	float spectraPositionZ = spectraToOpteraDist * sin(opteraPanToSpectra);

	// Calc pan spectra offset
	// Optera pan is aligned to global coord's, spectra pan is not since pan is not calibrated yet
	// the pan direction for PTZ looking at optera is the opposite direction for optera looking at spectra
	float spectraPanToOptera = opteraPanToSpectra - (float)PI;  // this is what the calibrated spectra to optera pan angle should be 
	// take into account all possible positions of point 1 relative to spectra to optera line
	float rawSpectraPanToOptera;
	if (opteraPoint1ToSpectraDir == 1) {
		rawSpectraPanToOptera = _calibrationPoint[1].spectraPanRad - spectraOpteraToPoint1Angle;  // this is the uncalibrated spectra to optera pan angle
	}
	else {
		rawSpectraPanToOptera = _calibrationPoint[1].spectraPanRad + spectraOpteraToPoint1Angle;
	}
	float ptzReferencePan = rawSpectraPanToOptera - spectraPanToOptera;  // the difference gives the ptz pan correction
	ptzReferencePan = foldOverBoundary(ptzReferencePan, (float)-PI, (float)PI);

	// save results
	_tracking->_cameraInfos._cameraSpectra.setDistanceToOptera(spectraToOpteraDist);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosX(spectraPositionX);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosZ(spectraPositionZ);
	_tracking->_cameraInfos._cameraSpectra.setInstallHeight(spectraHeight);
	_tracking->_cameraInfos._cameraSpectra.setReferencePan(ptzReferencePan);
	_tracking->_options.updateCalibrationParams(&ptzReferencePan, &spectraPositionX, &spectraPositionZ, &spectraToOpteraDist, &spectraHeight, NULL, NULL, NULL, NULL);
	DOUT(" calibratePtz: ptzReferencePan: " << ptzReferencePan << " spectraPositionX: " << spectraPositionX << " spectraPositionZ: " << spectraPositionZ << " spectraToOpteraDist: " << spectraToOpteraDist << " spectraHeight: " << spectraHeight << endl);
	return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::calibratePtzPositionCameraTilts() {

	// Point Optera and Spectra at 4 common points to find PTZ height, PTZ X/Z location, and camera tilts using numeric solver.

	// This routine calibrates the PTZ location and the camera tilts by pointing the PTZ and Optera to 4 common points
	// on the ground plane. This assumes that the PTZ pan calibration has already been performed. This method uses a numeric solver.

#define NUM_POINT_PAIRS 4

	// known parameters
	float optera_h = (float)_tracking->_cameraInfos._cameraOptera.getInstallHeight();
	bool separated_cameras = _tracking->_cameraInfos._cameraSpectra.getSeparatedCameras();

	// copy point pair data
	pointpair_t point_pair[NUM_POINT_PAIRS];

	int ii;
	for (ii = 0; ii < NUM_POINT_PAIRS; ii++) {
		point_pair[ii].optera_pan = _calibrationPoint[ii + 1].opteraPanRad;
		point_pair[ii].optera_tilt = _calibrationPoint[ii + 1].opteraTiltRad;
		point_pair[ii].spectra_pan = _calibrationPoint[ii + 1].spectraPanRad;
		point_pair[ii].spectra_tilt = _calibrationPoint[ii + 1].spectraTiltRad;
	}

	// init unknown variables
	vars_t var;
	var.spectra_H = optera_h;
	var.spectra_X = 0;
	var.spectra_Z = 0;
	var.optera_tilt_err_X = 0;
	var.optera_tilt_err_Z = 0;
	var.spectra_tilt_err_X = 0;
	var.spectra_tilt_err_Z = 0;
	var.spectra_pan_offset = 0;

	// solve for unknown variables
	int num_iter;
	double final_err_feet = 0, max_err_feet = 0;
	numeric_calibrate(point_pair,				            // (I)   point pair inputs
					  NUM_POINT_PAIRS,						// (I)   # of point pairs
					  _tracking->_cameraInfos._cameraType,	// (I)   CAMERA_180/270/360
					  separated_cameras,                    // (I)   optera separated from spectra
					  true,                                 // (I)   optera and spectra can see each other
					  optera_h,								// (I)   height of optera (feet)
					  &var,									// (I/O) unknown vars to be solved for
					  &num_iter,							// (O)   # of iterations solver used
					  &final_err_feet);						// (O)   final total error in point pair matching (feet)

	// save results
	float spectraToOpteraDist = (float)sqrt((var.spectra_X * var.spectra_X) + (var.spectra_Z * var.spectra_Z));
	float spectraX = (float)var.spectra_X, spectraZ = (float)var.spectra_Z, spectraH = (float)var.spectra_H;
	float opteraTiltX = (float)RAD2DEG(var.optera_tilt_err_X), opteraTiltZ = (float)RAD2DEG(var.optera_tilt_err_Z);
	float spectraTiltX = (float)RAD2DEG(var.spectra_tilt_err_X), spectraTiltZ = (float)RAD2DEG(var.spectra_tilt_err_Z);
	_tracking->_cameraInfos._cameraSpectra.setDistanceToOptera(spectraToOpteraDist);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosX(var.spectra_X);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosZ(var.spectra_Z);
	_tracking->_cameraInfos._cameraSpectra.setInstallHeight(var.spectra_H);
	_tracking->_cameraInfos.setPtzTilt((float)var.spectra_tilt_err_X, (float)var.spectra_tilt_err_Z);
	_tracking->_cameraInfos.setOpteraTilt((float)var.optera_tilt_err_X, (float)var.optera_tilt_err_Z);
	_tracking->_options.updateCalibrationParams(NULL, &spectraX, &spectraZ, &spectraToOpteraDist, &spectraH,
		&opteraTiltX, &opteraTiltZ, &spectraTiltX, &spectraTiltZ);
	DOUT(" calibratePtzPositionCameraTilts Results: spectra_H: " << var.spectra_H << " spectra_X: " << var.spectra_X << " spectra_Z: " << var.spectra_Z);
	DOUT(" optera_tilt_err_X: " << RAD2DEG(var.optera_tilt_err_X) << " optera_tilt_err_Z: " << RAD2DEG(var.optera_tilt_err_Z) << " spectra_tilt_err_X: " << RAD2DEG(var.spectra_tilt_err_X) << " spectra_tilt_err_Z: " << RAD2DEG(var.spectra_tilt_err_Z) << endl);
	DOUT(" calibratePtzPositionCameraTilts: num_iter: " << num_iter << " final_err_feet: " << final_err_feet << endl);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::calibratePtzPanPositionCameraTilts() {

	// Point Optera and Spectra at 4 common points to find PTZ height, PTZ X/Z location, camera tilts, and PTZ pan offset using numeric solver.

	// This routine calibrates the PTZ location, the camera tilts, and the PTZ pan offset by pointing the PTZ and Optera to 4 common points
	// on the ground plane. This method has the ability to calibrate the full range of Optera 180 tilts. This method uses a numeric solver.

	// Since the Optera 180 has a large tilt range and this method also solves for the PTZ pan offset, there is an increased risk of the 
	// solver reaching a local minimum. To resolve this, random Optera tilt and PTZ pan starting positions are chosen and multiple iterations
	// are performed, and the result with the lowest error is chosen as the final solution.

#define NUM_POINT_PAIRS 4

	// known parameters
	float optera_h = (float)_tracking->_cameraInfos._cameraOptera.getInstallHeight();
	bool separated_cameras = _tracking->_cameraInfos._cameraSpectra.getSeparatedCameras();

	// copy point pair data
	pointpair_t point_pair[NUM_POINT_PAIRS];

	int ii;
	for (ii = 0; ii < NUM_POINT_PAIRS; ii++) {
		point_pair[ii].optera_pan = _calibrationPoint[ii + 1].opteraPanRad;
		point_pair[ii].optera_tilt = _calibrationPoint[ii + 1].opteraTiltRad;
		point_pair[ii].spectra_pan = _calibrationPoint[ii + 1].spectraPanRad;
		point_pair[ii].spectra_tilt = _calibrationPoint[ii + 1].spectraTiltRad;
	}

	vars_t var;
	vars_t min_err_var;
	srand((unsigned int)(std::time(NULL)));

	double final_err_feet = 0, max_err_feet = 0;
	double min_error = 1.0e9;
	int num_iter, min_err_num_iter;
	for (int n = 0; n < 20; n++) {
		// init unknown variables
		var.spectra_H = optera_h;
		var.spectra_X = 0;
		var.spectra_Z = 0;
		var.optera_tilt_err_X = 0;
		var.optera_tilt_err_Z = 0;
		var.spectra_tilt_err_X = 0;
		var.spectra_tilt_err_Z = 0;
		var.spectra_pan_offset = 0;

		float spectra_X = 0, spectra_Z = 0;
		if (separated_cameras) {
			// select random starting spectra location
			spectra_X = (float)(((rand() / double(RAND_MAX)) * 200) - 100);  // -100 to +100 range
			spectra_Z = (float)(((rand() / double(RAND_MAX)) * 200) - 100);
		}
		var.spectra_X = spectra_X;
		var.spectra_Z = spectra_Z;

		float tilt = 0;
		if (_tracking->_cameraInfos._cameraType = CameraInfos::CAMERA_180) {
			// select random starting optera tilt
			tilt = (float)(((rand() / double(RAND_MAX)) * 180) - 90);  // -90 to +90 range
		}
		// select random starting spectra pan
		float spectra_pan_off = (float)(((rand() / double(RAND_MAX)) * 360) - 180); // -180 to +180 range
		var.optera_tilt_err_Z  = tilt * PI / 180;
		var.spectra_pan_offset = spectra_pan_off * PI / 180;

		// solve for unknown variables
		numeric_calibrate(point_pair,							// (I)   point pair inputs
			              NUM_POINT_PAIRS,						// (I)   # of point pairs
			              _tracking->_cameraInfos._cameraType,  // (I)   CAMERA_180/270/360
						  separated_cameras,                    // (I)   optera separated from spectra
						  true,                                 // (I)   optera and spectra can see each other
			              optera_h,								// (I)   height of optera (feet)
			              &var,									// (I/O) unknown vars to be solved for
			              &num_iter,							// (O)   # of iterations solver used
			              &final_err_feet);						// (O)   final total error in point pair matching (feet)

		// keep track of best solution
		if ((min_error > final_err_feet) || (n == 0)) {
			min_error = final_err_feet;
			min_err_var = var;
			min_err_num_iter = num_iter;
		}
		DOUT("calibratePtzPanPositionCameraTilts: error: " << final_err_feet << " spectra_X_start: " << spectra_X << " spectra_X_end: " << var.spectra_X << " spectra_Z_start: " << spectra_Z << " spectra_Z_end: " << var.spectra_Z << " tilt_start: " << tilt << " tilt_end: " << var.optera_tilt_err_Z * 180 / PI << " pan_start: " << spectra_pan_off << " pan_end: " << var.spectra_pan_offset * 180 / PI << endl);
	}

	// recall best solution
	var = min_err_var;

	// install results
	float spectraToOpteraDist = (float)sqrt((var.spectra_X * var.spectra_X) + (var.spectra_Z * var.spectra_Z));
	float spectraX = (float)var.spectra_X, spectraZ = (float)var.spectra_Z, spectraH = (float)var.spectra_H;
	float opteraTiltX = (float)RAD2DEG(var.optera_tilt_err_X), opteraTiltZ = (float)RAD2DEG(var.optera_tilt_err_Z);
	float spectraTiltX = (float)RAD2DEG(var.spectra_tilt_err_X), spectraTiltZ = (float)RAD2DEG(var.spectra_tilt_err_Z);
	float spectraPanOffset = (float)var.spectra_pan_offset;
	_tracking->_cameraInfos._cameraSpectra.setDistanceToOptera(spectraToOpteraDist);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosX(var.spectra_X);
	_tracking->_cameraInfos._cameraSpectra.setReferencePosZ(var.spectra_Z);
	_tracking->_cameraInfos._cameraSpectra.setInstallHeight(var.spectra_H);
	_tracking->_cameraInfos._cameraSpectra.setReferencePan(var.spectra_pan_offset);
	_tracking->_cameraInfos.setPtzTilt((float)var.spectra_tilt_err_X, (float)var.spectra_tilt_err_Z);
	_tracking->_cameraInfos.setOpteraTilt((float)var.optera_tilt_err_X, (float)var.optera_tilt_err_Z);

	// save results to config file
	_tracking->_options.updateCalibrationParams(&spectraPanOffset, &spectraX, &spectraZ, &spectraToOpteraDist, &spectraH,
		&opteraTiltX, &opteraTiltZ, &spectraTiltX, &spectraTiltZ);
	DOUT(" calibratePtzPositionCameraTiltsPan Results: spectra_H: " << var.spectra_H << " spectra_X: " << var.spectra_X << " spectra_Z: " << var.spectra_Z);
	DOUT(" optera_tilt_err_X(deg): " << RAD2DEG(var.optera_tilt_err_X) << " optera_tilt_err_Z(deg): " << RAD2DEG(var.optera_tilt_err_Z) << " spectra_tilt_err_X(deg): " << RAD2DEG(var.spectra_tilt_err_X) << " spectra_tilt_err_Z(deg): " << RAD2DEG(var.spectra_tilt_err_Z) << " spectra_pan_offset(deg): " << RAD2DEG(var.spectra_pan_offset) << endl);
	DOUT(" calibratePtzPositionCameraTiltsPan: num_iter: " << min_err_num_iter << " final_err_feet: " << min_error << endl);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::savePointMeasurement(int viewX, int viewY, float rawPtzPanNorm, float rawPtzTiltNorm, int pointNum, bool usePtzPanCorrection) {

	float rawPtzPanRad = rawPtzPanNorm * (float)PI;

	if (_tracking->_mosaicToPanTilt.getOpteraPanTiltAngles()) {
		float panRadians = 0, tiltRadians = 0;

		// convert Optera view point to radians
		if (getViewPointRadians(viewX, viewY, panRadians, tiltRadians)) {
			DOUT("savePointMeasurement: ViewPointRadians: " << pointNum << " opteraPanRad: " << panRadians << " opteraTiltRad: " << tiltRadians);
			DOUT(" ptzPanNormRaw: " << rawPtzPanNorm << " ptzTiltNormRaw: " << rawPtzTiltNorm);
			DOUT(" ptzPanRadRaw: " << rawPtzPanRad << " ptzTiltRadRaw: " << _tracking->_cameraInfos.convertOnvifToTilt(rawPtzTiltNorm) << endl);
			_tracking->_mosaicToPanTilt.mapAnalyticPanTilt(panRadians, tiltRadians);
		}
		tiltRadians -= (float)PI_2;  // rotate horizon from PI/2 to 0 
		_calibrationPoint[pointNum].opteraPanRad = panRadians;
		_calibrationPoint[pointNum].opteraTiltRad = tiltRadians;

		// convert Spectra generic Onvif point to radians
		float ptzReferencePanRad = (float)_tracking->_cameraInfos._cameraSpectra.getReferencePan();
		// PTZ Pan Correction can only be used if a valid pan calibration has already been performed
		panRadians = (usePtzPanCorrection) ? rawPtzPanRad - ptzReferencePanRad : rawPtzPanRad;
		panRadians = foldOverBoundary(panRadians, -(float)PI, (float)PI);
		tiltRadians = _tracking->_cameraInfos.convertOnvifToTilt(rawPtzTiltNorm);
		_calibrationPoint[pointNum].spectraPanRad = panRadians;
		_calibrationPoint[pointNum].spectraTiltRad = tiltRadians;
		DOUT("savePointMeasurement: point: " << pointNum << " opteraPanRad: " << _calibrationPoint[pointNum].opteraPanRad << " opteraTiltRad: " << _calibrationPoint[pointNum].opteraTiltRad << " spectraPanRad: " << _calibrationPoint[pointNum].spectraPanRad << " spectraTiltRad : " << _calibrationPoint[pointNum].spectraTiltRad << " rawPtzPanRad: " << rawPtzPanRad << " ptzReferencePanRad: " << ptzReferencePanRad << endl);
	}
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::calcInsideAngle(float absAngle1Rad, float absAngle2Rad, float &insideAngleMagRad, int &insideAngleDir) {
	// Returns the inside angular distance between input angles 1 & 2.
	// The inside angle mag and dir are returned. Direction = 1 indicates CW direction.

	// make sure the inputs are both positive
	if (absAngle1Rad < 0) { absAngle1Rad += 2 * (float)PI; }
	if (absAngle2Rad < 0) { absAngle2Rad += 2 * (float)PI; }

	insideAngleMagRad = abs(absAngle2Rad - absAngle1Rad);
	insideAngleDir = (absAngle2Rad > absAngle1Rad) ? 1 : 0;
	
	// correct for 180 foldover - direction also flips if foldover happens
	if (insideAngleMagRad >(float)PI) { 
		insideAngleMagRad = 2 * (float)PI - insideAngleMagRad;
		insideAngleDir = (insideAngleDir == 1) ? 0 : 1;
	}

	return;
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::calibratePtz(int viewX, int viewY, float rawPtzPanNorm, float rawPtzTiltNorm) {

	// Service desired calibrate method

	// Using last commanded ptz pan instead of reading it back from ptz because of GetStatus resolution issues

	if (_viewGenerator == PelcoImextk_kInvalidViewGen) {
        return false;
    }
	if (_tracking->_mosaicToPanTilt.getOpteraPanTiltAngles()) {

		switch (_saMode) {

		case SA_CALIBRATE_PTZ_PAN_OLD: // PTZ Pan Calibration (calc's Spectra position as well)
			return calibratePtzPan(viewX, viewY, rawPtzPanNorm, true);
			break;

		case SA_CALIBRATE_PTZ_PAN:  // PTZ Pan Calibration
			calibratePtzPan(viewX, viewY, rawPtzPanNorm, false);
			savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 0, false);
			return true;
			break;

		case SA_CALIBRATE_PTZ_PAN_POSITION:  // PTZ Pan and Position Calibration
			switch (_calibrationState) {
			case CAL_PAN_STATE:
				calibratePtzPan(viewX, viewY, rawPtzPanNorm, false);
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 0, false);
				return true;
				break;

			case CAL_POINT_1_STATE:
				// save point 1 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 1, true);

				// solve for Spectra x,y,z location
				calibratePtzPosition();
				return true;
				break;
			}
			break;

		case SA_CALIBRATE_PTZ_PAN_POSITION_BLIND:
			switch (_calibrationState) {
			case CAL_POINT_1_STATE:
				// save point 1 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 1, false);
				return true;
				break;

			case CAL_POINT_2_STATE:
				// save point 2 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 2, false);

				// Solve for Spectra pan & x,y,z location
				calibratePtzPanAndPosition();
				return true;
				break;
			}
			break;

		case SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS:
			switch (_calibrationState) {
			case CAL_POINT_1_STATE:  // Point 1
				// save point 1 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 1, true);
				return true;
				break;

			case CAL_POINT_2_STATE:  // Point 2 
				// save point 2 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 2, true);
				return true;
				break;

			case CAL_POINT_3_STATE:  // Point 3 
				// save point 3 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 3, true);
				return true;
				break;

			case CAL_POINT_4_STATE:  // Point 4 
				// save point 4 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 4, true);

				// Solve for Spectra x,y,z location and camera tilts
				calibratePtzPositionCameraTilts();
				return true;
				break;
			}
			break;

		case SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS:
			switch (_calibrationState) {
			case CAL_POINT_1_STATE:  // Point 1
				// save point 1 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 1, false);
				return true;
				break;

			case CAL_POINT_2_STATE:  // Point 2 
				// save point 2 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 2, false);
				return true;
				break;

			case CAL_POINT_3_STATE:  // Point 3 
				// save point 3 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 3, false);
				return true;
				break;

			case CAL_POINT_4_STATE:  // Point 4 
				// save point 4 measurement
				savePointMeasurement(viewX, viewY, rawPtzPanNorm, rawPtzTiltNorm, 4, false);

				// Solve for Spectra x,y,z location and camera tilts
				calibratePtzPanPositionCameraTilts();
				return true;
				break;
			}
			break;

		default:
			return false;
		}
	}
    return false;
}


//////////////////////////////////////////////////////////////////////////////////
// read cursor position and send ptz pan/tilt to spectra
bool SituationControl::ptzFollowOpteraClick(int mx, int my) {
	if (PelcoImextk_kInvalidViewGen != _viewGenerator) {

		// wait for library to get optera pan/tilt angle
		if (_tracking->_mosaicToPanTilt.getOpteraPanTiltAngles()) {
			float ptzX, ptzY, targetHeight = 0;
			// get previously determined targetHeight (from zoom to box size)
			targetHeight = _targetHeight / 2;  // want center of target
			bool separatedCameras = _tracking->_cameraInfos._cameraSpectra.getSeparatedCameras();
			if (getSlavePanTiltFromAnalytic(mx, my, targetHeight, ptzX, ptzY, separatedCameras)) {
				// add 3x zoom for y less than 0.33
				float zFactor = MAX_ZOOM_FACTOR;
				float yCutoff = (float) 0.33;
				float z = 1;
				if (ptzY > yCutoff) {
					z = (zFactor - 1) * (ptzY - yCutoff) / (1 - yCutoff) + 1;
				}
				z = _tracking->_cameraInfos.convertZoomRatioToOnvif(z);
				DOUT(" ptzFollowOpteraClick: map optera " << " ptz " << ptzX << "x" << ptzY << " zoom " << _tracking->_cameraInfos.convertZoomRatioToOnvif(z) << endl);
				_tracking->_ptzControl.manualSetAbsoluteLocation(ptzX, ptzY, NULL);  // don't change zoom - use zoom to box to set zoom
			}
			return true;
		}
		else {
			DOUT("ptzFollowOpteraClick: OpteraPanTiltAngles not available yet..." << endl);
		}
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::setControlMode(SAControlMode mode) {
    _saMode = mode;
    getViewDimension();
    if (_saMode == SA_AUTO) {
        // start zoom at 1
        _tracking->_ptzControl.manualSetAbsoluteZoom(_tracking->_cameraInfos.convertZoomRatioToOnvif(1));
        _tracking->_ptzControl.setPtzEnable(true);
    }
    else {
        _tracking->_ptzControl.setPtzEnable(false);
	}
	if ((mode == SA_CALIBRATE_PTZ_PAN_OLD) ||
		(mode == SA_CALIBRATE_PTZ_PAN) ||
		(mode == SA_CALIBRATE_PTZ_PAN_POSITION) ||
		(mode == SA_CALIBRATE_PTZ_PAN_POSITION_BLIND) ||
		(mode == SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS) ||
		(mode == SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS)) {
		startCalibration(mode);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::setViewGenerator(PelcoImextk_ViewGen viewGenerator, PelcoImextk_Context imextkContext) {
    _viewGenerator = viewGenerator;
    _tracking->_mosaicToPanTilt.SetViewGenerator(viewGenerator, imextkContext);
    getViewDimension();
}

//////////////////////////////////////////////////////////////////////////////////
void SituationControl::getViewDimension(void) {
    if (PelcoImextk_kInvalidViewGen != _viewGenerator) {
        int width, height;
        pelco_imextk_get_view_size(_viewGenerator, &width, &height);
        if ((_viewWidth != width) || (_viewHeight != height)) {
            // find out view dimension and min/max Y that has video in it
            _viewWidth = width;
            _viewHeight = height;
            DOUT(" view " << width << "x" << height << endl);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::getViewPointRadians(int x, int y, float &panRadians, float &tiltRadians) {
    if (pelco_imextk_view_to_spherical(_viewGenerator, x, y, &panRadians, &tiltRadians) == PELCO_IMEXTK_NO_ERROR) {
        DOUT("getViewPointRadians: point(X x Y) " << x << "x" << y << " radians(Pan x Tilt) " << panRadians << "x" << tiltRadians << endl);
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationControl::getSlavePanTiltFromAnalytic(int viewX, int viewY, float targetHeight, float &spectraPanNorm, float &spectraTiltNorm, bool separatedCameras) {
	float panRadians;
	float tiltRadians;
	if (getViewPointRadians(viewX, viewY, panRadians, tiltRadians)) {
		_tracking->_mosaicToPanTilt.mapAnalyticPanTilt(panRadians, tiltRadians);
		_tracking->_cameraInfos.getSlavePositionFromOpteraRadians(panRadians, tiltRadians, targetHeight, spectraPanNorm, spectraTiltNorm, separatedCameras);
		return true;
	}
	return false;
}

// move ptz camera
void SituationControl::movePtzCamera(int mx, int my) {
    // for 360 move optera in the main window
    if (CameraInfos::CAMERA_360 != _tracking->_cameraInfos._cameraType) {
        // move spectra based on mx
        int half = _viewWidth / 2;
        float diff = ((float)(mx - half)) / half;
        float x, y, z;
        waitForCameraIdle(_tracking->_ptzControl);
        _tracking->_ptzControl.getCameraLocation(x, y, z);
        DOUT(" mx " << mx << " x: " << x << " diff " << diff);
        x = foldOverBoundary(x + diff);
        DOUT(" send ptz " << x << "x" << y << "x" << z << endl);
        _tracking->_ptzControl.manualSetAbsoluteLocation(x, y, &z);
        waitForCameraIdle(_tracking->_ptzControl);
    }
}


//////////////////////////////////////////////////////////////////////////////////
void SituationControl::zoomToBox(int viewX, int viewY, int viewXanchor, int viewYanchor) {

	// Convert box view coordinates to Optera pan/tilt radians
	float opteraPanRad, opteraTiltRad, opteraPanRadAnchor, opteraTiltRadAnchor;
	if (getViewPointRadians(viewX, viewY, opteraPanRad, opteraTiltRad) &&
		getViewPointRadians(viewXanchor, viewYanchor, opteraPanRadAnchor, opteraTiltRadAnchor)) {

		// do wrap and fold corrections
		_tracking->_mosaicToPanTilt.mapAnalyticPanTilt(opteraPanRad, opteraTiltRad);
		_tracking->_mosaicToPanTilt.mapAnalyticPanTilt(opteraPanRadAnchor, opteraTiltRadAnchor);
		
		float leftOpteraPanRad = min(opteraPanRad, opteraPanRadAnchor);    // pan increases from left to right
		float rightOpteraPanRad = max(opteraPanRad, opteraPanRadAnchor);
		float topOpteraTiltRad = max(opteraTiltRad, opteraTiltRadAnchor);  // tilt increases from bottom to top
		float bottomOpteraTiltRad = min(opteraTiltRad, opteraTiltRadAnchor);

		// optera 360 wrap around
		if (_tracking->_cameraInfos._cameraType == CameraInfos::CAMERA_360) {
			if (abs(leftOpteraPanRad - rightOpteraPanRad) > abs((leftOpteraPanRad + 2*PI) - rightOpteraPanRad)) {
				// unwrap the left pan and swap it with the right pan
				float temp = (float)(leftOpteraPanRad + 2*PI);
				leftOpteraPanRad = rightOpteraPanRad;
				rightOpteraPanRad = temp;
			}
		}

		ObjectRect opteraObj;
		float centerOpteraPanRad, centerOpteraTiltRad;
		opteraObj.panLeft = leftOpteraPanRad;
		opteraObj.panRight = rightOpteraPanRad;
		opteraObj.tiltTop = topOpteraTiltRad;
		opteraObj.tiltBottom = bottomOpteraTiltRad;
		opteraObj.getCenter(centerOpteraPanRad, centerOpteraTiltRad);

		SpectraRectObj spectraObj;
		_tracking->getSpectraPosition(opteraObj, spectraObj);

		// check if setting the object height (vertical line) (needed for Click to Point dual camera translation)
		float boxHeight = topOpteraTiltRad - bottomOpteraTiltRad;
		float boxWidth = rightOpteraPanRad - leftOpteraPanRad;
		boxWidth = boxWidth < 0.001f ? 0.001f : boxWidth;  // prevent div by 0
		if (boxHeight / boxWidth > 10) {
			// save height and exit
			_targetHeight = _tracking->_cameraInfos.getTargetHeightFromOpteraRadians(opteraObj);
			DOUT(" zoomToBox: new targetHeight = " << _targetHeight << " height / width = " << boxHeight / boxWidth << endl);
			return;
		}

		// zoom is based on widest box dimension
		float zoom = _tracking->_cameraInfos.convertZoomRatioToOnvif(_tracking->_cameraInfos.getZoomFactor(spectraObj));

		DOUT(" zoomToBox: screen: L= " << min(viewXanchor, viewX) << " R= " << max(viewXanchor, viewX) << " T= " << min(viewYanchor, viewY) << " B= " << max(viewYanchor, viewY) << endl);
		DOUT(" zoomToBox: optera radians: L= " << leftOpteraPanRad << " R= " << rightOpteraPanRad << " T= " << topOpteraTiltRad << " B= " << bottomOpteraTiltRad << " height = " << boxHeight << " width = " << boxWidth << " height / width = " << boxHeight / boxWidth << endl);
		DOUT(" zoomToBox: ptz (norm) L= " << spectraObj.panLeftNorm << " R= " << spectraObj.panRightNorm << " T= " << spectraObj.tiltTopNorm << " B= " << spectraObj.tiltBottomNorm << endl);
		DOUT(" zoomToBox: ptz center (norm): pan= " << spectraObj.panCenterNorm << " tilt= " << spectraObj.tiltCenterNorm << " zoom: " << zoom << endl);
		_tracking->_ptzControl.manualSetAbsoluteLocation(spectraObj.panCenterNorm, spectraObj.tiltCenterNorm, &zoom);
	}
}
