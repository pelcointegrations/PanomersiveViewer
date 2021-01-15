//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include <atlstr.h>
#include <algorithm>
#include <functional>
#include <assert.h>
#include <chrono>
#include "CameraInfos.hpp"
#include "Objects.hpp"
#include "ObjectFilter.hpp"
#include "MosaicToPanTilt.hpp"
#include "numeric_calibrate.h"

using namespace std;

//#define _DEBUG_OUTPUT

#ifdef _DEBUG_OUTPUT
#define DOUT( s )              \
{                              \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}
#else
#define DOUT( s )
#endif

namespace {
    // degree to radians
    static const double degreesToRadians(PI / 180);
    static const double radiansToDegrees(180 / PI);
}

float CameraInfos::getTargetHeightFromOpteraRadians(ObjectRect opteraObj) {
	// calculate height of target point above target base
	// NOTE: Assumes horizon = 90deg tilt reference

	float opteraTiltTopRad = opteraObj.tiltTop;
	float opteraTiltBottomRad = opteraObj.tiltBottom;
	float opteraPanCenter, opteraTiltCenter;
	opteraObj.getCenter(opteraPanCenter, opteraTiltCenter);
	translateOpteraCameraToMount(opteraPanCenter, opteraTiltTopRad);
	translateOpteraCameraToMount(opteraPanCenter, opteraTiltBottomRad);

	float groundDistToTargetPoint = (float)_cameraOptera.getInstallHeight() * tan(opteraTiltTopRad);
	float groundDistToTargetBase = (float)_cameraOptera.getInstallHeight() * tan(opteraTiltBottomRad);
	float groundDistTargetPointToBase = groundDistToTargetPoint - groundDistToTargetBase;
	float targetPointHeight = groundDistTargetPointToBase * sin(opteraTiltTopRad) * cos(opteraTiltTopRad);
	DOUT("getTargetHeightFromOpteraRadians: opteraTiltRad: " << opteraTiltTopRad << " opteraTiltBaseRad: " << opteraTiltBottomRad << " groundDistToTargetPoint: " << groundDistToTargetPoint << " groundDistToTargetBase : " << groundDistToTargetBase << " targetPointHeight : " << targetPointHeight << endl);
	return targetPointHeight;
}

//////////////////////////////////////////////////////////////////////////////////////
float CameraInfos::calcCameraTiltError(float panRad, float tiltXrad, float tiltZrad) {

	// This function calculates the camera tilt at a particular pan angle given the tilt
	// about the X-axis and the Z-axis

	// equation assumes counterclockwise increasing pan angle, but optera/spectra pan angles are clockwise increasing
	panRad = -panRad;
	float tiltError = atan((tan(tiltZrad) * cos(panRad)) + (tan(tiltXrad) * sin(panRad)));

	DOUT("calcCameraTiltError: panRad = " << panRad << " tiltXrad = " << tiltXrad << " tiltZrad = " << tiltZrad << " tiltError = " << tiltError << endl);

	return tiltError;
}

////////////////////////////////////////////////////////////////////////////////////
void CameraInfos::translateOpteraCameraToMount(float& opteraPanRad, float& opteraTiltRad) {

	// Performs Camera space to Mount (global) space rotation transform on Optera pan/tilt

	float opteraPanRadOrig = opteraPanRad, opteraTiltRadOrig = opteraTiltRad;
	float optera_pan_offset = 0.0f;
	float opteraReferenceTiltX = (float)_cameraOptera.getReferenceTiltX();
	float opteraReferenceTiltZ = (float)_cameraOptera.getReferenceTiltZ();

	// Calc xyz point at Optera unit sphere in Not-Parallel coordinate space
	float R = 1.0f;
	cart_3d_point_t camera_pnt, mount_pnt;
	camera_pnt.X = R * cos(opteraTiltRad) * cos(-opteraPanRad);  // expecting CCW pan, optera pan is CW
	camera_pnt.Y = R * sin(opteraTiltRad);
	camera_pnt.Z = -R * cos(opteraTiltRad) * sin(-opteraPanRad);  // minus (-R) to adjust for right hand coord. system

	// Apply XYZ rotations to transform this point to Parallel coordinate space
	float c2m_rotation_X = opteraReferenceTiltX;  // tilts are defined mount to camera
	float c2m_rotation_Y = -optera_pan_offset; // optera offset is CW rotation, need CCW for this transform
	float c2m_rotation_Z = opteraReferenceTiltZ;
	mount_pnt = rotateCameraToMount(camera_pnt, c2m_rotation_X, c2m_rotation_Y, c2m_rotation_Z);

	// Calc Optera pan / tilt in Parallel coordinate space
	opteraPanRad = (float)-atan2((double)mount_pnt.Z, (double)mount_pnt.X);  // minus for right hand coord sys
	opteraTiltRad = (float)atan2((double)mount_pnt.Y, sqrt(mount_pnt.X * mount_pnt.X + mount_pnt.Z * mount_pnt.Z));
	DOUT("getOpteraToTargetLocation: opteraPanRad: " << opteraPanRad << " opteraTiltRadOrig: " << opteraTiltRadOrig << " opteraTiltRad(corrected): " << opteraTiltRad << endl);

}

////////////////////////////////////////////////////////////////////////////////////
void CameraInfos::calcOpteraToTargetLocation(float opteraPanRad, float opteraTiltRad, float targetPointHeight, float& correctedTiltRad, float& opteraTargetXFeet, float& opteraTargetZFeet, eTiltRef tiltRef) {

	// convert from camera coordinate system to mount (global) coordinate system
	translateOpteraCameraToMount(opteraPanRad, opteraTiltRad);

	correctedTiltRad = opteraTiltRad;

	// Tilt needs to be referenced to horizon = 0 for these calcs
	if (tiltRef == HORZ_TILT_AT_90_DEG) {
		opteraTiltRad -= (float)PI_2;
	}

	// don't allow to go over horizon
	if (opteraTiltRad > -1.0f * (float)PI / 180)
		opteraTiltRad = -1.0f * (float)PI / 180;

	// calc target X,Z location relative to optera
	float opteraToTargetDist = ((float)_cameraOptera.getInstallHeight() - targetPointHeight) / sin(-opteraTiltRad);
	opteraTargetXFeet = opteraToTargetDist * cos(opteraTiltRad) *  cos(opteraPanRad);  // expects CCW pan rotation
	opteraTargetZFeet = opteraToTargetDist * cos(opteraTiltRad) * -sin(opteraPanRad);  // -sin for right hand coord.
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::getSlavePositionFromOpteraRadians(float opteraPanRad, float opteraTiltRad, float targetPointHeight, float& slavePanNorm, float& slaveTiltNorm, bool separatedCameras) {
	
	float slavePanRad = 0, slaveTiltRad = 0, opteraTiltRadOrig = 0, slaveTiltRadOrig = 0;

	opteraTiltRad -= (float)PI_2;  // translate from horz = 90 to horz = 0

	// see Geometry for Two Cameras at Different Positions writeup by D. Campbell for derivation of equations
	//  - A global coordinate system is defined using an OpenGL right-handed coordinate system where:
	//		- the y-axis is the vertical axis
	//		- the z-axis is the horizontal axis pointing at the viewer
	//		- the x-axis is the horizontal axis pointing to the right
	//		- positive rotation about an axis is counter-clockwise when looking into the center from the positive axis
	//		- the ground plane is defined as the reference plane
	//		- optera X,Z coord's are assigned to reference plane 0,0
	//      - optera pan angle = 0 is aligned with the reference plane X-axis. 
	//      - optera pan angle increases in clockwise direction.
	//      - spectra is calibrated so its pan angle = 0 is also aligned with the reference plane X-axis
	//      - spectra X,Z location in the reference plane is determined at calibration given the distance between the 
	//           cameras and the pan angle of the optera looking at spectra
	//  - The target Y-plane is determined by calculating the height of the center of the target object

	// calc target location coord's in Optera space
	float correctedOpteraTiltRad, opteraTargetX, opteraTargetZ;
	calcOpteraToTargetLocation(opteraPanRad, opteraTiltRad, targetPointHeight, correctedOpteraTiltRad, opteraTargetX, opteraTargetZ, HORZ_TILT_AT_0_DEG);

	// translate target location to Reference space
	float referenceOpteraX = 0, referenceOpteraZ = 0;
	float referenceTargetX = opteraTargetX + referenceOpteraX;
	float referenceTargetZ = opteraTargetZ + referenceOpteraZ;

	// translate target location to Spectra (Parallel) coordinate space
	float slaveTargetXs = referenceTargetX - (float)_cameraSpectra.getReferencePosX();
	float slaveTargetYs = -(float)_cameraSpectra.getInstallHeight();
	float slaveTargetZs = referenceTargetZ - (float)_cameraSpectra.getReferencePosZ();
			
	// Apply XYZ rotations to transform this point to Spectra (Not-Parallel) coordinate space
	cart_3d_point_t slave_camera_target_pnt, slave_mount_target_pnt;
	slave_mount_target_pnt.X = slaveTargetXs;
	slave_mount_target_pnt.Y = slaveTargetYs;
	slave_mount_target_pnt.Z = slaveTargetZs;
	float c2m_rotation_X = (float)_cameraSpectra.getReferenceTiltX();  // tilts are defined mount to camera
	float c2m_rotation_Y = -(float)_cameraSpectra.getReferencePan();   // optera offset is CW rotation, need CCW for this transform
	float c2m_rotation_Z = (float)_cameraSpectra.getReferenceTiltZ();
	slave_camera_target_pnt = rotateMountToCamera(slave_mount_target_pnt, c2m_rotation_X, c2m_rotation_Y, c2m_rotation_Z);

	// calc spectra pan/tilt values
	slavePanRad = (float)atan2(slave_camera_target_pnt.Z, slave_camera_target_pnt.X);
	float slaveTargetBaseDist = (float)sqrt(slave_camera_target_pnt.X * slave_camera_target_pnt.X + slave_camera_target_pnt.Z * slave_camera_target_pnt.Z);
	slaveTiltRad = (float)atan(-(-slave_camera_target_pnt.Y - targetPointHeight) / slaveTargetBaseDist);

	DOUT("getSlavePositionFromOpteraRadians: slaveTargetYns: " << slave_camera_target_pnt.Y << " targetPointHeight: " << targetPointHeight << " opteraTargetX : " << opteraTargetX << " opteraTargetZ : " << opteraTargetZ << " referenceTargetX : " << referenceTargetX << " referenceTargetZ : " << referenceTargetZ << endl);
	DOUT("                                   referenceSlaveX: " << _cameraSpectra.getReferencePosX() << " referenceSlaveZ: " << _cameraSpectra.getReferencePosZ() << " slaveTargetXs: " << slaveTargetXs << " slaveTargetZs: " << slaveTargetZs << " slaveTargetXns: " << slaveTargetXns << " slaveTargetZns: " << slaveTargetZns << endl);
	DOUT("                                   slavePanRad: " << slavePanRad << " slaveTiltRad : " << slaveTiltRad << endl);

	// convert to normalized units
	slavePanRad = foldOverBoundary(slavePanRad, (float)-PI, (float)PI);
	slavePanNorm = convertPanToOnvif(slavePanRad);
	slaveTiltNorm = convertTiltToOnvif(slaveTiltRad);
	DOUT("getSlavePositionFromOpteraRadians: opteraPanRad: " << opteraPanRad << " opteraTiltRad: " << opteraTiltRad << " slavePanRad: " << slavePanRad << " slaveTiltRad: " << slaveTiltRad << " slavePanNorm: " << slavePanNorm << " slaveTiltNorm: " << slaveTiltNorm << " separatedCameras: " << separatedCameras << endl);
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setInstallHeight(float opteraHeight, float spectraHeight)
{
	_cameraOptera.setInstallHeight(opteraHeight);
	_cameraSpectra.setInstallHeight(spectraHeight);
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setConeError(float opteraConeErrorRads, float spectraConeErrorRads)
{
	_cameraOptera.setConeError(opteraConeErrorRads);
	_cameraSpectra.setConeError(spectraConeErrorRads);
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setDistanceToOptera(float distToOptera)
{
	_cameraSpectra.setDistanceToOptera(distToOptera);
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setSeparatedCameras(bool separatedCameras)
{
	_cameraOptera.setSeparatedCameras(separatedCameras);
	_cameraSpectra.setSeparatedCameras(separatedCameras);
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setMinMoveDistance(const float& minAngularMoveDist, const float& minGroundMoveDist)
{
	if (minAngularMoveDist) {
		_cameraOptera.setMinAngularMoveDistance(minAngularMoveDist);
		_cameraSpectra.setMinAngularMoveDistance(minAngularMoveDist);
	}
	if (minGroundMoveDist) {
		_cameraOptera.setMinGroundMoveDistance(minGroundMoveDist);
		_cameraSpectra.setMinGroundMoveDistance(minGroundMoveDist);
	}
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setPtzTilt(float tiltXrads, float tiltZrads)
{
	_cameraSpectra.setReferenceTiltX(tiltXrads);
	_cameraSpectra.setReferenceTiltZ(tiltZrads);
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setOpteraTilt(float tiltXrads, float tiltZrads)
{
	_cameraOptera.setReferenceTiltX(tiltXrads);
	_cameraOptera.setReferenceTiltZ(tiltZrads);
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::convertZoomRatioToOnvif(float zoom)
{
	float z;
	// limit from 1 to 30x
	z = limitValue<float>(zoom, 1, _spectraMaxOpticalZoom);
	// onvif starts at 0
	z -= 1;
	return (z * _spectraZoomStep);  // normalize to 0.0 - 1.0
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::convertOnvifToZoomRatio(float zoom)
{
	float z;
	z = zoom / _spectraZoomStep;
	// onvif started at 0
	z += 1;
	// limit from 1 to 30x
	z = limitValue<float>(z, 1, _spectraMaxOpticalZoom);
	return (z);
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::convertTiltToOnvif(float tiltRad)
{
	// input radians: horz = 0, down = -pi/2
	// output generic: maxTilt = 1, minTilt = -1

	float tiltNorm = ((tiltRad - _spectraMinTiltRad) / _spectraTiltRangeRad) * _spectraTiltRangeNorm + _spectraMinTiltNorm;
	tiltNorm = limitValue(tiltNorm, _spectraMinTiltNorm, _spectraMaxTiltNorm);

	return tiltNorm;
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::convertOnvifToTilt(float tiltNorm)
{
	// input generic: maxTilt = 1, minTilt = -1
	// output radians: horz = 0, down = -pi/2

	float tiltRad = ((tiltNorm - _spectraMinTiltNorm) / _spectraTiltRangeNorm) * _spectraTiltRangeRad + _spectraMinTiltRad;
	tiltRad = limitValue(tiltRad, _spectraMinTiltRad, _spectraMaxTiltRad);

	return tiltRad;
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::convertPanToOnvif(float panRad)
{
	// input radians: -pi to +pi
	// output generic: minPan = -1, maxPan = 1 

	float panNorm = panRad / float(PI);
	panNorm = limitValue(panNorm, -1.0f, 1.0f);

	return panNorm;
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::convertOnvifToPan(float panNorm)
{
	// input generic: minPan = -1, maxPan = 1 
	// output radians: -pi to +pi

	float panRad = panNorm * float(PI);
	panRad = limitValue(panRad, (float)-PI, (float)PI);

	return panRad;
}

//////////////////////////////////////////////////////////////////////////////////
void CameraInfos::setPtzCameraSpec(SpectraModelNumber model)
{
    switch (model) {
	case D6230:
			_cameraSpectra.setHFov(59.5);  // 6230 non-L version
			_cameraSpectra.setVFov(35.6);
			_cameraSpectra.setVAngle(75.0);
			_cameraSpectra.setCameraType(0);
			_spectraMaxOpticalZoom = 30;
			_spectraMaxTiltRad = 3.0f * (float)degreesToRadians;
			_spectraMinTiltRad = -93.0f * (float)degreesToRadians;
			_objectScale = 1.0;
			break;
	case D6230L:
			_cameraSpectra.setHFov(63.7);  // 6230 L version
			_cameraSpectra.setVFov(38.5);
			_cameraSpectra.setVAngle(75.0);
			_cameraSpectra.setCameraType(0);
			_spectraMaxOpticalZoom = 30;
			_spectraMaxTiltRad = 3.0f * (float)degreesToRadians;
			_spectraMinTiltRad = -93.0f * (float)degreesToRadians;
			_objectScale = 1.0;
			break;
	case D6220:
			_cameraSpectra.setHFov(55.4);  // 6220 non-L version
			_cameraSpectra.setVFov(32.9);
			_cameraSpectra.setVAngle(75.0);
			_cameraSpectra.setCameraType(0);
			_spectraMaxOpticalZoom = 20;
			_spectraMaxTiltRad = 3.0f * (float)degreesToRadians;
			_spectraMinTiltRad = -93.0f * (float)degreesToRadians;
			_objectScale = 1.0;
			break;
	case D6220L:
			_cameraSpectra.setHFov(59.5);  // 6220 L version
			_cameraSpectra.setVFov(35.6);
			_cameraSpectra.setVAngle(75.0);
			_cameraSpectra.setCameraType(0);
			_spectraMaxOpticalZoom = 20;
			_spectraMaxTiltRad = 3.0f * (float)degreesToRadians;
			_spectraMinTiltRad = -93.0f * (float)degreesToRadians;
			_objectScale = 1.0;
			break;
		default:
			_spectraMaxOpticalZoom = 20;
			_spectraMaxTiltRad = 0.0f * (float)degreesToRadians;
			_spectraMinTiltRad = -90.0f * (float)degreesToRadians;
			_objectScale = 1.0;
            break;
	}
	_spectraTiltRangeRad = _spectraMaxTiltRad - _spectraMinTiltRad;
	_spectraMaxTiltNorm = 1.0f;
	_spectraMinTiltNorm = -1.0f;
	_spectraTiltRangeNorm = _spectraMaxTiltNorm - _spectraMinTiltNorm;
    _spectraMaxDigitalZoom = 12.0f;
    _spectraZoomStep = (float)(1.0f / (_spectraMaxDigitalZoom * _spectraMaxOpticalZoom));
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::getSpectraZoom(double panFov, double tiltFov) {
	double panScale = tan(_cameraSpectra.getHFov() * degreesToRadians / 2) /
        tan(panFov * degreesToRadians / 2);
	double tiltScale = tan(_cameraSpectra.getVFov() * degreesToRadians / 2) /
        tan(tiltFov * degreesToRadians / 2);
    // zoom takes minimum (widest)
	double zoom = (panScale > tiltScale) ? (tiltScale) : (panScale);
	DOUT("getSpectraZoom: tilt " << tiltFov << " vfov " << _cameraSpectra.getVFov() << " scale " << tiltScale << " zoom " << zoom << endl);
	DOUT("getSpectraZoom: pan " << panFov << " hfov " << _cameraSpectra.getHFov() << " scale " << panScale << " zoom " << zoom << endl);
    return (float)limitValue<double>(zoom, 1.0, _spectraMaxOpticalZoom);
}

//////////////////////////////////////////////////////////////////////////////////
float CameraInfos::getZoomFactor(const SpectraRectObj& spectraObj, double scale) {
	if (spectraObj.radSize() != 0) {
		double panFov = abs((double)(spectraObj.panRightNorm - spectraObj.panLeftNorm)) * 180 * scale; // degree
		double tiltFov = abs((double)(spectraObj.tiltBottomNorm - spectraObj.tiltTopNorm)) * 45 * scale; // degree
		// convert from FOV degrees to a zoom factor (1x to 30x)
        return getSpectraZoom(panFov, tiltFov);
    }
    return (float) 1.0;
}

