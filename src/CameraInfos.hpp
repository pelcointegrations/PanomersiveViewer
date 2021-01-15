//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================

#ifndef __PELCO_IMEXTK_CAMERAINFOS_HPP__
#define __PELCO_IMEXTK_CAMERAINFOS_HPP__

#include <cstdint>
#include <map>
#include <deque>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "mycamera.hpp"
#include "Objects.hpp"


enum SpectraModelNumber {
	D6230,
	D6230L,
	D6220,
	D6220L,
DNONE
};

typedef struct {
	double X;
	double Y;
	double Z;
} cart_3d_point_t;

template <class T> inline T limitValue(T n, T lower = kMinMinY, T upper = kMaxMinY) {
    return (std::max)(lower, (std::min)(n, upper));
};

struct SpectraRectObj {
	float panLeftNorm;
	float panCenterNorm;
	float panRightNorm;
	float tiltTopNorm;
	float tiltCenterNorm;
	float tiltBottomNorm;
	float panFovNorm;
	float tiltFovNorm;
	float radSize(void) const { return abs(panRightNorm - panLeftNorm) * abs(tiltTopNorm - tiltBottomNorm); }
	void getDimensions(float &height, float &width) const {
		width = abs(panLeftNorm - panRightNorm);
		height = abs(tiltTopNorm - tiltBottomNorm);
	}
};

struct CameraInfos {
	enum OpteraCameraTypes {
		CAMERA_360,
		CAMERA_270,
		CAMERA_180,
		CAMERA_NONE
	};
	enum eTiltRef {
		HORZ_TILT_AT_90_DEG,
		HORZ_TILT_AT_0_DEG
	};
	MyCameraOptera _cameraOptera;
	MyCamera	   _cameraSpectra; 

	OpteraCameraTypes _cameraType;
	float       _spectraMaxOpticalZoom;
	float       _spectraMaxDigitalZoom;
	float       _spectraZoomStep;
	float       _objectScale;              // when zoom, large value than 1 scale down zoom value
	float       _spectraMaxTiltRad;
	float       _spectraMinTiltRad;
	float       _spectraTiltRangeRad;
	float       _spectraMaxTiltNorm;
	float       _spectraMinTiltNorm;
	float       _spectraTiltRangeNorm;

	// input: optera pan from (-PI, PI) output spectra pan angle from 1 to -1
	//        optera tilt from (0, PI / 2) output spectra tilt angle from 1 to -1 (0 to -90)
	void    getSlavePositionFromOpteraRadians(float opteraPanRad, float opteraTiltRad, float targetPointHeight, float& slavePanNorm, float& slaveTiltNorm, bool separatedCameras);
	void    calcOpteraToTargetLocation(float opteraPanRad, float opteraTiltRad, float targetPointHeight, float& correctedTiltRad, float& opteraToTargetXFeet, float& opteraToTargetYFeet, eTiltRef tiltRef);
	void	translateOpteraCameraToMount(float& opteraPanRad, float& opteraTiltRad);
	float   getTargetHeightFromOpteraRadians(ObjectRect opteraObj);
	void    setInstallHeight(float opteraHeight, float spectraHeight);
	void    setConeError(float opteraConeError, float spectraConeError);
	void    setDistanceToOptera(float distanceToOptera);
	void    setSeparatedCameras(bool separatedCameras);
	void    setMinMoveDistance(const float& minAngularMoveDistance, const float& minGroundMoveDistance);
	void    setPtzTilt(float tiltX, float tiltZ);
	void    setOpteraTilt(float tiltX, float tiltZ);
	// convert zoom in 1x, 2x to onvif zoom from 0 to 1
	float   convertZoomRatioToOnvif(float zoom);
	float   convertOnvifToZoomRatio(float zoom);
	// convert tilt radians (horz=0, down=-pi/2) to onvif generic (1=max tilt, -1=min tilt)
	float   convertTiltToOnvif(float tiltRad);
	float   convertOnvifToTilt(float tiltNorm);
	// convert pan radians (-pi to +pi) to onvif generic (-1 to +1)
	float   convertPanToOnvif(float panRad);
	float   convertOnvifToPan(float panNorm);
	// use obj dimension to calculate zoom, scale: scale the angle by this factor
	float   getZoomFactor(const SpectraRectObj& spectraObj, double scale = 1.0);
	float   getSpectraZoom(double panFov, double tiltFov);
	// set spectra spec
	void    setPtzCameraSpec(SpectraModelNumber model);
	float   calcCameraTiltError(float panRad, float tiltXrad, float tiltZrad);
};

#endif
