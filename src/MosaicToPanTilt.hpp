//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================

#ifndef __PELCO_IMEXTK_MOSAICTOPANORAMIC_HPP__
#define __PELCO_IMEXTK_MOSAICTOPANORAMIC_HPP__

#include <cstdint>
#include <map>
#include <deque>
#include <sstream>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "soapPTZBindingProxy.h"
#include "mycamera.hpp"
#include "ObjectFilter.hpp"
#include <pelco/imextk/ViewGenerator.h>
#include "UserOptions.hpp"
#include "CameraInfos.hpp"

// convert radians to degrees and vice-versa
#define RAD2DEG(x) ((x) * 180.0 / PI)
#define DEG2RAD(x) ((x) * PI / 180.0)

enum CameraMoveStatus {
    CAMERA_IDLE,
    CAMERA_MOVE,
    CAMERA_UNKNOWN
};

#define MAX_ENDURA_COORDINATE 10000
#define HALF_MAX_ENDURA_COORDINATE (MAX_ENDURA_COORDINATE / 2)

// convert percentage to pixels
int imageCoordinate(int perc, int size);

// convert range (0, MAX_ENDURA_COORDINATE) to range (0, 1)
float normalizeEndura(int perc);

// convert range (0, 1) to range (0, MAX_ENDURA_COORDINATE)
int floatToEndura(float x);

// parse drawing primitives user data
void parseObjects(const unsigned char * buffer, int size, std::vector<ObjectRect>& objects);

// draw a line in an image
void drawLine(int x0, int y0, int x1, int y1, unsigned char *im, int level, const int width, const int size);
// draw rectangle box
void drawBox(int x0, int y0, int x1, int y1, unsigned char *im, int level, int width, int height, int stride);
// wrap value around if value is over boundary
float foldOverBoundary(float x, float lowbound = -1.0, float highbound = 1.0);

struct PtzElement {
	std::shared_ptr<float> _pan_pos;
	std::shared_ptr<float> _tilt_pos;
	std::shared_ptr<float> _pan_tilt_speed;
	std::shared_ptr<float> _zoom;
};

class PtzData {
public:
    PtzData(void);
    ~PtzData(void);
    void setPtzElement(PtzElement &data);
    bool getPtzElement(PtzElement &data);
    void notify(void);
private:
    std::mutex _mutex;
    std::condition_variable _cv;
    std::vector<PtzElement> _data;
};

class PtzThread {
public:
    PtzThread();
    virtual ~PtzThread();
    void startThread();
    virtual void stopThread();
    static DWORD __stdcall startRun(LPVOID args);
    virtual void run();
protected:
    bool                    _runStatus;
private:
    std::mutex              _mutex;
    std::condition_variable _cv;
    HANDLE                  _threadHandle;
    DWORD                   _threadId;
};

class OnvifPtz : public PtzThread {
public:
    OnvifPtz(void);
    virtual ~OnvifPtz(void);
    void setIpAddress(const std::string& ip);
	void setPtzAbsoluteLocation(float *x = NULL, float *y = NULL, float *z = NULL, float *xydt = NULL);
    void run(void);
    bool getAbsoluteLocation(float& x, float& y, float &z);
    bool getMoveStatus(CameraMoveStatus& moveStatus);
private:
    void setPtzAbsoluteLocation(PtzElement& data);
    std::mutex      _mutex;
    std::string     _endPoint;
    std::string     _token;
    std::unique_ptr<PTZBindingProxy> _proxy;
    PtzData         _ptzData;
};

class PtzControl {
    public:
        PtzControl(void);
        virtual ~PtzControl(void);
        void setIpAddress(const std::string& ip);
        // from -1 to 1 for pan 360, -1 to 1 for tilt 90, zoom from 0 to 1
        void setAbsoluteLocation(float pan, float tilt, float *zoom = NULL);
        // z range from 0 to 1 where 0 is no zoom and 1 is maximum zoom
        void setAbsoluteZoom(float zoom);
		float getLastPanTiltSpeed(void) { return _last_pan_tilt_speed; }
        void setPtzEnable(bool enable) {
            _enable = enable;
        };
        // move ptz, bypass enable flags, use in situation control
        void manualSetAbsoluteLocation(float pan, float tilt, float *zoom = NULL);
        bool getCameraLocation(float &pan, float &tilt, float &zoom);
        void manualSetAbsoluteZoom(float zoom);
        bool isCameraIdle(void);
    private:
        bool isEnable(void) { return _enable; }
        bool            _enable;
		OnvifPtz        _onvifPtz;
		float			_last_pan;
		float			_last_tilt;
		float			_last_pan_tilt_speed;
};

struct MosaicToPanTilt {
        MosaicToPanTilt();
        CameraInfos::OpteraCameraTypes setCameraLayoutInfo(const std::string& layoutInfo, int mosaicWidth, int mosaicHeight);
        void SetViewGenerator(PelcoImextk_ViewGen viewGen, PelcoImextk_Context context);
        void mosiacToPanTiltConv(const std::vector<ObjectRect>& mosaic, std::vector<ObjectRect>& pantilt);
        // shift optera pan/tilt origin to analytic origin
        void mapAnalyticPanTilt(float &panRadians, float &tiltRadians);
        // get optera pan/view angles
        bool getOpteraPanTiltAngles(void);

    private:
        void mosaicToPanTilt(const std::vector<ObjectRect>& mosaic, std::vector<ObjectRect>& pantilt);
        void sortMergeObjects(std::vector<ObjectRect>& panTilt);
        // for mosaic image, find out which lines have images
        void findValidMosaicPosition(void);
        // check if objectrects should be merged
        bool mergeCandidate(std::vector<ObjectRect>::iterator it, std::vector<ObjectRect>::iterator st);
        float    _cameraTiltAngle;
        float    _corridorMode;
        std::string    _version;
        // original mosaic dimension
        int      _mosaicWidth;
        int      _mosaicHeight;
        CameraInfos::OpteraCameraTypes _cameraType;
        PelcoImextk_ViewGen  _viewGen;
        PelcoImextk_Context  _imextkContext;
        float                _minPanRadians;
        float                _maxPanRadians;
        float                _minTiltRadians;
        float                _maxTiltRadians;
        float                _centerPanRadians;
        float                _centerTiltRadians;
		float                _panShift;
        std::vector<int>     _mapHeights;
};

struct SituationTracking {
    MosaicToPanTilt   _mosaicToPanTilt;
    PtzControl        _ptzControl;
    CameraInfos       _cameraInfos;
    ObjectFilter      _objectFilter;
	ZoomTimer         _zoomTimer;
	UserOptions&      _options;

	SituationTracking(UserOptions& options);
    ~SituationTracking(void);
    void setLayoutInfo(const std::string& layoutInfo, int mosaicWidth, int mosaicHeight);

	void getSpectraPosition(ObjectRect opteraObj, SpectraRectObj& spectraObj);
	bool checkPtzReCenterNeeded(SpectraRectObj& spectraObj);
    void movePtz(const std::vector<ObjectRect>& mosaic);
    void followObject(float x, float y);
    void setPtzIpAddress(const std::string& ip);

private:
	float			_lastSpectraTiltNorm;
	float			_lastSpectraPanNorm;
	float			_lastZoom;
};

#endif
