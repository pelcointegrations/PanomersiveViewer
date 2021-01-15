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
#include "MosaicToPanTilt.hpp"
#include "soapDeviceBindingProxy.h"
#include "CameraInfos.hpp"

//show analytic configuration, if not defined, show detected objects
//#define SHOW_CONFIGURATION

using namespace std;

//#define _DEBUG_OUTPUT
//#define _MOVE_DEBUG_OUT

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

#ifdef _MOVE_DEBUG_OUT
#define MOVE_DOUT( s )                       \
{                                            \
   std::wostringstream os_;                  \
   os_ << s;                                 \
   OutputDebugStringW( os_.str().c_str() );  \
}
#else
#define MOVE_DOUT( s )
#endif

namespace {
    // degree to radians
    static const double degreeToRadians(PI / 180);
    static const double radiansToDegree(180 / PI);

    enum ObjectType {
        obj_none = 0,
        obj_person,
        obj_groupOfPeople,
        obj_vehicle,
        obj_objectTypesCount
    };

    enum DrawingType {
        d_none = 0,
        d_line,
        d_polygon,
        d_polygonfilled,
        d_ellipse,
        d_ellipsefilled,
        d_text,
        d_track,
        d_arrow,
        d_trackline,
        d_invalid
    };

    //////////////////////////////////////////////////////////////////////////////////
    unsigned short getUnsignedShort(const uint8_t *p)
    {
        return (unsigned short)(*p << 8 | *(p + 1));
    }

    // add 2 pi to value if less than min
    // if larger than maximum after add, use folder over maximum
    float wrapMinRadians(float value, float minRadians) {
        return (value >= minRadians) ? value : (float)(value + PI * 2);
    }

    // add 2 pi to value if less than min
    // if larger than maximum after add, use folder over maximum
    float wrapRadians(float value, float minRadians, float maxRadians) {
        float useValue = (value >= minRadians) ? value : (float)(value + PI * 2);
        if (useValue > maxRadians) {
            float minDiff = minRadians - value;
            float maxDiff = useValue - maxRadians;
            useValue = (minDiff < maxDiff) ? (minRadians + minDiff) : (maxRadians - maxDiff);
        }
        return useValue;
    }

    //////////////////////////////////////////////////////////////////////////////////
    DrawingType nextDrawingPrimitive(const uint8_t *drawing, int drawingLens)
    {
        if (drawingLens > 1) {
            return (DrawingType)*drawing;
        }
        return d_invalid;
    }

    // Track 13 bytes: 0x07 objectId #vertices vertices[2] color objectType
    // #vertices should be 2
    // all values are one bytes except vertice which is 4 bytes
    // zone configuration: line, polygon 
    // object can be polygon or track. if polygon, it would be four vertices only
    int parseObject(const unsigned char *drawing, int drawingLens, vector<ObjectRect>& objects)
    {
        int len = -1;
        if (drawingLens > 2) {
            int numVertices = drawing[2];
            len = 4 + numVertices * 4;
            ObjectRect object;
            object.id = ((unsigned int)drawing[1] & 0x0ff);
            if (drawingLens >= len) {
                switch (*drawing) {
                    case d_line:
                    case d_ellipse:
                    case d_trackline:
                        break;
                    case d_polygon:
                        {
                            if (numVertices == 4) {
                                // 4 points for rectangle
                                object.left = (int)MAX_ENDURA_COORDINATE;
                                object.right = 0;
                                object.top = (int)MAX_ENDURA_COORDINATE;
                                object.bottom = 0;
                                const unsigned char *data = drawing + 3;
                                for (int i = 0; i < 4; ++i, data += 4) {
                                    object.left = min(object.left, (int)getUnsignedShort(data));
                                    object.right = max(object.right, (int)getUnsignedShort(data));
                                    object.top = min(object.top, (int)getUnsignedShort(data + 2));
                                    object.bottom = max(object.bottom, (int)getUnsignedShort(data + 2));
                                }
                                // check next drawing primitives to see if this is zone configuration
                                DrawingType dtype = nextDrawingPrimitive(drawing + len, drawingLens - len);
#ifdef SHOW_CONFIGURATION
                                if (dtype == d_text || dtype == d_arrow) {  // debug, draw configuration
#else
                                if (dtype != d_text && dtype != d_arrow) {
#endif
                                    // assuming it is object if no text or arrow node follows
                                    objects.push_back(object);
                                }
                            }
                        }
                        break;
                    case d_track:
                        {
                            len += 1;
                            if (numVertices == 2) {
                                // 4 points for rectangle
                                object.left = (unsigned int)getUnsignedShort(drawing + 3);
                                object.top = (unsigned int)getUnsignedShort(drawing + 5);
                                object.right = (unsigned int)getUnsignedShort(drawing + 7);
                                object.bottom = (unsigned int)getUnsignedShort(drawing + 9);
                                //object.objectType = (unsigned int)drawing[12];
                                objects.push_back(object);
                            }
                        }
                        break;
                    case d_polygonfilled:
                    case d_ellipsefilled:
                    case d_arrow:
                        len += 1;
                        break;
                    case d_text:
                        {
                            int textLens = drawing[len];
                            len += (textLens + 1);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
        return len;
    }

    // get spectra model, need to find a better way to know the camera information
    SpectraModelNumber getSpectraModelNumber(const string ip) {
        string endPoint = "http://" + ip + "/onvif/device_service";
        DeviceBindingProxy proxy(endPoint.c_str());
        _tds__GetDeviceInformation information;
        _tds__GetDeviceInformationResponse response;
        SpectraModelNumber modelNumber = DNONE;
        if (proxy.GetDeviceInformation(&information, response) == SOAP_OK) {
			if (response.Model.compare(0, 6, "D6230") == 0) {
				modelNumber = D6230;
			}
			else if (response.Model.compare(0, 6, "D6230L") == 0) {
				modelNumber = D6230L;
			}
			else if (response.Model.compare(0, 6, "D6220") == 0) {
				modelNumber = D6220;
			}
			else if (response.Model.compare(0, 6, "D6220L") == 0) {
				modelNumber = D6220L;
			}
		}
        return modelNumber;
    }

    //////////////////////////////////////////////////////////////////////////////////
    bool mergeEnduraObject(vector<ObjectRect>::iterator it, vector<ObjectRect>::iterator st) {
        if (it->id == st->id) {
            it->tiltTop = max(it->tiltTop, st->tiltTop);
            it->tiltBottom = min(it->tiltBottom, st->tiltBottom);
            it->panLeft = min(it->panLeft, st->panLeft);
            it->panRight = max(it->panRight, st->panRight);
            DOUT(" coor " << it->panLeft << "x" << it->tiltTop << " " << it->panRight << "x" << it->tiltBottom << endl);
            return true;
        }
        return false;
    }

}

// object coordinate in percentage, convert it to image width and height
int imageCoordinate(int perc, int size)
{
    return (int)(size * normalizeEndura(perc));
}

//////////////////////////////////////////////////////////////////////////////////
void parseObjects(const unsigned char *buffer, int size, std::vector<ObjectRect>& objects)
{
    int len;
    while (size > 0 && (len = parseObject(buffer, size, objects)) > 0) {
        buffer += len;
        size -= len;
    }
}

//////////////////////////////////////////////////////////////////////////////////
void drawLine(int x0, int y0, int x1, int y1, unsigned char *im, int color, const int width, const int size)
{
    int fraction;

    int height = size / width;
    x0 = limitValue<int>(x0, 0, width - 1);
    x1 = limitValue<int>(x1, 0, width - 1);
    y0 = limitValue<int>(y0, 0, height - 1);
    y1 = limitValue<int>(y1, 0, height - 1);

    int dy = y1 - y0;
    int dx = x1 - x0;
    int stepx = (dx >= 0) ? 1 : -1;
    int stepy = (dy >= 0) ? width : -width;
    dy = abs(dy) << 1; // abs(dy) * 2
    dx = abs(dx) << 1; // abs(dx) * 2

    y0 *= width;
    y1 *= width;
    im[x0 + y0] = color;

    if (dx > dy) {
        fraction = dy - (dx >> 1);
        while (x0 != x1) {
            if (fraction >= 0) {
                y0 += stepy;
                fraction -= dx;
            }
            x0 += stepx;
            fraction += dy;
            im[x0 + y0] = color;

            //DOUBLE THE THICKNESS
            if (x0 + 1 < width) {
                im[x0 + 1 + y0] = color;
            }
            if (x0 + y0 + width < size) {
                im[x0 + y0 + width] = color;
            }
        }
    }
    else {
        fraction = dx - (dy >> 1);
        while (y0 != y1) {
            if (fraction >= 0) {
                x0 += stepx;
                fraction -= dy;
            }
            y0 += stepy;
            fraction += dx;
            im[x0 + y0] = color;

            //DOUBLE THE THICKNESS
            if (x0 + 1 < width) {
                im[x0 + 1 + y0] = color;
            }
            if (x0 + y0 + width < size) {
                im[x0 + y0 + width] = color;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void drawBox(int x0, int y0, int x1, int y1, unsigned char *im, int level, int width, int height, int stride)
{
    assert(width == stride);
    int size = width * height;
    drawLine(x0, y0, x1, y0, im, level, width, size);
    drawLine(x1, y0, x1, y1, im, level, width, size);
    drawLine(x1, y1, x0, y1, im, level, width, size);
    drawLine(x0, y1, x0, y0, im, level, width, size);
}

// assuming value in loop (like 0 to 360 degree where 0 = 360)
// range = highbound - lowbound
// value < lowboun --> value = value + range
// value > highbound --> value = value - range
float foldOverBoundary(float x, float lowbound, float highbound)
{
    float range = highbound - lowbound;
    return (x > highbound) ? (x - range) : ((x < lowbound) ? (x + range) : x);
}

// normalize to 1
float normalizeEndura(int perc) {
    return (float)perc / (float)MAX_ENDURA_COORDINATE;
}

//////////////////////////////////////////////////////////////////////////////////
int floatToEndura(float x) {
    if (x >= 0.0001 && x <= 1.0) {
        return (int)(x * (float)MAX_ENDURA_COORDINATE);
    }
    else if (x > 1.0) {
        return MAX_ENDURA_COORDINATE;
    }
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////
MosaicToPanTilt::MosaicToPanTilt()
    : _cameraTiltAngle(0),
    _corridorMode(0),
    _version(),
    _mosaicWidth(0),
    _mosaicHeight(0),
    _cameraType(CameraInfos::CAMERA_NONE),
    _viewGen(PelcoImextk_kInvalidViewGen),
    _imextkContext(PelcoImextk_kInvalidContext),
    _minPanRadians(0),
    _maxPanRadians(0),
    _minTiltRadians(0),
    _maxTiltRadians(0),
    _centerPanRadians(0),
    _centerTiltRadians(0),
	_panShift(0),
    _mapHeights()
{
}

//////////////////////////////////////////////////////////////////////////////////
CameraInfos::OpteraCameraTypes MosaicToPanTilt::setCameraLayoutInfo(const std::string& layoutInfo, int width, int height)
{
    string layout(layoutInfo);
    replace(layout.begin(), layout.end(), ',', ' ');
    istringstream ss(layout);
    int numFacets;
    ss >> _version;
    ss >> _cameraTiltAngle;
    ss >> _corridorMode;
    ss >> numFacets;
    _mosaicWidth = width;
    _mosaicHeight = height;
    switch (numFacets) {
        case 5:
            {
                _cameraType = CameraInfos::CAMERA_360;
            }
            break;
        case 4:
            {
                _cameraType = CameraInfos::CAMERA_270;
            }
            break;
        case 2:
            {
                _cameraType = CameraInfos::CAMERA_180;
            }
            break;
        default:
            _cameraType = CameraInfos::CAMERA_NONE;
            break;
    };
    getOpteraPanTiltAngles();
    return _cameraType;
}

//////////////////////////////////////////////////////////////////////////////////
void MosaicToPanTilt::SetViewGenerator(PelcoImextk_ViewGen viewGen, PelcoImextk_Context context)
{
    _viewGen = viewGen;
    _imextkContext = context;
    getOpteraPanTiltAngles();
}

bool MosaicToPanTilt::getOpteraPanTiltAngles(void)
{
    if ((_minPanRadians == 0) && (_maxPanRadians == 0)) {
        float rWidth, rHeight;
        if (pelco_imextk_get_data_bounds(_imextkContext, &_centerPanRadians, &_centerTiltRadians,
            &rWidth, &rHeight) ==
            PELCO_IMEXTK_NO_ERROR) {
            _minPanRadians = _centerPanRadians - (float)(rWidth / 2.0);
            _maxPanRadians = _centerPanRadians + (float)(rWidth / 2.0);
            _minTiltRadians = _centerTiltRadians - (float)(rHeight / 2.0);
            _maxTiltRadians = _centerTiltRadians + (float)(rHeight / 2.0);
        }
        if ((_minPanRadians != 0) && (_maxPanRadians != 0)) {
            DOUT("getOpteraPanTiltAngles:  pan range: " << _minPanRadians << " x " << _maxPanRadians << " tilt range: " << _minTiltRadians << " x " << _maxTiltRadians << " center coord: " << _centerPanRadians << "x" << _centerTiltRadians << endl);
            // also set optera pan offset
            if (CameraInfos::CAMERA_360 == _cameraType) {
                _panShift = (float)(-PI / 4);
            }
            findValidMosaicPosition();
            return true;
        }
        return false;
    }
    return true;
}

// set tilt range 4.71 to 6.28
// set pan range based on model
void MosaicToPanTilt::mapAnalyticPanTilt(float &panRadians, float &tiltRadians) {
    // hard limit at 4.71 to 6.28
	DOUT("mapAnalyticPanTilt: in: panRadians: " << panRadians << " tiltRadians: " << tiltRadians << endl);
	if (_cameraType != CameraInfos::CAMERA_360) {
		tiltRadians = wrapMinRadians(tiltRadians, _minTiltRadians);  // 360: wraps too soon at this limit
	}
    tiltRadians = limitValue<float>(tiltRadians, (float)(3 * PI_2), (float)(5 * PI_2));
    tiltRadians -= (float)(3 * PI_2);

    panRadians = wrapRadians(panRadians, _minPanRadians, _maxPanRadians);
	panRadians -= (_centerPanRadians + _panShift);
    // foldover for 360 model due to non-zero _panShift
    panRadians = foldOverBoundary(panRadians, _minPanRadians - _centerPanRadians, _maxPanRadians - _centerPanRadians);
	DOUT("mapAnalyticPanTilt: out : panRadians: " << panRadians << " tiltRadians: " << tiltRadians << endl);
}

void MosaicToPanTilt::mosiacToPanTiltConv(const std::vector<ObjectRect>& mosaic, std::vector<ObjectRect>& pantilt)
{
    pantilt.clear();
    switch (_cameraType) {
        case CameraInfos::CAMERA_360:
        case CameraInfos::CAMERA_270:
        case CameraInfos::CAMERA_180:
            mosaicToPanTilt(mosaic, pantilt);
            break;
        default:
            pantilt = mosaic;
            break;
    };
}

//////////////////////////////////////////////////////////////////////////////////
void MosaicToPanTilt::mosaicToPanTilt(const std::vector<ObjectRect>& mosaic, std::vector<ObjectRect>& pantilt)
{
    for (auto obj : mosaic) {
        int left = imageCoordinate(obj.left, _mosaicWidth);
        int right = imageCoordinate(obj.right, _mosaicWidth);
        int top = imageCoordinate(obj.top, _mosaicHeight);
        int bottom = imageCoordinate(obj.bottom, _mosaicHeight);
        DOUT( "mosaicToPanTilt: object id " << obj.id << " rect: " << obj.left << "x" << obj.top << " " << obj.right << "x" << obj.bottom << endl);
        DOUT(" mosaicToPanTilt: rect: " << left << "x" << top << " " << right << "x" << bottom << endl);
        float leftPanRadians, rightPanRadians, topTiltRadians, bottomTiltRadians;
        if (pelco_imextk_gl_mosaic_pos_to_spherical(_viewGen, leftPanRadians, topTiltRadians, left, _mapHeights[top])
            == PELCO_IMEXTK_NO_ERROR && pelco_imextk_gl_mosaic_pos_to_spherical(_viewGen, rightPanRadians, bottomTiltRadians, right, _mapHeights[bottom]) == PELCO_IMEXTK_NO_ERROR) {
			DOUT(" mapAnalyticPanTilt (before): spherical: leftPanRadians: " << leftPanRadians << " topTiltRadians: " << topTiltRadians << " rightPanRadians: " << rightPanRadians << " bottomTiltRadians : " << bottomTiltRadians << endl);
            mapAnalyticPanTilt(leftPanRadians, topTiltRadians);
			mapAnalyticPanTilt(rightPanRadians, bottomTiltRadians);
			DOUT(" mapAnalyticPanTilt (after): spherical: leftPanRadians: " << leftPanRadians << " topTiltRadians: " << topTiltRadians << " rightPanRadians: " << rightPanRadians << " bottomTiltRadians : " << bottomTiltRadians << endl);
            // wrap around for optera 360
			if (rightPanRadians < leftPanRadians && leftPanRadians - rightPanRadians > PI && _cameraType == CameraInfos::CAMERA_360) {
                rightPanRadians += (float) (2 * PI);
            }
            float minPan = min(leftPanRadians, rightPanRadians);
            float maxPan = max(leftPanRadians, rightPanRadians);
            float minTilt = min(topTiltRadians, bottomTiltRadians);
            float maxTilt = max(topTiltRadians, bottomTiltRadians);
            obj.panLeft = minPan;
            obj.panRight = maxPan;
            obj.tiltTop = maxTilt;
            obj.tiltBottom = minTilt;
			DOUT("mosaicToPanTilt: map min " << obj.panLeft << "x" << obj.tiltTop << " max " << obj.panRight << "x" << obj.tiltBottom << endl);
			//assert(obj.radSize() > 0);
			if (obj.radSize() > 0) {
				pantilt.push_back(obj);
			}
        }
    }
    sortMergeObjects(pantilt);
}

//////////////////////////////////////////////////////////////////////////////////
bool MosaicToPanTilt::mergeCandidate(vector<ObjectRect>::iterator it, vector<ObjectRect>::iterator st)
{
    static float hTolerance = (float) 0.025;
    static float sTolerance = (float) 0.25;
    // merge right
    if (abs(it->tiltTop - st->tiltTop) < sTolerance && abs(it->tiltBottom - st->tiltBottom) < sTolerance && abs(it->panRight - st->panLeft) < hTolerance) {
        return true;
    }
    static float vTolerance = (float) 0.15;
    // merge down
    if (abs(it->panLeft - st->panLeft) < sTolerance && abs(it->panRight - st->panRight) < sTolerance) {
        if ((abs(it->tiltTop - st->tiltBottom) < vTolerance) || (abs(it->tiltBottom - st->tiltTop) < vTolerance)) {
            return true;
        }
    }
    return false;
}
//////////////////////////////////////////////////////////////////////////////////
void MosaicToPanTilt::sortMergeObjects(vector<ObjectRect>& panTilt) {
    // sort according to left coordinate, ascending order
    sort(panTilt.begin(), panTilt.end(), [](const ObjectRect &a, const ObjectRect &b) ->bool { return a.panLeft < b.panLeft; });
    // check for candidate to merge
    for (vector<ObjectRect>::iterator it = panTilt.begin(); it != panTilt.end(); ++it) {
        vector<ObjectRect>::iterator st = it;
        for (++st; st != panTilt.end(); ++st) {
            if (mergeCandidate(it, st)) {
                st->id = it->id;
            }
        }
    }
    // check for merge
    for (vector<ObjectRect>::iterator it = panTilt.begin(); it != panTilt.end(); ++it) {
        vector<ObjectRect>::iterator st = it;
        for (++st; st != panTilt.end();) {
            if (mergeEnduraObject(it, st)) {
                st = panTilt.erase(st);
            }
            else {
                ++st;
            }
        }
    }
    // sort according to size, descending order
    sort(panTilt.begin(), panTilt.end(), [](const ObjectRect &a, const ObjectRect &b) ->bool { return a.radSize() > b.radSize(); });
}

//////////////////////////////////////////////////////////////////////////////////
void MosaicToPanTilt::findValidMosaicPosition(void)
{
    _mapHeights.clear();
    int lastj = 0;
    float pan, tilt;
    for (int j = 0; j <= _mosaicHeight; ++j) {
        if (pelco_imextk_gl_mosaic_pos_to_spherical(_viewGen, pan, tilt, 0, j) == PELCO_IMEXTK_NO_ERROR) {
            lastj = j;
        }
        _mapHeights.push_back(lastj);
    }
}

//////////////////////////////////////////////////////////////////////////////////
PtzData::PtzData(void)
    : _mutex(),
    _cv(),
    _data()
{
}

//////////////////////////////////////////////////////////////////////////////////
PtzData::~PtzData(void)
{
    notify();
}

//////////////////////////////////////////////////////////////////////////////////
void PtzData::notify(void)
{
    _cv.notify_one();
}

//////////////////////////////////////////////////////////////////////////////////
void PtzData::setPtzElement(PtzElement &data)
{
    unique_lock<mutex> lk(_mutex);
    if (!_data.empty()) {
        DOUT("drop " << _data.size() << " ptz commands" << endl);
        _data.clear();
    }
    _data.push_back(data);
    _cv.notify_one();
}

//////////////////////////////////////////////////////////////////////////////////
bool PtzData::getPtzElement(PtzElement &data)
{
    unique_lock<mutex> lk(_mutex);
    _cv.wait_for(lk, std::chrono::seconds(2));
    if (!_data.empty()) {
        data = _data.back();
        _data.pop_back();
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////
PtzControl::PtzControl(void)
    :_enable(true),
	_onvifPtz(),
	_last_pan(FLT_MAX),
	_last_tilt(FLT_MAX),
	_last_pan_tilt_speed(0)
{
}

//////////////////////////////////////////////////////////////////////////////////
PtzControl::~PtzControl(void)
{
}

//////////////////////////////////////////////////////////////////////////////////
void PtzControl::setIpAddress(const string& ip)
{
    _onvifPtz.setIpAddress(ip);
}

// pan & tilt are in the range of (-1 to 1)
void PtzControl::manualSetAbsoluteLocation(float pan, float tilt, float *zoom)
{
	// determine proportional pan/tilt speed
	float speed_factor = 2.0f;
	float pan_tilt_speed = 0.5;
	
	if ((_last_pan >= -1) && (_last_pan <= 1) && (_last_tilt >= -1) && (_last_pan <= 1)) {
		float delta_pan = abs(pan - _last_pan);
		float delta_tilt = abs(tilt - _last_tilt);
		float delta_position = sqrt((delta_pan * delta_pan) + (delta_tilt * delta_tilt));
		pan_tilt_speed = delta_position * speed_factor;
		pan_tilt_speed = limitValue<float>(pan_tilt_speed, (float)(0.05), (float)(1.0));
	}
	if (!isCameraIdle()) {
		// if still moving, use greater of last speed or new speed
		pan_tilt_speed = (pan_tilt_speed > _last_pan_tilt_speed) ? pan_tilt_speed : _last_pan_tilt_speed;
	}
    DOUT("manualSetAbsoluteLocation: pan: " << pan << " tilt: " << tilt << endl);
    pan = foldOverBoundary(pan);
    _onvifPtz.setPtzAbsoluteLocation(&pan, &tilt, zoom, &pan_tilt_speed);
	_last_pan = pan;
	_last_tilt = tilt;
	_last_pan_tilt_speed = pan_tilt_speed;
}

//////////////////////////////////////////////////////////////////////////////////
void PtzControl::manualSetAbsoluteZoom(float zoom)
{
    _onvifPtz.setPtzAbsoluteLocation(NULL, NULL, &zoom);
}

// pan & tilt are in the range of (-1 to 1)
void PtzControl::setAbsoluteLocation(float pan, float tilt, float *zoom)
{
    if (isEnable()) {
        manualSetAbsoluteLocation(pan, tilt, zoom);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PtzControl::setAbsoluteZoom(float zoom)
{
    if (isEnable()) {
        manualSetAbsoluteZoom(zoom);
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool PtzControl::getCameraLocation(float &pan, float &tilt, float &zoom)
{
    return _onvifPtz.getAbsoluteLocation(pan, tilt, zoom);
}


//////////////////////////////////////////////////////////////////////////////////
bool PtzControl::isCameraIdle(void)
{
    CameraMoveStatus status;
    if (_onvifPtz.getMoveStatus(status)) {
        if (CAMERA_IDLE == status) {
            return true;
        }
    }
    return false;
}

PtzThread::PtzThread()
    : _mutex(),
    _cv(),
    _threadHandle(),
    _threadId(0),
    _runStatus(false)
{
}

PtzThread::~PtzThread() {
    stopThread();
    DOUT("PtzThread::~PtzThread\n");
}

void PtzThread::startThread() {
    if (_threadId == 0) {
        _runStatus = true;
        _threadHandle = CreateThread(NULL, 0, PtzThread::startRun, (LPVOID) this, 0, &_threadId);
    }
}

void PtzThread::stopThread() {
    _runStatus = false;
    _cv.notify_all();
    if (_threadId != 0) {
        WaitForSingleObject(_threadHandle, INFINITE);
        CloseHandle(_threadHandle);
        _threadId = 0;
    }
}

DWORD PtzThread::startRun(LPVOID args) {
    try {
        PtzThread* pt = (PtzThread *)args;
        pt->run();
    }
    catch (exception &e) {
        DOUT(" exception: " << e.what());
    }
    DOUT("thread startrun exit\n");
    return 0;
}

void PtzThread::run() {
    while (_runStatus) {
        unique_lock<mutex> lk(_mutex);
        _cv.wait_for(lk, chrono::seconds(2));
    }
}

//////////////////////////////////////////////////////////////////////////////////
OnvifPtz::OnvifPtz(void)
    : _mutex(),
    _endPoint(),
    _token(),
    _proxy()
{
}

//////////////////////////////////////////////////////////////////////////////////
OnvifPtz::~OnvifPtz(void) 
{
    _ptzData.notify();
    stopThread();
}

//////////////////////////////////////////////////////////////////////////////////
void OnvifPtz::setIpAddress(const string& ip)
{
    lock_guard<mutex> lk(_mutex);
    _endPoint = "http://" + ip + "/onvif/ptz_service";
    _proxy.reset(new PTZBindingProxy(_endPoint.c_str()));
    if (_proxy.get()) {
        _tptz__GetConfigurations conf;
        _tptz__GetConfigurationsResponse response;
        if (_proxy->GetConfigurations(&conf, response) == SOAP_OK) {
            _token = response.PTZConfiguration[0]->token;
        }
        startThread();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void OnvifPtz::setPtzAbsoluteLocation(float *x, float *y, float *z, float *xydt) {
    PtzElement element;
    if (x != NULL) {
        element._pan_pos = make_shared<float>(*x);
    }
    if (y != NULL) {
        element._tilt_pos = make_shared<float>(*y);
    }
    if (z != NULL) {
        element._zoom = make_shared<float>(*z);
    }
	if (xydt != NULL) {
		element._pan_tilt_speed = make_shared<float>(*xydt);
	}
    _ptzData.setPtzElement(element);
}

//////////////////////////////////////////////////////////////////////////////////
void OnvifPtz::setPtzAbsoluteLocation(PtzElement& data) {
    lock_guard<mutex> lk(_mutex);
    if (_proxy.get() != NULL) {
		float *x = data._pan_pos.get();
        float *y = data._tilt_pos.get();
        float *xydt = data._pan_tilt_speed.get();
        float *z = data._zoom.get();
        _tptz__AbsoluteMove move;
        tt__PTZVector position;

        move.Position = &position;
		tt__PTZSpeed speed;
		tt__Vector2D  panTiltSpeed;
		speed.PanTilt = &panTiltSpeed;
		move.Speed = &speed;
		DOUT("move ptz: ");
        tt__Vector2D  panTilt;
        if (x != NULL && y != NULL && *x >= -1 && *x <= 1 && *y >= -1 && *y <= 1) {
            position.PanTilt = &panTilt;
            panTilt.x = (float)*x;
            panTilt.y = (float)*y;
            DOUT(" panTilt: x=" << *x << ", y=" << *y);
            
			// y speed is not used. x speed is used for both pan and tilt
			if (xydt != NULL && *xydt >= 0 && *xydt <= 1) {
				speed.PanTilt = &panTiltSpeed;
				panTiltSpeed.x = (float)*xydt;
				panTiltSpeed.y = (float)*xydt;
			}
			else {
				panTiltSpeed.x = (float)1.0f;
				panTiltSpeed.y = (float)1.0f;
			}
			DOUT(" speed: " << panTiltSpeed.x);
        }
        tt__Vector1D zoom;
        if (z != NULL) {
            position.Zoom = &zoom;
            zoom.x = *z;
			DOUT(" zoom: " << *z);
        }
		DOUT(endl);
        move.ProfileToken = _token;
        _tptz__AbsoluteMoveResponse response;
        _proxy->AbsoluteMove(&move, response);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void OnvifPtz::run(void)
{
    while (_runStatus) {
        PtzElement data;
        if (_ptzData.getPtzElement(data)) {
            if (_runStatus) {
                setPtzAbsoluteLocation(data);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool OnvifPtz::getAbsoluteLocation(float& x, float& y, float &z) {
    x = 0;
    y = 0;
    z = 0;
    bool flag = false;
    lock_guard<mutex> lk(_mutex);
    if (_proxy.get() != NULL) {
        _tptz__GetStatus status;
        _tptz__GetStatusResponse response;
        status.ProfileToken = _token;
        _proxy->GetStatus(&status, response);
        if (response.PTZStatus != NULL && response.PTZStatus->Position != NULL) {
            if (response.PTZStatus->Position->PanTilt != NULL) {
                x = response.PTZStatus->Position->PanTilt->x;
                y = response.PTZStatus->Position->PanTilt->y;
                flag = true;
            }
            if (response.PTZStatus->Position->Zoom != NULL) {
                z = response.PTZStatus->Position->Zoom->x;
                flag &= true;
            }
        }
    }
    return flag;
}

//////////////////////////////////////////////////////////////////////////////////
bool OnvifPtz::getMoveStatus(CameraMoveStatus& moveStatus) {
    bool flag = false;
    moveStatus = CAMERA_MOVE;
    lock_guard<mutex> lk(_mutex);
    if (_proxy.get() != NULL) {
        _tptz__GetStatus status;
        _tptz__GetStatusResponse response;
        status.ProfileToken = _token;
        _proxy->GetStatus(&status, response);
        if (response.PTZStatus != NULL && response.PTZStatus->MoveStatus != NULL) {
            if (*response.PTZStatus->MoveStatus->PanTilt == tt__MoveStatus__IDLE &&
                *response.PTZStatus->MoveStatus->Zoom == tt__MoveStatus__IDLE) {
                moveStatus = CAMERA_IDLE;
            }
            flag = true;
        }
    }
    return flag;
}

//////////////////////////////////////////////////////////////////////////////////
SituationTracking::SituationTracking(UserOptions& options)
    : _mosaicToPanTilt()
    , _ptzControl()
	, _cameraInfos()
	, _objectFilter(_cameraInfos)
    , _zoomTimer()
	, _lastSpectraPanNorm(0)
	, _lastSpectraTiltNorm(0)
	, _lastZoom(0)
	, _options(options)
{
	_cameraInfos._cameraOptera = MyCameraOptera(360, 71.5, 90 - 31.5, 360, MIN_ANGULAR_DIST_MOVED, MIN_GROUND_DIST_MOVED);  // 31.5 tilt standard
	_cameraInfos._cameraSpectra = MyCamera(63.7, 38.5, 75.0, 0, MIN_ANGULAR_DIST_MOVED, MIN_GROUND_DIST_MOVED);             // 6230 Spectra
    _cameraInfos._spectraMaxDigitalZoom = 12;
    _cameraInfos._spectraMaxOpticalZoom = 30;
    _cameraInfos._spectraZoomStep = (float)(1.0 / (_cameraInfos._spectraMaxDigitalZoom * _cameraInfos._spectraMaxOpticalZoom));
}

SituationTracking::~SituationTracking(void) {
}

//////////////////////////////////////////////////////////////////////////////////
void SituationTracking::setLayoutInfo(const std::string& layoutInfo, int mosaicWidth, int mosaicHeight)
{
	CameraInfos::OpteraCameraTypes type = _mosaicToPanTilt.setCameraLayoutInfo(layoutInfo,
		mosaicWidth, mosaicHeight);
	_cameraInfos._cameraType = type;

	// now that we know the camera type, update the optera specs
	switch (type) {
		case CameraInfos::CAMERA_360:
		case CameraInfos::CAMERA_NONE:
			_cameraInfos._cameraOptera.setHFov(360);
			_cameraInfos._cameraOptera.setVFov(71.5);
			_cameraInfos._cameraOptera.setVAngle(90 - 31.5); // 31.5 tilt standard 360
			_cameraInfos._cameraOptera.setCameraType(360);
			break;
		case CameraInfos::CAMERA_270:
			_cameraInfos._cameraOptera.setHFov(270);
			_cameraInfos._cameraOptera.setVFov(73.5);
			_cameraInfos._cameraOptera.setVAngle(90 - 24.5); // 22.5 tilt standard 270
			_cameraInfos._cameraOptera.setCameraType(270);
			break;
		case CameraInfos::CAMERA_180:
			_cameraInfos._cameraOptera.setHFov(180);
			_cameraInfos._cameraOptera.setVFov(40.0);
			_cameraInfos._cameraOptera.setVAngle(90);        // 0 tilt standard 180
			_cameraInfos._cameraOptera.setCameraType(180);
			break;
		default:
			break;
	}
}

//////////////////////////////////////////////////////////////////////////////////
void SituationTracking::getSpectraPosition(ObjectRect opteraObj, SpectraRectObj& spectraObj) {

	// calc height of target above target base
	float targetHeight = _cameraInfos.getTargetHeightFromOpteraRadians(opteraObj);

	// convert from optera object center in radians to PTZ normalized units
	float opteraPanCenterRad, opteraTiltCenterRad;
	opteraObj.getCenter(opteraPanCenterRad, opteraTiltCenterRad);
	bool separatedCameras = _cameraInfos._cameraSpectra.getSeparatedCameras();
	_cameraInfos.getSlavePositionFromOpteraRadians(opteraPanCenterRad, opteraTiltCenterRad, targetHeight / 2.0f, spectraObj.panCenterNorm, spectraObj.tiltCenterNorm, separatedCameras);
	
	// translate optera object pan/tilt edge points (radians) to spectra pan/tilt points (normalized)
	_cameraInfos.getSlavePositionFromOpteraRadians(opteraObj.panLeft, opteraObj.tiltTop, targetHeight, spectraObj.panLeftNorm, spectraObj.tiltTopNorm, separatedCameras);
	_cameraInfos.getSlavePositionFromOpteraRadians(opteraObj.panRight, opteraObj.tiltBottom, 0, spectraObj.panRightNorm, spectraObj.tiltBottomNorm, separatedCameras);
}

//////////////////////////////////////////////////////////////////////////////////
bool SituationTracking::checkPtzReCenterNeeded(SpectraRectObj& spectraObj) {

#define BOUNDARY_SIZE_PERCENT 75.0f

	// determine current Spectra pan/tilt FOV
	spectraObj.panFovNorm = 2.0f * (float)atan(tan(_cameraInfos._cameraSpectra.getHFov() * degreeToRadians / 2.0) / _cameraInfos.convertOnvifToZoomRatio(_lastZoom)) / (float)PI;
	spectraObj.tiltFovNorm = 2.0f * (float)atan(tan(_cameraInfos._cameraSpectra.getVFov() * degreeToRadians / 2.0) / _cameraInfos.convertOnvifToZoomRatio(_lastZoom)) / (float)PI;

	// calc Spectra FOV boundary width
	float panFovBoundaryWidthNorm = (spectraObj.panFovNorm  * BOUNDARY_SIZE_PERCENT) / 100.0f;
	float tiltFovBoundaryWidthNorm = (spectraObj.tiltFovNorm * BOUNDARY_SIZE_PERCENT) / 100.0f;

	// calc Spectra boundary limit edges
	float leftPanBoundaryNorm = _lastSpectraPanNorm - (panFovBoundaryWidthNorm / 2.0f);
	float rightPanBoundaryNorm = _lastSpectraPanNorm + (panFovBoundaryWidthNorm / 2.0f);
	float topTiltBoundaryNorm = _lastSpectraTiltNorm + (tiltFovBoundaryWidthNorm / 2.0f);
	float bottomTiltBoundaryNorm = _lastSpectraTiltNorm - (tiltFovBoundaryWidthNorm / 2.0f);
	//DOUT("checkPtzReCenterNeeded: zoomRatio: " << _cameraInfos.convertOnvifToZoomRatio(_lastZoom) << " HFov: " << _cameraInfos._cameraSpectra.getHFov() << "panFovNorm(deg): " << spectraObj.panFovNorm * PI * radiansToDegree << endl);
	//DOUT("checkPtzReCenterNeeded: zoomRatio: " << _cameraInfos.convertOnvifToZoomRatio(_lastZoom) << " VFov: " << _cameraInfos._cameraSpectra.getVFov() << "tiltFovNorm(deg): " << spectraObj.tiltFovNorm * PI * radiansToDegree << endl);
	//DOUT("checkPtzReCenterNeeded: OBJ Norm: Left: " << spectraObj.panLeftNorm << " right: " << spectraObj.panRightNorm << " top: " << spectraObj.tiltTopNorm << " bottom: " << spectraObj.tiltBottomNorm);
	//DOUT(" LIMIT Norm: Left: " << leftPanBoundaryNorm << " right: " << rightPanBoundaryNorm << " top: " << topTiltBoundaryNorm << " bottom: " << bottomTiltBoundaryNorm);
	//DOUT(" Hit Limit (L R T B): " << (spectraObj.panLeftNorm < leftPanBoundaryNorm) << " " << (spectraObj.panRightNorm > rightPanBoundaryNorm) << " " << (spectraObj.tiltTopNorm > topTiltBoundaryNorm) << " " << (spectraObj.tiltBottomNorm < bottomTiltBoundaryNorm) << endl);

	// check if tracking object is crossing the boundary
	if ((spectraObj.panLeftNorm < leftPanBoundaryNorm) ||
		(spectraObj.panRightNorm > rightPanBoundaryNorm) ||
		(spectraObj.tiltTopNorm > topTiltBoundaryNorm) ||
		(spectraObj.tiltBottomNorm < bottomTiltBoundaryNorm)) {
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
void SituationTracking::movePtz(const std::vector<ObjectRect>& mosaic) {
	// mosaic contains an array of analytic object rectangles for a frame

    // wait for library to get optera pan/tilt angle
    if (_mosaicToPanTilt.getOpteraPanTiltAngles()) {
        vector<ObjectRect> panTilt;

		// convert array of mosaic coordinates to cube face radians, merge objects, sort by size
        _mosaicToPanTilt.mosiacToPanTiltConv(mosaic, panTilt);
		ObjectRect opteraObj = {};
		SpectraRectObj spectraObj = {};
		spectraObj.panCenterNorm = _lastSpectraPanNorm;
		spectraObj.tiltCenterNorm = _lastSpectraTiltNorm;
		float opteraObjHeightRad = 0, opteraObjWidthRad = 0;
		float spectraObjHeightNorm = 0, spectraObjWidthNorm = 0;
		float speed = 0, aveSpeed = 0;
		float rawZoom = 0, zoom = 0;
		int zoomState = 0;
		bool changePosition = false, changeZoom = false, blockMove = false;
		// determine which object to point the PTZ at 
		if (_objectFilter.hasTrackObject(panTilt, opteraObj)) {

			// translate optera object pan/tilt points (radians) to spectra tilt/pan points (norm)
			getSpectraPosition(opteraObj, spectraObj);

			// get generic zoom (0.0 to 1.0) equivalent to object size
			rawZoom = _cameraInfos.convertZoomRatioToOnvif(_cameraInfos.getZoomFactor(spectraObj, _cameraInfos._objectScale));
			_objectFilter.getObjectSpeed(opteraObj.id, speed, aveSpeed);
			zoom = _zoomTimer.determineNextZoom(rawZoom, speed, aveSpeed, _cameraInfos._spectraZoomStep, &zoomState);

			// check if near edge of PTZ view
			if (!checkPtzReCenterNeeded(spectraObj)) {
				blockMove = true;
			}

			// if pan or tilt has changed, move ptz
			if (((spectraObj.panCenterNorm != _lastSpectraPanNorm) || (spectraObj.tiltCenterNorm != _lastSpectraTiltNorm)) && !blockMove) {
				_ptzControl.setAbsoluteLocation(spectraObj.panCenterNorm, spectraObj.tiltCenterNorm, &zoom);
				changePosition = true;
			}
			// if just zoom changed, change ptz zoom
			else if (zoom != _lastZoom) {
				_ptzControl.setAbsoluteZoom(zoom);
				changeZoom = true;
			}
			opteraObj.getDimensions(opteraObjHeightRad, opteraObjWidthRad);
			spectraObj.getDimensions(spectraObjHeightNorm, spectraObjWidthNorm);
		}
		else {
			if (_zoomTimer.timeToZoomOut(zoom)) {
				if (zoom != _lastZoom) {
					_ptzControl.setAbsoluteZoom(zoom);
					changeZoom = true;
					zoomState = 9;
				}
			}
		}
		if (changePosition || changeZoom || blockMove) {  // print debug even when re-center not performed
			CString str;
			float panTiltSpeed = _ptzControl.getLastPanTiltSpeed();
			float deltaTiltNorm = abs(spectraObj.tiltCenterNorm - _lastSpectraTiltNorm);
			float deltaPanNorm = abs(spectraObj.panCenterNorm - _lastSpectraPanNorm);
			float deltaNorm = sqrt((deltaTiltNorm * deltaTiltNorm) + (deltaPanNorm * deltaPanNorm));
			uint64_t ctime = getCurrentTimeMs();
			static uint64_t last_time = ctime;
			uint32_t delta_time = (uint32_t)(ctime - last_time);
			last_time = ctime;
			float opteraPanCenterRad, opteraTiltCenterRad;
			opteraObj.getCenter(opteraPanCenterRad, opteraTiltCenterRad);
			str.Format(_T("MOVE: OPTERA: objId: %3d panRad: %7.5f tiltRad: %7.5f wdthRad: %7.5f hghtRad: %7.5f speed: %8.4f ave_speed: %8.4f SPECTRA: panNorm: %7.5f tiltNorm: %8.5f wdthNorm: %7.5f hghtNorm: %7.5f deltaNorm: %7.5f panFovNorm: %7.5f tiltFovNorm: %7.5f speed: %7.5f rawZoom: %7.5f zoom: %7.5f zoomState: %d deltaTime: %4d ms blocked: %d\n"),
				opteraObj.id, opteraPanCenterRad, opteraTiltCenterRad, opteraObjWidthRad, opteraObjHeightRad, speed, aveSpeed, spectraObj.panCenterNorm, spectraObj.tiltCenterNorm, spectraObjWidthNorm, spectraObjHeightNorm, deltaNorm, spectraObj.tiltFovNorm, spectraObj.panFovNorm, panTiltSpeed, rawZoom, zoom, zoomState, delta_time, blockMove);
			MOVE_DOUT((LPCTSTR)str);
		}
		if (changePosition && !blockMove) {
			_lastSpectraTiltNorm = spectraObj.tiltCenterNorm;
			_lastSpectraPanNorm = spectraObj.panCenterNorm;
		}
		if (changeZoom) {
			_lastZoom = zoom;
		}
    }
}

//////////////////////////////////////////////////////////////////////////////////
void SituationTracking::followObject(float x, float y)
{
    unique_lock<mutex> lk(_objectFilter._followObject._mutex);
    _objectFilter._followObject._x = x;
    _objectFilter._followObject._y = y;
    _objectFilter._followObject._time = getCurrentTimeMs();
}

//////////////////////////////////////////////////////////////////////////////////
void SituationTracking::setPtzIpAddress(const std::string& ip)
{
    _ptzControl.setIpAddress(ip);
    _cameraInfos.setPtzCameraSpec(getSpectraModelNumber(ip));
}