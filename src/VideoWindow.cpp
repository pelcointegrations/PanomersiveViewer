//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include <Wingdi.h>
#include "VideoWindow.hpp"
#include "wgl.hpp"
#include "ErrorHandling.hpp"
#define _USE_MATH_DEFINES
#include <math.h>
#include <cassert>
#include <sstream>

#include "soapPTZBindingProxy.h"
#include "MosaicToPanTilt.hpp"
#include "SituationControl.hpp"

#include <iomanip>

//#define ENABLE_DEMO_TRACE
#ifdef ENABLE_DEMO_TRACE
#  ifdef _MSC_VER
#    include <windows.h>
#    include <sstream>
#    define DEMOTRACE(x)                           \
     do {  std::wostringstream s;  s << x;      \
           OutputDebugStringW(s.str().c_str()); \
                                      } while(0);
#  else
#    include <iostream>
#    define DEMOTRACE(x)  std::cerr << x << std::flush
#  endif
#else
#  define DEMOTRACE(x)
#endif

using namespace std;

const string ptzServiceEndpointSuffix = "/onvif/ptz_service";

#define WM_USER_REDRAW   (WM_USER + 1)

static const int kMarkerSize = 11;
static const int kLineSize = 3;

VideoWindow::Offscreen VideoWindow::_noMarker = {0};
VideoWindow::Offscreen VideoWindow::_redMarker = {0};
VideoWindow::Offscreen VideoWindow::_greenMarker = {0};
VideoWindow::Offscreen VideoWindow::_blueMarker = {0};
VideoWindow::Offscreen VideoWindow::_darkRedMarker = {0};
VideoWindow::Offscreen VideoWindow::_darkBlueMarker = {0};
VideoWindow::OffscreenRect VideoWindow::_zoomMarker;
float VideoWindow::_rawPtzPanNorm = 0;
float VideoWindow::_rawPtzTiltNorm = 0;
float VideoWindow::_rawPtzZoomNorm = 0;

int VideoWindow::_instanceCount = 0;
SituationControl::CalibrationState _lastState;

//////////////////////////////////////////////////////////////////////////////////
VideoWindow::VideoWindow(PelcoImextk_ViewType vt,
	HGLRC streamGlContext,
	PelcoImextk_Context imextkContext,
	HINSTANCE hInstance, HWND hParentWnd, const RECT& area,
	bool separateWindow,
	const wstring& windowTitle,
	PositionTrackingCallback cb,
	UserOptions::RenderMode renderMode,
	shared_ptr<PerformanceStats> stats, struct soap* psoap,
	shared_ptr<SituationControl> sitControl)
	: Window(defaultWindowClassInfo(hInstance),
	separateWindow ? NULL : hParentWnd,
	separateWindow ? WS_POPUP | WS_BORDER : WS_CHILD | WS_BORDER,
	area, windowTitle, NULL),
	_viewType(vt),
	_hParent(hParentWnd),
	_imextkContext(imextkContext),
	_viewGenerator(PelcoImextk_kInvalidViewGen),
	_hGLRC(NULL),
	_panTilting(false),
	_zooming(false),
	_isTrackingPosition(false),
	_isControllingPositionTracking(false),
	_trackedPanAngleRadians(0),
	_trackedTiltAngleRadians(0),
	_trackedZoomFactor(0),
	_isShowingDataBounds(false),
	_moveMouseAfterRotateTo(false),
	_frameCountOverlayMode(false),
	_bitrateOverlayMode(false),
	_renderMode(renderMode),
	_offscreen({ 0 }),
	_positionTrackingTextImg({ 0 }),
	_calibrationTextImg({ 0 }),
	_statsTextImg({ 0 }),
	_bitrateGraphImg({ 0 }),
	_glRenderDC(NULL),
	_positionTracker(cb),
	_stats(stats),
	_statsViewId(_stats->addView()),
	_sitControl(sitControl),
	_isShowingCalLine(false),
	_calPanAngleRadians(0),
	_calTiltAngleRadians(0),
	_drawZoom(false),
	_lastCalMark({ 0, 0 })
{
    _pSoap = psoap;

    ++_instanceCount;

    _hZTimer = NULL;

    _hZTimer = CreateWaitableTimer(NULL, TRUE, L"Zoom Stop Timeout");
    if (NULL == _hZTimer)
    {
        printf("CreateWaitableTimer failed (%d)\n", GetLastError());
    }
    _liDueTime.QuadPart = -2500000LL; //0.25 sec
    _liDueTimeForClearTimer.QuadPart = -100000000000LL;

    if (_instanceCount == 1) {
        _noMarker._hDC = CreateCompatibleDC(_hDC);
        resizeOffscreenStorage(_noMarker, kMarkerSize, kMarkerSize, false);
        memset(_noMarker._bmpBits, 0, _noMarker._width * _noMarker._height * 4);
        _redMarker._hDC = CreateCompatibleDC(_hDC);
        drawMarker(_redMarker, kMarkerSize, kMarkerSize, RGB(255, 0, 0));
        _greenMarker._hDC = CreateCompatibleDC(_hDC);
        drawMarker(_greenMarker, kMarkerSize, kMarkerSize, RGB(0, 255, 0));
        _blueMarker._hDC = CreateCompatibleDC(_hDC);
        drawMarker(_blueMarker, kMarkerSize, kMarkerSize, RGB(0, 0, 255));
        _darkRedMarker._hDC = CreateCompatibleDC(_hDC);
        drawMarker(_darkRedMarker, kMarkerSize, kMarkerSize, RGB(64, 0, 0));
        _darkBlueMarker._hDC = CreateCompatibleDC(_hDC);
        drawMarker(_darkBlueMarker, kMarkerSize, kMarkerSize, RGB(0, 0, 64));
        _zoomMarker._hDC = CreateCompatibleDC(_hDC);
        _zoomMarker._drawing = false;
	}
	_positionTrackingTextImg._hDC = CreateCompatibleDC(_hDC);
	_calibrationTextImg._hDC = CreateCompatibleDC(_hDC);
    _statsTextImg._hDC = CreateCompatibleDC(_hDC);
    _bitrateGraphImg._hDC = CreateCompatibleDC(_hDC);
    _noCalLine._hDC = CreateCompatibleDC(_hDC);
    resizeOffscreenStorage(_noCalLine, kLineSize, area.bottom - area.top, false);
    memset(_noCalLine._bmpBits, 0, _noCalLine._width * _noCalLine._height * 4);
    _blueCalLine._hDC = CreateCompatibleDC(_hDC);
    drawMarker(_blueCalLine, kLineSize, area.bottom - area.top, RGB(0, 0, 255));

    if (_renderMode == UserOptions::RM_FRAMEBUFFER) {
        // rendering to an offscreen buffer requires a fair amount of additional
        // opengl work.  First we'll create a separate, hidden window to provide
        // the context for hardware-acclerated opengl rendering.
        // Later, when the window is sized, we'll create an opengl framebuffer object,
        // into which ViewGenerator will render its output.
        _offscreen._hWnd = CreateWindow(_className.c_str(), L"",
                WS_POPUP,
                0, 0, area.right - area.left, area.bottom - area.top,
                _hWnd, NULL, _hInstance, NULL);
        ShowWindow(_offscreen._hWnd, SW_HIDE);
        _glRenderDC = GetDC(_offscreen._hWnd);
    }
    else {
        _glRenderDC = _hDC;
    }

    if ((_renderMode == UserOptions::RM_RAW) ||
            (_renderMode == UserOptions::RM_FRAMEBUFFER)) {
        // for both RM_RAW and RM_FRAMEBUFFER, we need a device independent
        // bitmap, for which we'll need a separate, compatible device context.
        _offscreen._hDC = CreateCompatibleDC(_hDC);
    }

    _hGLRC = createGlContext(_glRenderDC, streamGlContext);
    try {
        OpenGlGuard grd(_glRenderDC, _hGLRC);
        IMEXTK_CHECK(pelco_imextk_gl_create_view_generator(_imextkContext,
                        vt, &_viewGenerator));
    }
    catch (...) {
        cleanup();
        throw;
    }

    // Use charcoal grey to offset the boundary of any views that
    // feature "letterboxing" (e.g., the mercator view)
    setLetterboxingColor(50, 50, 50);
    if (_sitControl.get()) {
        _sitControl->setViewGenerator(_viewGenerator, _imextkContext);
    }

    // Make new font for bit rate overlay
    memset(&_logFont, 0, sizeof(_logFont));
    _logFont.lfHeight = -26; // see PS
    _logFont.lfWeight = FW_NORMAL;
    // This allows you to find which pixels changed for 
    // the font.  Anti-aliasing changes pixels around it which makes
    // it hard to find.  
    _logFont.lfQuality = NONANTIALIASED_QUALITY;
    const wchar_t fontName[] = L"Arial";
    wcsncpy_s(_logFont.lfFaceName, fontName, sizeof(fontName));
    _hfont = CreateFontIndirect(&_logFont);

}

//////////////////////////////////////////////////////////////////////////////////
VideoWindow::~VideoWindow()
{
    _stats->removeView(_statsViewId);
    cleanup();
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::cleanup()
{
    if (onvifPTZable()) {
        doOnvifPanTiltCont(0.0f, 0.0f);
        doOnvifZoom(0.0f);
    }
    --_instanceCount;
    if (_instanceCount == 0) {
        cleanupOffscreenStorage(_noMarker);
        cleanupOffscreenStorage(_redMarker);
        cleanupOffscreenStorage(_greenMarker);
        cleanupOffscreenStorage(_blueMarker);
        cleanupOffscreenStorage(_darkRedMarker);
        cleanupOffscreenStorage(_blueCalLine);
        cleanupOffscreenStorage(_noCalLine);
	}
	cleanupOffscreenStorage(_positionTrackingTextImg);
	cleanupOffscreenStorage(_calibrationTextImg);
    cleanupOffscreenStorage(_statsTextImg);
    cleanupOffscreenStorage(_bitrateGraphImg);

    OpenGlGuard grd(_glRenderDC, _hGLRC);
    if (_viewGenerator != PelcoImextk_kInvalidViewGen) {
        pelco_imextk_gl_delete_view_generator(_viewGenerator);
        _viewGenerator = PelcoImextk_kInvalidViewGen;
    }

    cleanupOffscreenStorage(_offscreen);

    wglDeleteContext(_hGLRC);
    _hGLRC = NULL;

    if (_offscreen._hDC != NULL) {
        DeleteObject(_offscreen._hDC);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::cleanupOffscreenStorage(Offscreen& img)
{
    if (img._bmp != NULL) {
        DeleteObject(img._bmp);
        img._bmp = NULL;
        img._bmpBits = NULL;
    }
    img._width = 0;
    img._height = 0;
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::move(const RECT& r)
{
    // if we're not a child window, go top-most to allow full-screen view.
    if (GetParent(_hWnd) == NULL) {
        SetWindowPos(_hWnd, HWND_TOP, r.left, r.top,
                r.right - r.left, r.bottom - r.top, NULL);
    }
    else {
        Window::move(r);
    }
}

//////////////////////////////////////////////////////////////////////////////////
UserOptions::PtzLimitOptions VideoWindow::getPtzLimitOptions() const
{
    UserOptions::PtzLimitOptions opts;
    IMEXTK_CHECK(pelco_imextk_get_ptz_limit_options(_viewGenerator,
                    &opts._mode, &opts._autoZoomInWithPanTilt,
                    &opts._autoZoomOutWithPanTilt,
                    &opts._autoPanTiltWithZoomOut));
    return opts;
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::setPtzLimitOptions(const UserOptions::PtzLimitOptions& opts)
{

    IMEXTK_CHECK(pelco_imextk_set_ptz_limit_options(_viewGenerator,
                    opts._mode, opts._autoZoomInWithPanTilt,
                    opts._autoZoomOutWithPanTilt,
                    opts._autoPanTiltWithZoomOut));
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::setFrameCountOverlayMode(bool overlayOn)
{
    if (_frameCountOverlayMode && !overlayOn) {
        drawPerformanceStats(true);
        // Since performance stats are on stop and 
        //  we want to erase the bitrate stats below it.
        drawBitrateStats(true);
    }
    _frameCountOverlayMode = overlayOn;
    drawNewView();
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::setBitrateOverlayMode(bool overlayOn)
{
    if (_bitrateOverlayMode && !overlayOn) {
        drawBitrateStats(true);
    }
    _bitrateOverlayMode = overlayOn;
    drawNewView();

}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::addBitmapToOverlay(unsigned int xoffset, unsigned int yoffset, unsigned int pixelWidth,
    unsigned int pixelHeight, const unsigned char * bmpData)
{
    if (_viewGenerator != PelcoImextk_kInvalidViewGen) {
        OpenGlGuard grd(_glRenderDC, _hGLRC);
        PelcoImextk_Result res = (pelco_imextk_gl_add_bitmap_to_overlay(_viewGenerator, xoffset, yoffset, pixelWidth, pixelHeight, bmpData));
        if ((res != PELCO_IMEXTK_NO_ERROR) && (res != PELCO_IMEXTK_ERR_OUT_OF_BOUNDS_ARGS)) {
            throwImextkError("Error on call pelco_imextk_gl_add_bitmap_to_overlay", res, _imextkContext);
        }
    }
}

void VideoWindow::setLetterboxingColor(unsigned char red, unsigned char green, unsigned char blue)
{
    if (_viewGenerator != PelcoImextk_kInvalidViewGen) {
        PelcoImextk_Result res = (pelco_imextk_set_letterboxing_color(_viewGenerator, red, green, blue));
        if (res != PELCO_IMEXTK_NO_ERROR) {
            throwImextkError("Error on call pelco_imextk_set_letterboxing_color", res, _imextkContext);
        }
    }
}

void VideoWindow::getLetterboxingColor(unsigned char& red, unsigned char& green, unsigned char &blue) {
    if (_viewGenerator != PelcoImextk_kInvalidViewGen) {
        PelcoImextk_Result res = (pelco_imextk_get_letterboxing_color(_viewGenerator, red, green, blue));
        if (res != PELCO_IMEXTK_NO_ERROR) {
            throwImextkError("Error on call pelco_imextk_get_letterboxing_color", res, _imextkContext);
        }
    }
}

void VideoWindow::setPTZEndpoint(string ptzEndpoint, string ptzToken)
{
    _ptzEndpoint = ptzEndpoint;
    _ptzToken = ptzToken;
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnResize(int width, int height)
{
    if (_offscreen._hWnd != NULL) {
        MoveWindow(_offscreen._hWnd, 0, 0, width, height, false);
    }
    if (_renderMode != UserOptions::RM_DIRECT) {
        resizeOffscreenStorage(_offscreen, width, height, _offscreen._isYUV);
    }
    if (_viewGenerator != PelcoImextk_kInvalidViewGen) {
        OpenGlGuard grd(_glRenderDC, _hGLRC);
        IMEXTK_CHECK(pelco_imextk_gl_resize_view(_viewGenerator,
                        max(width, 1), max(height, 1)));
    }
    resizeOffscreenStorage(_noCalLine, kLineSize, height, false);
    memset(_noCalLine._bmpBits, 0, _noCalLine._width * _noCalLine._height * 4);
    drawMarker(_blueCalLine, kLineSize, height, RGB(0, 0, 255));
    PostMessage(_hWnd, WM_USER_REDRAW, NULL, NULL);
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnLMouseDown(const POINTS& pt)
{
    // if we're not already capturing the mouse for some other operation...
    if (GetCapture() != _hWnd) {
        _ptzAnchor = pt;
        _panTilting = true;
        SetCapture(_hWnd);

        if (isNonOpteraCam() == true) {
			UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
			if (onvifPTZable() && (opts._mode == PELCO_IMEXTK_ONVIF_PTZ_SELECTED || opts._mode == PELCO_IMEXTK_PTZ_LIMIT_NONE)) {
				// draw center red marker
				int width, height;
				IMEXTK_CHECK(pelco_imextk_get_view_size(_viewGenerator, &width, &height));
				_calMark.x = width / 2;	_calMark.y = height / 2;
				drawCalibrationMark();
			}
			// calc normalized pan/tilt delta 
			std::pair<float, float> pantilt = pointToPanTiltPair(pt);
			pantilt.first /= 4.0f; pantilt.second /= 4.0f;
			// do relative ptz move (remember abs cmd'd pan/tilt)
			doOnvifPanTiltRel(pantilt.first, pantilt.second, 0.0, 1.0);
        }
        else {
            if (_sitControl.get()) {
                clearBoxForZoom();
                if (_sitControl->getControlMode() == SituationControl::SA_ZOOM_TO_BOX) {
                    _drawZoom = true;
                    _zoomMarker._xPos = _zoomMarker._xAnchor = (int)pt.x;
                    _zoomMarker._yPos = _zoomMarker._yAnchor = (int)pt.y;
                    _zoomMarker._clientWidth = (int) clientWidth();
                    _zoomMarker._clientHeight = (int) clientHeight();
                }
                else {
                    _drawZoom = false;
				}
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnLMouseUp(const POINTS& pt)
{
    if (_panTilting) {
        ReleaseCapture();
        _panTilting = false;

        if (isNonOpteraCam() == true) {
            UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
			if (onvifPTZable() && (opts._mode == PELCO_IMEXTK_ONVIF_PTZ_SELECTED || opts._mode == PELCO_IMEXTK_PTZ_LIMIT_NONE)) {   // single PTZ or sitAware PTZ cases
				// remove center red marker
				clearCalibrationMark();
			}
        }
        else if (_sitControl.get()) {
            _drawZoom = false;
            POINT mt;
            GetCursorPos(&mt);
            DEMOTRACE("screen point: Cursor Position: " << mt.x << "x" << mt.y << " Mouse Point " << pt.x << "x" << pt.y << endl);
            ScreenToClient(_hWnd, &mt);
            DEMOTRACE("client point " << mt.x << "x" << mt.y << " size  " << clientWidth() << "x" << clientHeight() << endl);

			// handle calibration point (and other SA points)
			_sitControl->onLMouseButtonUp(mt.x, mt.y, _zoomMarker._xAnchor, _zoomMarker._yAnchor, _rawPtzPanNorm, _rawPtzTiltNorm);

			// update calibration state and message
			if ((_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_OLD) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS)) {
				// draw new read mark
				_calMark = mt;
				drawCalibrationMark();
				// advance calibration state
				switch (_sitControl->getControlMode()) {
				case SituationControl::SA_CALIBRATE_PTZ_PAN_OLD:
					_lastState = SituationControl::CAL_PAN_STATE;
					break;
				case SituationControl::SA_CALIBRATE_PTZ_PAN:
					_lastState = SituationControl::CAL_PAN_STATE;
					break;
				case SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION:
					_lastState = SituationControl::CAL_POINT_1_STATE;
					break;
				case SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND:
					_lastState = SituationControl::CAL_POINT_2_STATE;
					break;
				case SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS:
				case SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS:
					_lastState = SituationControl::CAL_POINT_4_STATE;
					break;
				}
				SituationControl::CalibrationState state = _sitControl->getCalibrationState();
				if (state < _lastState) {
					state = (SituationControl::CalibrationState)((int)state + 1);
				}
				else {
					state = SituationControl::CAL_DONE_STATE;
				}
				_sitControl->setCalibrationState(state);
				// display new calibration message
				drawCalibrationText(false);
			}
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnLMouseDblClk(const POINTS& pt)
{
    if (_viewType == PELCO_IMEXTK_IMMERSIVE_VIEW) {
        // for immersive view, we'll just locally rotate to the mouse position
        centerOnMouse();
    }
    else {
        // for situational-awareness view, double-click will swarm all views to the
        // indicated position
        requestSwarmToMouse();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnMMouseDown(const POINTS& pt)
{
    // if we're not already capturing the mouse for some other operation...
    if (GetCapture() != _hWnd) {
        _ptzAnchor = pt;
        _zooming = true;
        SetCapture(_hWnd);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnMMouseUp(const POINTS& /*pt*/)
{
    if (_zooming) {
        _zooming = false;
        ReleaseCapture();
        if (isNonOpteraCam() == true) {
            UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
            if (onvifPTZable() && (opts._mode == PELCO_IMEXTK_ONVIF_PTZ_SELECTED)) {
                doOnvifZoom(0.0f);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnRMouseDown(const POINTS& pt)
{
    // if we're not already capturing the mouse for some other operation...
    if (GetCapture() != _hWnd) {
        _isControllingPositionTracking = true;
        emitPositionTrackingEvent(pt, TRACK);
        SetCapture(_hWnd);
        PostRedraw();

		if (_sitControl.get()) {
			// update calibration state and message
			if ((_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_OLD) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS) ||
				(_sitControl->getControlMode() == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS)) {

				// decrement calibration state
				SituationControl::CalibrationState state = _sitControl->getCalibrationState();
				if (state > 0) {
					if (state != SituationControl::CAL_DONE_STATE) {
						state = (SituationControl::CalibrationState)(state - 1);
					}
					else {
						state = _lastState;
					}
				}
				_sitControl->setCalibrationState(state);
				// display new calibration message
				drawCalibrationText(false);
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnRMouseUp(const POINTS& pt)
{
    if (_isControllingPositionTracking) {
        _isControllingPositionTracking = false;
        emitPositionTrackingEvent(pt, END_TRACKING);
        ReleaseCapture();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnMouseMove(const POINTS& pt)
{
    // take focus if we don't already have it.
    SetFocus(_hWnd);

    if (_viewGenerator == PelcoImextk_kInvalidViewGen) {
        return;
    }
    if (_drawZoom) {
        drawBoxForZoom(pt);
    }
    else if (_panTilting) {
        try {
            if (isNonOpteraCam() == true) {
                UserOptions::PtzLimitOptions opts = getPtzLimitOptions();

                if (onvifPTZable() && (opts._mode == PELCO_IMEXTK_ONVIF_PTZ_SELECTED)) {

                    std::pair<float, float> pantilt = pointToPanTiltPair(pt);

                    int result = doOnvifPanTiltCont(pantilt.first, pantilt.second);
                }
            } else {
                float pan;
                float tilt;
                IMEXTK_CHECK(pelco_imextk_get_view_angle(_viewGenerator, &pan, &tilt));

                float anchorPan;
                float anchorTilt;
                IMEXTK_CHECK(pelco_imextk_view_to_spherical(_viewGenerator,
                    _ptzAnchor.x, _ptzAnchor.y, &anchorPan, &anchorTilt));
                float mousePan;
                float mouseTilt;
                IMEXTK_CHECK(pelco_imextk_view_to_spherical(_viewGenerator,
                    pt.x, pt.y, &mousePan, &mouseTilt));

                // reverse sense to move the view instead of the data.
                rotateTo(pan - (mousePan - anchorPan),
                    tilt - (mouseTilt - anchorTilt), false);
            }
        }
        catch (...) {
            // ignore errors here -- could be caused by the mouse moving too far
            // outside the view area.
        }
    }
    else if (_zooming) {
        float deltaZ = -float(pt.y - _ptzAnchor.y) / float(clientHeight());
        UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
        if (onvifPTZable() && (opts._mode == PELCO_IMEXTK_ONVIF_PTZ_SELECTED)) {
            resetAndCancelTimer(_hZTimer);
            doOnvifZoom(deltaZ);
        }
        else {
            //UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
            //optera cameras can zoom, or non-optera cameras that aren't being limited by ptz option
            if ((isNonOpteraCam() == false) || (opts._mode == PELCO_IMEXTK_PTZ_LIMIT_NONE))
            {
                float zoom;
                IMEXTK_CHECK(pelco_imextk_get_zoom(_viewGenerator, &zoom));
                zoom += deltaZ;
                zoom = max(min(zoom, 1.0f), 0.0f);
                IMEXTK_CHECK(pelco_imextk_set_zoom(_viewGenerator, zoom));
                drawNewView();
            }
        }
    }
    else if (_isControllingPositionTracking) {
        emitPositionTrackingEvent(pt, TRACK);
        PostRedraw();
    }
    _ptzAnchor = pt;
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnMouseWheel(short wheelValue, const POINTS& screenPt)
{
    if ((_viewGenerator == PelcoImextk_kInvalidViewGen) ||
            (_viewType != PELCO_IMEXTK_IMMERSIVE_VIEW)) {
        return;
    }

    POINT pt = {screenPt.x, screenPt.y};
    ScreenToClient(_hWnd, &pt);

    const float zoomSpeed = 0.02f; // imperically determined value.
    float deltaZ = float(wheelValue) / float(WHEEL_DELTA) * zoomSpeed;

    if (isNonOpteraCam() == true) {
        UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
        if (onvifPTZable() && (opts._mode == PELCO_IMEXTK_ONVIF_PTZ_SELECTED)) {
            resetAndCancelTimer(_hZTimer);
            int result = doOnvifZoom(deltaZ);
            SetWaitableTimer(_hZTimer, &_liDueTime, 0, NULL, NULL, 0);
        }
        else {
            UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
            if (opts._mode == PELCO_IMEXTK_PTZ_LIMIT_NONE) {
                //allow zoom for non optera cameras if limiting is off
				int width;
				int height;
				IMEXTK_CHECK(pelco_imextk_get_view_size(_viewGenerator, &width, &height));
				POINTS point;
				point.x = width/2;
				point.y = height/2;
                doDigitalZoomPT(point, deltaZ);
                drawNewView();
            }
        }
        return;
    }

    // We want to zoom while keeping pt at the same position within the view.
    //
    // To do that, we map pt into view absolute coordinates
    // before and after zooming.  (All points except the center
    // of the view will move as a result of zooming.)
    // Then we pan/tilt the view by the amount that our target
    // point moved, in order to bring that point back to where it was.
    //
    float pan1, tilt1;
    if (pelco_imextk_view_to_spherical(_viewGenerator,
                    pt.x, pt.y, &pan1, &tilt1) == PELCO_IMEXTK_NO_ERROR) {

        float zoom;
        IMEXTK_CHECK(pelco_imextk_get_zoom(_viewGenerator, &zoom));
        zoom += deltaZ;
        zoom = min(max(zoom, 0.0f), 1.0f);
        IMEXTK_CHECK(pelco_imextk_set_zoom(_viewGenerator, zoom));

        float pan2, tilt2;
        if (pelco_imextk_view_to_spherical(_viewGenerator,
                        pt.x, pt.y, &pan2, &tilt2) == PELCO_IMEXTK_NO_ERROR) {
            float pan, tilt;
            IMEXTK_CHECK(pelco_imextk_get_view_angle(_viewGenerator, &pan, &tilt));
            pan -= pan2 - pan1;
            tilt -= tilt2 - tilt1;

            // override PTZ limiting options during this operation - we don't want any
            // zooming side-effects here.
            UserOptions::PtzLimitOptions oldOpts = getPtzLimitOptions();
            UserOptions::PtzLimitOptions override = oldOpts;
            override._autoZoomInWithPanTilt = false;
            override._autoZoomOutWithPanTilt = false;
            setPtzLimitOptions(override);
            try {
                IMEXTK_CHECK(pelco_imextk_set_view_angle(_viewGenerator, pan, tilt));
            }
            catch (...) {
                setPtzLimitOptions(oldOpts);
                throw;
            }
            setPtzLimitOptions(oldOpts);
        }
        drawNewView();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnKeyUp(WPARAM virtualKeyCode, LPARAM repeatFlags)
{
    if (virtualKeyCode == 0x42) { // 'B' key
        clearDataBoundsMarkers();
        _isShowingDataBounds = !_isShowingDataBounds;

        PostRedraw();
    }
    else if (virtualKeyCode == 0x43) { // 'C' key
        rotateToDataCenter();
    }
    else if (virtualKeyCode == 0x53) { // 'S' key
        // 'swarm' to the current mouse position
        if (_isTrackingPosition) {
            requestSwarmToMouse();
        }
    }
    else if (virtualKeyCode == VK_ESCAPE) {
        // bubble the command to the parent window
        SendMessage(_hParent, WM_KEYUP, virtualKeyCode, repeatFlags);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::requestSwarmToMouse()
{
    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(_hWnd, &pt);
    if ((pt.x >= 0) && (size_t(pt.x) < clientWidth()) &&
            (pt.y >= 0) && (size_t(pt.y) < clientHeight())) {
        POINTS pts = {SHORT(pt.x), SHORT(pt.y)};
        _moveMouseAfterRotateTo = true;
        emitPositionTrackingEvent(pts, POINT_OF_INTEREST);
        _moveMouseAfterRotateTo = false;
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::rotateToDataCenter()
{
    float centerPan;
    float centerTilt;
    float width;
    float height;
    if (pelco_imextk_get_data_bounds(_imextkContext,
                    &centerPan, &centerTilt,
                    &width, &height) ==
            PELCO_IMEXTK_NO_ERROR) {
        rotateTo(centerPan, centerTilt, false);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::centerOnMouse()
{
    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(_hWnd, &pt);
    if ((pt.x >= 0) && (size_t(pt.x) < clientWidth()) &&
            (pt.y >= 0) && (size_t(pt.y) < clientHeight())) {
        try {
            if (isNonOpteraCam() == true) {
                //skip it for now....
                float deltaX;
                float deltaY;
                IMEXTK_CHECK(pelco_imextk_get_view_angle(_viewGenerator, &deltaX, &deltaY));
                deltaX = -deltaX;
                deltaY = -deltaY;
                IMEXTK_CHECK(pelco_imextk_set_view_angle(_viewGenerator, deltaX, deltaY));
                IMEXTK_CHECK(pelco_imextk_set_zoom(_viewGenerator, 0.0f));
                drawNewView();
            }
            else {
                float mousePan;
                float mouseTilt;
                IMEXTK_CHECK(pelco_imextk_view_to_spherical(_viewGenerator,
                    pt.x, pt.y, &mousePan, &mouseTilt));
                _moveMouseAfterRotateTo = true;
                rotateTo(mousePan, mouseTilt, true);
                _moveMouseAfterRotateTo = false;
            }
        }
        catch (...) {
            // ignore errors here -- could be caused by the mouse moving too far
            // outside the view area.
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::rotateTo(float panRadians, float tiltRadians, bool autoZoom)
{
    //[SPS-5520] fix, checks if _isTrackingPosition == true
    // clear old track marker if active tracking while rotating
    clearTrackedPositionMarker();
    //[SPS-5519] fix, checks if we rotate while actively displaying bounds _isShowingDataBounds = true
    clearDataBoundsMarkers();
    // clear calibration line
    clearCalibrationLine();
    // temporarily set data limiting mode to auto-zoom, so that we can
    // always successfully pan to the requested position
    UserOptions::PtzLimitOptions oldOpts;
    if (autoZoom) {
        oldOpts = getPtzLimitOptions();
        UserOptions::PtzLimitOptions override = oldOpts;
        override._autoZoomInWithPanTilt = true;
        setPtzLimitOptions(override);
    }

    IMEXTK_CHECK(pelco_imextk_set_view_angle(_viewGenerator, panRadians, tiltRadians));

    if (autoZoom) {
        setPtzLimitOptions(oldOpts);
    }

    drawNewView();

    if (_moveMouseAfterRotateTo) {
        int x;
        int y;
        if (pelco_imextk_spherical_to_view(_viewGenerator,
                        panRadians, tiltRadians,
                        &x, &y) == PELCO_IMEXTK_NO_ERROR) {
            POINT pt = {x, y};
            ClientToScreen(_hWnd, &pt);
            SetCursorPos(pt.x, pt.y);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::emitPositionTrackingEvent(const POINTS& pt, TrackingEvent evt)
{
    float panRadians;
    float tiltRadians;
    float zoom;
    if ((pelco_imextk_view_to_spherical(_viewGenerator,
                    pt.x, pt.y, &panRadians, &tiltRadians)
                    != PELCO_IMEXTK_NO_ERROR) ||
            (pelco_imextk_get_zoom(_viewGenerator, &zoom)
                    != PELCO_IMEXTK_NO_ERROR)) {
        // point is probably outside the viewing area -- ignore
        return;
    }

    if (_positionTracker) {
        _positionTracker(evt, panRadians, tiltRadians, zoom);
    }
}

void VideoWindow::transPositionToMosaic(const POINTS& pt, POINTS& transPT)
{
    int x;
    int y;
    x = transPT.x;
    y = transPT.y;
    pelco_imextk_gl_view_to_mosaic_pos(_viewGenerator, pt.x, pt.y, &x, &y);
    transPT.x = x;
    transPT.y = y;
}

void VideoWindow::transSphericalToMosaic(float panRadians, float tiltRadians, int& xPos, int& yPos)
{
    pelco_imextk_gl_spherical_to_mosaic_pos(_viewGenerator, panRadians, tiltRadians, &xPos, &yPos);
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::trackPosition(bool isTracking, float panAngleRadians,
        float tiltAngleRadians)
{
    int x, y;

    clearTrackedPositionMarker(); //checks if _isTrackingPosition == true

    _isTrackingPosition = isTracking;
    _trackedPanAngleRadians = panAngleRadians;
    _trackedTiltAngleRadians = tiltAngleRadians;
    pelco_imextk_get_zoom(_viewGenerator, &_trackedZoomFactor);

    if (_isTrackingPosition) {
        // draw the new marker

        //update mosaic position
        transSphericalToMosaic(panAngleRadians, tiltAngleRadians, _trackedRawMosaicPosX, _trackedRawMosaicPosY);

        if (pelco_imextk_spherical_to_view(_viewGenerator,
                        _trackedPanAngleRadians, _trackedTiltAngleRadians,
                        &x, &y) == PELCO_IMEXTK_NO_ERROR) {
            drawPositionTrackingText(false);
            unsigned int xPos = unsigned int(max(0, x - int(_redMarker._width/2)));
            unsigned int yPos = unsigned int(max(0, y - int(_redMarker._height/2)));
            addBitmapToOverlay(xPos, yPos,
                    _redMarker._width, _redMarker._height, _redMarker._bmpBits);
        }
    }
    else {
		drawPositionTrackingText(true);
    }
    drawNewView();
}

//////////////////////////////////////////////////////////////////////////////////
bool VideoWindow::EraseBackground()
{
    // "erase" to the imextk-controlled contents
    drawNewView();
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::OnPaint(const PAINTSTRUCT& ps)
{
    if (_viewGenerator == PelcoImextk_kInvalidViewGen) {
        FillRect(_hDC, &ps.rcPaint, (HBRUSH)GetStockObject(DKGRAY_BRUSH));
        RECT r = {0, 0, int(clientWidth()), int(clientHeight())};
        DrawText(_hDC, L"View Initialization Error", -1, &r,
                DT_CENTER | DT_SINGLELINE | DT_VCENTER);
    }
    else {
        // if we're using an offscreen bitmap (i.e., we're either in framebuffer or raw mode),
        // copy it to this window now
        if (_offscreen._bmp != NULL) {
            int w = int(min(clientWidth(), _offscreen._width));
            int h = int(min(clientHeight(), _offscreen._height));
            BitBlt(_hDC, 0, 0, w, h,
                    _offscreen._hDC, 0, 0, SRCCOPY);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawPerformanceStats(bool undraw)
{

    unsigned int labelWidth = 700;
    unsigned int labelHeight = _frameCountOverlayHeight;

    resizeOffscreenStorage(_statsTextImg, labelWidth, labelHeight, false);
    HDC hDC = _statsTextImg._hDC;

    if (undraw) {
        memset(_statsTextImg._bmpBits, 0,
                _statsTextImg._width * _statsTextImg._height * 4);
    }
    else {
        SelectObject(hDC, GetStockObject(SYSTEM_FONT));
        SetTextColor(hDC, 0x00010101);
        SetBkColor(hDC, 0x00FFFFFF);
        SetBkMode(hDC, OPAQUE);

        size_t numPix = labelWidth * labelHeight;
        memset(_statsTextImg._bmpBits, 0, numPix);

        wstringstream msg;

        PerformanceStats::StreamStats strmStats;
        for (int i = 0; i < 6; ++i) {
            _stats->getStreamStats(i, strmStats);
            size_t totalFrames = strmStats._renderedFrames + strmStats._droppedFrames;
            float pct = 100.0f;
            if (totalFrames > 0) {
                pct = float(strmStats._renderedFrames) * 100.0f / float(totalFrames);
            }
            if (strmStats._active) {
                msg << L" Stream " << (i + 1)
                    << L" Dropped " << strmStats._droppedFrames
                    << L" of " << totalFrames
                    << L" frames (rendering " << setprecision(2) << fixed << strmStats.getRenderedFramesPerSecond()
                    << "fps, receiving " << strmStats.getTotalFramesPerSecond() << "fps)\n";
            }
        }

        PerformanceStats::ViewStats viewStats;
        _stats->getViewStats(_statsViewId, viewStats);
        PelcoImextk_StreamOptimizedType optimizedType;
        pelco_imextk_get_stream_optimized_type(_imextkContext, &optimizedType);
        std::wstringstream optimizedTypeString;
        if (optimizedType == PELCO_IMEXTK_OPTIMIZED_PERFORMANCE) {
            optimizedTypeString << "Processing Resources";
        }
        else {
            optimizedTypeString << "Image Quality";
        }

        msg << L" View " << _statsViewId
            << L" Render Count:  " << viewStats._renderedViews
            << L" (" << setprecision(2) << fixed << viewStats.getRenderedViewsPerSecond() << "fps)"
            << L"\n Optimization Bias:  " << optimizedTypeString.str();

        RECT r = {0, 0, labelWidth, labelHeight};
        DrawText(hDC, msg.str().c_str(), -1, &r, DT_LEFT);

        // set the alpha channel to fully opaque, for any pixel that's
        // been touched (GDI functions overwrite this, so we need to do it last).
        for (size_t pix = 0; pix < numPix; ++pix) {
            unsigned char* pBuf = &_statsTextImg._bmpBits[4 * pix];
            if ((pBuf[0] > 0) || (pBuf[1] > 0) || (pBuf[2] > 0)) {
                pBuf[3] = 0x90;
            }
        }
    }

    addBitmapToOverlay(0, 0,
            _statsTextImg._width, _statsTextImg._height,
            _statsTextImg._bmpBits);
}

//////////////////////////////////////////////////////////////////////////////////
//  This will graph the rolling bit rate
//
void VideoWindow::drawBitrateStats(bool undraw)
{
    unsigned int overlayStartingLine = 0;
    if (_frameCountOverlayMode == true) {
        overlayStartingLine = _frameCountOverlayHeight + 1;
    }
    unsigned int labelWidth = 700;
    unsigned int labelHeight = 65;

    resizeOffscreenStorage(_statsTextImg, labelWidth, labelHeight, false);
    HDC hDC = _statsTextImg._hDC;

    if (undraw) {
        memset(_statsTextImg._bmpBits, 0,
            _statsTextImg._width * _statsTextImg._height * 4);

        // There may be a chance that you turned off the frame count in the mean time
        //  which will not erase everything.  Add another write of zeros just in case.
        addBitmapToOverlay(0, _frameCountOverlayHeight + 1,
            _statsTextImg._width, _statsTextImg._height,
            _statsTextImg._bmpBits);
    }
    else {
        // BGRA layout
        // A  0 - totally transparent
        // A 0xff - totally opaque
        //
        // From experimentation, this will always set the A channel 
        //   to zero, so you need to change it after you draw the fonts
        //
        //SelectObject(hDC, GetStockObject(SYSTEM_FONT));
        // SetTextColor(hDC, 0x00010101);
        SetTextColor(hDC, 0x00010101);
        SetTextColor(hDC, 0x00ffffff);
        SelectObject(hDC, _hfont);
        SetBkColor(hDC, 0x00FFFFFF);
        SetBkColor(hDC, 0xf0f0f0);
        SetBkMode(hDC, TRANSPARENT);

        size_t numPix = labelWidth * labelHeight;
        memset(_statsTextImg._bmpBits, 0, numPix * 4);

        wstringstream msg;
        msg.precision(4);
        PerformanceStats::StreamStats strmStats;
        for (int i = 0; i < 6; ++i) {
            _stats->getStreamStats(i, strmStats);
            size_t totalFrames = strmStats._renderedFrames + strmStats._droppedFrames;
            float pct = 100.0f;
            if (totalFrames > 0) {
                pct = float(strmStats._renderedFrames) * 100.0f / float(totalFrames);
            }
            if (strmStats._active) {
                msg << "Average Bit Rate:  " << (strmStats.getAverageBitrate() / 1.0e6) << " Mbps\nCurrent Bit Rate:    " << (strmStats.getRollingBitrate() / 1.0e6) << " Mbps\n";
            }
        }

        RECT r = { 0, 0, labelWidth, labelHeight };
        DrawText(hDC, msg.str().c_str(), -1, &r, DT_LEFT);

        // set the alpha channel to fully opaque, for any pixel that's
        // been touched (GDI functions overwrite this, so we need to do it last).
        for (size_t pix = 0; pix < numPix; ++pix) {
            unsigned char* pBuf = &_statsTextImg._bmpBits[4 * pix];
            // Needs to match the text color chosen above
            if ((pBuf[0] == 0xff) && (pBuf[1] == 0xff) && (pBuf[2] == 0xff))    {
                pBuf[3] = 0xff;
                // Let's put a drop shadow at this horizontal location but one pixel line down 
                //   and also one pixel to the right
                // Be sure you have an extra row for your fonts !
                // Only do this if it is not part of the character string
                unsigned char * pBufDrop = pBuf + 4;
                if ((pBufDrop[0] == 0xff) && (pBufDrop[1] == 0xff) && (pBufDrop[2] == 0xff))    {
                    // This is part of a character, leave it alone 
                }
                else {
                    pBufDrop[3] = 0xff;
                }
                pBufDrop = &_statsTextImg._bmpBits[(4 * (pix + 1)) + (labelWidth * 4)];
                if ((pBufDrop[0] == 0xff) && (pBufDrop[1] == 0xff) && (pBufDrop[2] == 0xff))    {
                    // This is part of a character, leave it alone 
                }
                else {
                    pBufDrop[3] = 0xff;
                }
            } 
        }
    }

    addBitmapToOverlay(0, overlayStartingLine,
        _statsTextImg._width, _statsTextImg._height,
        _statsTextImg._bmpBits);

    // Make a graph of now
    //
    //  This will need to keep track of the last xxx rolling values
    //   and then make a graph 
    //  To do this, the rolling value will be saved for a graph
    //
    
    // Width needs to be divisible by 5
    //   Draw graph points with lines connecting them
    const unsigned int kGraphSpace = 5;
    const unsigned int kGraphPoints = 80;
    // The +1 is for space for a drop shadow
    const unsigned int kGraphWidth = (kGraphPoints - 1) * kGraphSpace + 1;
    const unsigned int kGraphHeight = 100;

    resizeOffscreenStorage(_bitrateGraphImg, kGraphWidth, kGraphHeight, false);
    hDC = _bitrateGraphImg._hDC;

    if (undraw) {
        memset(_bitrateGraphImg._bmpBits, 0,
            _bitrateGraphImg._width * _bitrateGraphImg._height * 4);
        // There may be a chance that you turned off the frame count in the mean time
        //  which will not erase everything.  Add another write of zeros just in case.
        addBitmapToOverlay(0, _frameCountOverlayHeight + 1 + labelHeight,
            _bitrateGraphImg._width, _bitrateGraphImg._height,
            _bitrateGraphImg._bmpBits);
    }
    else {
        memset(_bitrateGraphImg._bmpBits, 0, _bitrateGraphImg._width * _bitrateGraphImg._height * 4);

        // BGRA layout when accessing by byte
        // ARGB layout when accessing by int
        static double rollingBrGraph[kGraphWidth] = { 0 };
        static int graphValueIndex = 0;
        PerformanceStats::StreamStats strmStats;
        // Only care about stream 0 for now
        _stats->getStreamStats(0, strmStats);
        rollingBrGraph[graphValueIndex++] = strmStats.getRollingBitrate();
        graphValueIndex %= kGraphPoints;

        // I want 0,0 to be the bottom right corner of the graph
        // In graphics, this is the xsize,ysize coordinate, so adjust
        //
        // Make the very right of the graph 'now', and the left of that to be 
        // the history
        const double kMaxGraphHeight = 5.0E6;
        int yBeforeThisOne = 0;
        // assuming the bitmap is on a 4 byte boundary - should be guaranteed
        UINT32 * pBuf32 = (UINT32 *)_bitrateGraphImg._bmpBits;
        for (int i = 0; i < (kGraphPoints - 1); i++) {
            double graphYStart = ((rollingBrGraph[(graphValueIndex + i) % kGraphPoints]) / kMaxGraphHeight) * kGraphHeight;
            double graphYEnd =   ((rollingBrGraph[(graphValueIndex + 1 + i) % kGraphPoints]) / kMaxGraphHeight) * kGraphHeight;

            // Find the slope
            double slope = (graphYEnd - graphYStart) / kGraphSpace;

            for (int j = 0; j < kGraphSpace; j++) {
                //  y = mx + b
                double graphY = (slope * j) + graphYStart;

                int yInt = static_cast<int>(round(graphY));
                // Must be between 1 (because of drop shadow) and (kGraphHeight - 1)
                yInt = (yInt > (kGraphHeight - 1)) ? (kGraphHeight - 1) : yInt;
                yInt = (yInt == 0) ? 1 : yInt;

                // switch coordinate system
                yInt = (kGraphHeight - 1) - yInt;

                // If the last plotted point is not within one, then we 
                // want to draw a vertical line starting at the y value of 
                //  one plus the last y value 
                // Only do this after the first value
                if ((i == 0) && (j == 0)) {
                    // First one - ingore
                }
                else {
                    if (yInt != yBeforeThisOne) {
                        if (yInt > yBeforeThisOne) {
                            for (int k = 0; k < (yInt - yBeforeThisOne); k++) {
                                int index = (((yBeforeThisOne + k) * kGraphWidth) + (i * kGraphSpace) + j);
                                pBuf32[index] = 0xffffffff;

                                // Drop shadow
                                index += 1;
                                pBuf32[index] = 0xff000000;
                                index += kGraphWidth;
                                pBuf32[index] = 0xff000000;
                            }
                        }
                        else {
                            for (int k = 0; k < (yBeforeThisOne - yInt); k++) {
                                int index = (((yInt + k) * kGraphWidth) + (i * kGraphSpace) + j);
                                pBuf32[index] = 0xffffffff;

                                // Drop shadow
                                index += 1;
                                pBuf32[index] = 0xff000000;
                                index += kGraphWidth;
                                pBuf32[index] = 0xff000000;
                            }
                        }
                    }
                }
                int index = ((yInt * kGraphWidth) + (i * kGraphSpace) + j);
                pBuf32[index] = 0xffffffff;

                // Drop shadow
                index += 1;
                pBuf32[index] = 0xff000000;
                index += kGraphWidth;
                pBuf32[index] = 0xff000000;

                yBeforeThisOne = yInt;
            }
        }
    }
    
    addBitmapToOverlay(0, overlayStartingLine + labelHeight,
        _bitrateGraphImg._width, _bitrateGraphImg._height,
        _bitrateGraphImg._bmpBits);
}

#define TRACKRAWMOSAIC
//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawPositionTrackingText(bool undraw)
{
	LONG labelWidth = 200;
	LONG labelX = LONG(clientWidth()) - labelWidth;
	LONG labelY = 0;
	LONG labelHeight = 20;
	static const int NumRows = 4;
	int rowY = 0;

#ifdef TRACKRAWMOSAIC
	const int numLabels = 5;
#else
	const int numLabels = 4;
#endif

	resizeOffscreenStorage(_positionTrackingTextImg, labelWidth, numLabels*labelHeight, false);
	HDC hDC = _positionTrackingTextImg._hDC;

	if (undraw) {
		memset(_positionTrackingTextImg._bmpBits, 0,
			_positionTrackingTextImg._width * _positionTrackingTextImg._height * 4);
	}
	else {
		SelectObject(hDC, GetStockObject(SYSTEM_FONT));
		SetTextColor(hDC, 0x00010101);
		SetBkColor(hDC, 0x00FFFFFF);
		SetBkMode(hDC, OPAQUE);

		size_t numPix = labelWidth * numLabels*labelHeight;

		memset(_positionTrackingTextImg._bmpBits, 0, numPix);

		wstringstream msg;
		msg << L"Spherical: (" << (_trackedPanAngleRadians * 180.0f / float(M_PI))
			<< L", "
			<< (_trackedTiltAngleRadians * 180.0f / float(M_PI))
			<< L")";
		RECT r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
		DrawText(hDC, msg.str().c_str(), -1, &r, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
		rowY += labelHeight;

		msg.str(L"");
		msg << L"Zoom: " << _trackedZoomFactor;
		r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
		DrawText(hDC, msg.str().c_str(), -1, &r, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
		rowY += labelHeight;

		// draw an indicator of the current mouse position
		int x;
		int y;
		if (pelco_imextk_spherical_to_view(_viewGenerator,
			_trackedPanAngleRadians, _trackedTiltAngleRadians,
			&x, &y) == PELCO_IMEXTK_NO_ERROR) {

			wstringstream msg;
			msg << L"Screen:     (" << x << L", " << y << L")";
			RECT r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
			DrawText(hDC, msg.str().c_str(), -1, &r, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
			rowY += labelHeight;

			float panFOV, tiltFOV;
			if (pelco_imextk_get_view_angular_size(_viewGenerator,
				&panFOV, &tiltFOV) == PELCO_IMEXTK_NO_ERROR) {
				msg.str(L"");
				msg << L"View FOV:   (" << panFOV << L", " << tiltFOV << L")";
				r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
				DrawText(hDC, msg.str().c_str(), -1, &r, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
				rowY += labelHeight;
			}
		}

#ifdef TRACKRAWMOSAIC
		msg.str(L"");
		msg << L"MScreen: (" << _trackedRawMosaicPosX << L", " << _trackedRawMosaicPosY << L")";
		r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
		DrawText(hDC, msg.str().c_str(), -1, &r,
			DT_LEFT | DT_SINGLELINE | DT_VCENTER);
#endif
		// set the alpha channel to fully opaque, for any pixel that's
		// been touched (GDI functions overwrite this, so we need to do it last).
		for (size_t pix = 0; pix < numPix; ++pix) {
			unsigned char* pBuf = &_positionTrackingTextImg._bmpBits[4 * pix];
			if ((pBuf[0] > 0) || (pBuf[1] > 0) || (pBuf[2] > 0)) {
				pBuf[3] = 0x90;
			}
		}
	}

	addBitmapToOverlay(labelX, labelY,
		_positionTrackingTextImg._width, _positionTrackingTextImg._height,
		_positionTrackingTextImg._bmpBits);
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawCalibrationText(bool undraw)
{
	LONG labelWidth = 300;
	LONG labelHeight = 20;
	LONG labelY = 0;
	LONG labelX = LONG(clientWidth()/2) - labelWidth/2;
	int rowY = 0;
	const int numLabels = 5;

	resizeOffscreenStorage(_calibrationTextImg, labelWidth, numLabels*labelHeight, false);
	HDC hDC = _calibrationTextImg._hDC;

	memset(_calibrationTextImg._bmpBits, 0,
		_calibrationTextImg._width * _calibrationTextImg._height * 4);

	if (!undraw) {
		SelectObject(hDC, GetStockObject(SYSTEM_FONT));
		SetTextColor(hDC, 0x00010101);
		SetBkColor(hDC, 0x00FFFFFF);
		SetBkMode(hDC, OPAQUE);

		size_t numPix = labelWidth * numLabels*labelHeight;

		memset(_calibrationTextImg._bmpBits, 0, numPix);

		// get calibration state
		int calibrationState = _sitControl->getCalibrationState();
		bool separatedCameras = _sitControl->getCamerasSeparated();
		wstringstream msg1, msg2, msg3;

		switch (calibrationState) {
			case 1:
				switch (separatedCameras) {
				case true:
					msg1 << L"Pan Calibration:";
					msg2 << L"1) Adjust PTZ to point at Optera";
					msg3 << L"2) Adjust Optera to point at PTZ";
					break;
				case false:
					msg1 << L"Pan Calibration:";
					msg2 << L"1) Adjust PTZ to point at Common Pan Point";
					msg3 << L"2) Adjust Optera to point at Common Pan Point";
					break;
				}
				break;
			case 2:
				msg1 << L"Calibration - Point 1:";
				msg2 << L"1) Adjust PTZ to point at Point 1";
				msg3 << L"2) Adjust Optera to point at Point 1";
				break;
			case 3:
				msg1 << L"Calibration - Point 2:";
				msg2 << L"1) Adjust PTZ to point at Point 2";
				msg3 << L"2) Adjust Optera to point at Point 2";
				break;
			case 4:
				msg1 << L"Calibration - Point 3:";
				msg2 << L"1) Adjust PTZ to point at Point 3";
				msg3 << L"2) Adjust Optera to point at Point 3";
				break;
			case 5:
				msg1 << L"Calibration - Point 4:";
				msg2 << L"1) Adjust PTZ to point at Point 4";
				msg3 << L"2) Adjust Optera to point at Point 4";
				break;
			case 6:
				msg1 << L"Calibration Complete";
				msg2 << L"";
				msg3 << L"";
				break;
			default:
				msg1 << L"Unknown Calibration State";
				msg2 << L"";
				msg3 << L"";
				break;
		}

		RECT r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
		DrawText(hDC, msg1.str().c_str(), -1, &r, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
		rowY += labelHeight;

		r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
		DrawText(hDC, msg2.str().c_str(), -1, &r, DT_LEFT | DT_SINGLELINE | DT_VCENTER);
		rowY += labelHeight;

		r = { 0, rowY, labelX + labelWidth, rowY + labelHeight };
		DrawText(hDC, msg3.str().c_str(), -1, &r, DT_LEFT | DT_SINGLELINE | DT_VCENTER);

		// set the alpha channel to fully opaque, for any pixel that's
		// been touched (GDI functions overwrite this, so we need to do it last).
		for (size_t pix = 0; pix < numPix; ++pix) {
			unsigned char* pBuf = &_calibrationTextImg._bmpBits[4 * pix];
			if ((pBuf[0] > 0) || (pBuf[1] > 0) || (pBuf[2] > 0)) {
				pBuf[3] = 0x90;
			}
		}
	}

	addBitmapToOverlay(labelX, labelY,
		_calibrationTextImg._width, _calibrationTextImg._height,
		_calibrationTextImg._bmpBits);
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawMarker(Offscreen& img,
        unsigned int width, unsigned int height, COLORREF clr)
{
    resizeOffscreenStorage(img, width, height, false);

    SelectObject(img._hDC, img._bmp);
    SelectObject(img._hDC, GetStockObject(DC_PEN));
    SelectObject(img._hDC, GetStockObject(DC_BRUSH));
    SetDCBrushColor(img._hDC, clr);
    Rectangle(img._hDC, 0, 0, width, height);

    // set the alpha channel to fully opaque (GDI functions overwrite this,
    // so we need to do it last).
    for (size_t pix = 0; pix < (width * height); ++pix) {
        img._bmpBits[4 * pix + 3] = 0xFF;
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawRectangle(Offscreen& img,
    unsigned int width, unsigned int height, COLORREF clr) {
    unsigned int penWidth = 2;
    if (height > penWidth && width > penWidth) {
        resizeOffscreenStorage(img, width, height, false);

        SelectObject(img._hDC, img._bmp);
        SelectObject(img._hDC, GetStockObject(DC_PEN));
        SelectObject(img._hDC, GetStockObject(DC_BRUSH));
        SetDCPenColor(img._hDC, clr);
        Rectangle(img._hDC, 0, 0, width, height);

        // set the alpha channel to opaque on boundary(GDI functions overwrite this,
        // so we need to do it last).
        for (size_t pix = 0; pix < (width * penWidth); ++pix) {
            img._bmpBits[4 * pix + 3] = 0xFF;
            img._bmpBits[4 * (pix + (height - penWidth) * width) + 3] = 0xFF;
        }
        for (size_t my = penWidth; my < height - penWidth; ++my) {
            for (size_t mx = 0; mx < penWidth; ++mx) {
                // left
                img._bmpBits[4 * (my * width + mx) + 3] = 0xFF;
                // right
                img._bmpBits[4 * (my * width + (width - mx -1)) + 3] = 0xFF;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawDataBoundsMarker(Offscreen& img,
        float panAngle, float tiltAngle,
        float centerPan, float centerTilt)
{
    float globalPanAngle;
    float globalTiltAngle;
    if (pelco_imextk_map_spherical_coordinates(
                    panAngle, tiltAngle, centerPan, centerTilt,
                    &globalPanAngle, &globalTiltAngle) == PELCO_IMEXTK_NO_ERROR) {
        int x;
        int y;
        if (pelco_imextk_spherical_to_view(_viewGenerator,
                        globalPanAngle, globalTiltAngle,
                        &x, &y) == PELCO_IMEXTK_NO_ERROR) {
            addBitmapToOverlay(x - (img._width/2), y - (img._height/2),
                    img._width, img._height, img._bmpBits);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawDataBounds()
{
    float centerPan;
    float centerTilt;
    float width;
    float height;
    if (pelco_imextk_get_data_bounds(_imextkContext,
                    &centerPan, &centerTilt,
                    &width, &height) ==
            PELCO_IMEXTK_NO_ERROR) {

        drawDataBoundsMarker(_greenMarker, 0, 0, centerPan, centerTilt);
        drawDataBoundsMarker(_redMarker, -width/2.0f, 0, centerPan, centerTilt);
        drawDataBoundsMarker(_blueMarker, width/2.0f, 0, centerPan, centerTilt);
        drawDataBoundsMarker(_darkRedMarker, 0, height/2.0f, centerPan, centerTilt);
        drawDataBoundsMarker(_darkBlueMarker, 0, - height/2.0f, centerPan, centerTilt);
    }

#ifdef DEBUG_DATA_BOUNDS
    float* ptsBuf = NULL;
    int ptsBufLength = 0;

    if (pelco_imextk_get_all_data_bounds(_imextkContext,
                    &ptsBuf, &ptsBufLength) ==
            PELCO_IMEXTK_NO_ERROR) {
        for (int i = 0; i < ptsBufLength; i += 4) {
            int x;
            int y;
            // draw ideal point in green
            if (pelco_imextk_spherical_to_view(_viewGenerator,
                            ptsBuf[i], ptsBuf[i+1],
                            &x, &y) == PELCO_IMEXTK_NO_ERROR) {
                drawDataBoundsMarker(_greenMarker, x, y, centerPan, centerTilt);
            }

            // draw data-corrected point in blue
            if (pelco_imextk_spherical_to_view(_viewGenerator,
                            ptsBuf[i+2], ptsBuf[i+3],
                            &x, &y) == PELCO_IMEXTK_NO_ERROR) {
                drawDataBoundsMarker(_blueMarker, x, y, centerPan, centerTilt);
            }
        }

        free(ptsBuf);
    }
#endif
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::clearDataBoundsMarkers(void)
{
    if (_isShowingDataBounds) {
        float centerPan;
        float centerTilt;
        float width;
        float height;
        if (pelco_imextk_get_data_bounds(_imextkContext,
            &centerPan, &centerTilt,
            &width, &height) ==
            PELCO_IMEXTK_NO_ERROR) {

            drawDataBoundsMarker(_noMarker, 0, 0, centerPan, centerTilt);
            drawDataBoundsMarker(_noMarker, -width / 2.0f, 0, centerPan, centerTilt);
            drawDataBoundsMarker(_noMarker, width / 2.0f, 0, centerPan, centerTilt);
            drawDataBoundsMarker(_noMarker, 0, height / 2.0f, centerPan, centerTilt);
            drawDataBoundsMarker(_noMarker, 0, -height / 2.0f, centerPan, centerTilt);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::UpdateStreamImage(const unsigned char* pFrameData,
        size_t frameWidth, size_t frameHeight, bool isYUV)
{
    if ((_renderMode == UserOptions::RM_RAW) && (_offscreen._bmpBits != NULL)) {
        // in raw mode, we'll just copy the entire frame into bitmap memory,
        // then blt it onto the screen in the WM_PAINT event handler, OnPaint.
        resizeOffscreenStorage(_offscreen,
                unsigned int(frameWidth), unsigned int(frameHeight), isYUV);
        memcpy(_offscreen._bmpBits, pFrameData,
                _offscreen._width * _offscreen._height * (_offscreen._isYUV ? 1 : 4));

        // schedule the next render event.  These will get consolidated by Windows, so
        // we will generally get multiple UpdateStreamImage calls (one for each stream)
        // prior to actually rendering the combined data in OnPaint
        PostRedraw();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::notifyContentUpdated()
{
    drawNewView();
    if (isNonOpteraCam() == true) {
        UserOptions::PtzLimitOptions opts = getPtzLimitOptions();
        if (onvifPTZable() && (opts._mode == PELCO_IMEXTK_ONVIF_PTZ_SELECTED)) {
            if (WaitForSingleObject(_hZTimer, 0) == WAIT_OBJECT_0) {
                doOnvifZoom(0.0f);
                resetAndCancelTimer(_hZTimer);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
LRESULT VideoWindow::OnUserMessage(UINT msgID, WPARAM wparam, LPARAM lparam)
{
    LRESULT retval = 0;
    if (msgID == WM_USER_REDRAW) {
        drawNewView();
    }
    return retval;
}


//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawNewView()
{
    if ((_renderMode == UserOptions::RM_DIRECT) ||
            (_renderMode == UserOptions::RM_FRAMEBUFFER)) {

        if (_frameCountOverlayMode) {
            drawPerformanceStats(false);
        }

        if (_bitrateOverlayMode) {
            drawBitrateStats(false);
        }

        if (_isShowingDataBounds) {
            drawDataBounds();
		}
		if (_isShowingCalLine) {
			drawCalibrationLine();
		}
		if (_isShowingCalMark) {
			drawCalibrationMark();
		}
        OpenGlGuard grd(_glRenderDC, _hGLRC);
        //
        // This is where ViewGenerator performs its de-warping on whatever
        // data has previously been passed into it via UpdateStreamImage.
        // It renders results into GPU memory.  (Either this window's back buffer
        // or the offscreen window's frame buffer, depending on _renderMode)
        //
        if (_renderMode == UserOptions::RM_DIRECT) {
            _stats->incrementRenderedViews(_statsViewId);

            IMEXTK_CHECK(pelco_imextk_gl_render_direct(_viewGenerator));

            // update this window's display by swapping buffers
            SwapBuffers(_glRenderDC);
        }
        else if (_offscreen._bmpBits != NULL) {
            OpenGlGuard grd(_glRenderDC, _hGLRC);
            // If we're using a framebuffer, go ahead and render to the framebuffer now
            IMEXTK_CHECK(pelco_imextk_gl_render_to_buffer(_viewGenerator, _offscreen._bmpBits,
                            _offscreen._width));
            _stats->incrementRenderedViews(_statsViewId);
            PostRedraw();
        }
    }
    else {
        PostRedraw();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::resizeOffscreenStorage(Offscreen& offscreen,
        unsigned int width, unsigned int height, bool isYUV)
{
    if ((offscreen._bmp == NULL) ||
            (width != offscreen._width) ||
            (height != offscreen._height) ||
            (isYUV != offscreen._isYUV)) {
        cleanupOffscreenStorage(offscreen);

        offscreen._width = width;
        offscreen._height = height;
        offscreen._isYUV = isYUV;

        struct BmpInfo {
                BITMAPINFO info;
                RGBQUAD rgb[256];
        } bmpInfo;
        BITMAPINFO& info = bmpInfo.info;
        memset(&info, 0, sizeof(info));
        info.bmiHeader.biSize = sizeof(info);
        info.bmiHeader.biWidth = long(width);
        // height < 0 indicates top-down row order.
        info.bmiHeader.biHeight = -long(height);
        info.bmiHeader.biPlanes = 1;
        if (isYUV) {
            // we're only copying the Y plane for now,
            // rather than trying to convert RGB here.
            info.bmiHeader.biBitCount = 8;
            info.bmiHeader.biClrUsed = 256;
            for (int i = 0; i < 256; ++i) {
                bmpInfo.rgb[i].rgbRed = i;
                bmpInfo.rgb[i].rgbGreen = i;
                bmpInfo.rgb[i].rgbBlue = i;
                bmpInfo.rgb[i].rgbReserved = 0;
            }
        }
        else {
            info.bmiHeader.biBitCount = 32;
        }
        info.bmiHeader.biCompression = BI_RGB;
        offscreen._bmp = CreateDIBSection(offscreen._hDC, &info,
                DIB_RGB_COLORS, (void**)&offscreen._bmpBits,
                NULL, 0);
        if (offscreen._bmp == NULL) {
            DWORD err = GetLastError();
        }
        SelectObject(offscreen._hDC, offscreen._bmp);
    }
}

/////////////////////////////////////////////////////////////////////////
void VideoWindow::resetAndCancelTimer(HANDLE& handle)
{
    SetWaitableTimer(handle, &_liDueTimeForClearTimer, 0, NULL, NULL, 0);  //reset signal state to not signaled;
    CancelWaitableTimer(handle);
}

/////////////////////////////////////////////////////////////////////////
void VideoWindow::clearTrackedPositionMarker(void)
{
    if (_isTrackingPosition) {
        int x, y;
        // clear the old marker
        if (pelco_imextk_spherical_to_view(_viewGenerator,
            _trackedPanAngleRadians, _trackedTiltAngleRadians,
            &x, &y) == PELCO_IMEXTK_NO_ERROR) {
            unsigned int xPos = unsigned int(max(0, x - int(_noMarker._width / 2)));
            unsigned int yPos = unsigned int(max(0, y - int(_noMarker._height / 2)));
            addBitmapToOverlay(xPos, yPos,
                _noMarker._width, _noMarker._height, _noMarker._bmpBits);
        }
    }
}

/////////////////////////////////////////////////////////////////////////
bool VideoWindow::isNonOpteraCam(void)
{
    try {
        bool nonOptera = false;
        IMEXTK_CHECK(pelco_imextk_get_non_optera_cam_mode_setting(_imextkContext, &nonOptera));
        return nonOptera;
    }
    catch (...) {
        //on failure just assume optera cam
        return false;
    }
}

int VideoWindow::doOnvifGetStatus(float &panNorm, float &tiltNorm, float &zoomNorm)
{
	bool flag = false;
	PTZBindingProxy proxy(_pSoap);
	_tptz__GetStatus cm;
	cm.ProfileToken = _ptzToken;
	_tptz__GetStatusResponse cmr;
	
	// get status
	int result = SOAP_ERR;
	result = proxy.GetStatus(_ptzEndpoint.c_str(), NULL, &cm, cmr);
	if (result != SOAP_OK) {
		soap_print_fault(_pSoap, stderr);
	}
	else if (cmr.PTZStatus != NULL && cmr.PTZStatus->Position != NULL) {
		if (cmr.PTZStatus->Position->PanTilt != NULL) {
			panNorm = cmr.PTZStatus->Position->PanTilt->x;
			tiltNorm = cmr.PTZStatus->Position->PanTilt->y;
			flag = true;
		}
		if (cmr.PTZStatus->Position->Zoom != NULL) {
			zoomNorm = cmr.PTZStatus->Position->Zoom->x;
			flag &= true;
		}
	}
	return flag;
}

int VideoWindow::doOnvifPanTiltCont(float pan, float tilt)
{
	PTZBindingProxy proxy(_pSoap);
	_tptz__ContinuousMove cm;
	tt__PTZSpeed vel;
	tt__Vector2D pantilt;

	pantilt.x = pan;
	pantilt.y = tilt;
	vel.PanTilt = &pantilt;
	cm.ProfileToken = _ptzToken;
	cm.Velocity = &vel;
	_tptz__ContinuousMoveResponse cmr;
	int result = SOAP_ERR;
	result = proxy.ContinuousMove(_ptzEndpoint.c_str(), NULL, &cm, cmr);
	if (result != SOAP_OK) {
		soap_print_fault(_pSoap, stderr);
	}
	return result;
}

int VideoWindow::doOnvifPanTiltRel(float panDeltaNorm, float tiltDeltaNorm, float zoomNorm, float speedNorm)
{
	// if don't know current ptz absolute pan/tilt, read it back from camera
	//  - the camea read back position has limited resolution
	//  - after initial read back, use last cmd'd position which has more accuracy
	//  - calibration also uses the last cmd'd position instead of read back position for high accuracy
	if (_rawPtzPanNorm == 0 && _rawPtzTiltNorm == 0) {
		doOnvifGetStatus(_rawPtzPanNorm, _rawPtzTiltNorm, _rawPtzZoomNorm);
	}
	// add relative move delta's to current position
	_rawPtzPanNorm += panDeltaNorm;
	_rawPtzTiltNorm += tiltDeltaNorm;

	// wrap pan back in range if needed
	if (_rawPtzPanNorm < -1.0) _rawPtzPanNorm += 2.0;
	if (_rawPtzPanNorm > 1.0) _rawPtzPanNorm -= 2.0;

	// limit tilt to stay in range
	_rawPtzTiltNorm = limitValue(_rawPtzTiltNorm, -1.0f, 1.0f);
	int result = doOnvifPanTiltAbs(_rawPtzPanNorm, _rawPtzTiltNorm, _rawPtzZoomNorm, 1.0);
	return result;
}

int VideoWindow::doOnvifPanTiltAbs(float panNorm, float tiltNorm, float zoomNorm, float speedNorm)
{
	PTZBindingProxy proxy(_pSoap);
	_tptz__AbsoluteMove cm;
	tt__PTZSpeed speed;
	tt__Vector2D  panTilt;

	DEMOTRACE("doOnvifPanTiltAbs: panNormDegs = " << panNorm * 360 << " tiltNormDegs = " << tiltNorm * 360 << endl);

	panTilt.x = panNorm;
	panTilt.y = tiltNorm;
	tt__PTZVector position;
	cm.Position = &position;
	cm.ProfileToken = _ptzToken;
	tt__Vector2D  panTiltSpeed;
	speed.PanTilt = &panTiltSpeed;
	cm.Speed = &speed;
	_tptz__AbsoluteMoveResponse cmr;
	tt__Vector1D zoom;

	// set  position
	position.PanTilt = &panTilt;
	// set speed
	speed.PanTilt = &panTiltSpeed;
	panTiltSpeed.x = speedNorm;
	panTiltSpeed.y = speedNorm;
	// set zoom
	position.Zoom = &zoom;
	zoom.x = zoomNorm;
	
	// do move
	int result = SOAP_ERR;
	result = proxy.AbsoluteMove(_ptzEndpoint.c_str(), NULL, &cm, cmr);
	if (result != SOAP_OK) {
		soap_print_fault(_pSoap, stderr);
	}
	return result;
}

int VideoWindow::doOnvifZoom(float zoomVel)
{
    PTZBindingProxy proxy(_pSoap);
    _tptz__ContinuousMove cm;
    tt__PTZSpeed vel;
    tt__Vector1D zoom;
    zoom.x = zoomVel;
    vel.Zoom = &zoom;
    cm.ProfileToken = _ptzToken;
    cm.Velocity = &vel;
    _tptz__ContinuousMoveResponse cmr;
    int result = SOAP_ERR;
    result = proxy.ContinuousMove(_ptzEndpoint.c_str(), NULL, &cm, cmr);
    if (result != SOAP_OK) {
        soap_print_fault(_pSoap, stderr);
    }
    return result;
}
std::pair<float, float> VideoWindow::pointToPanTiltPair(const POINTS& pt)
{
    float clientH = static_cast<float>(clientHeight());
    float clientW = static_cast<float>(clientWidth());

    //we want below center to be negative so we flip the sign on y-axis
    float deltaY = -float(pt.y - (clientH / 2.0f)) / (clientH / 2.0f);
    float deltaX = float(pt.x - (clientW / 2.0f)) / (clientW / 2.0f);
    pair<float, float> pantilt(deltaX, deltaY);
    return pantilt;
}

void VideoWindow::doDigitalZoomPT(const POINTS& point, float deltaZ)
{
    float zoom;
    IMEXTK_CHECK(pelco_imextk_get_zoom(_viewGenerator, &zoom));

    zoom += deltaZ;
    zoom = min(max(zoom, 0.0f), 1.0f);
    IMEXTK_CHECK(pelco_imextk_set_zoom(_viewGenerator, zoom));

    float shiftX;
    float shiftY;
    float curPanShift;
    float curTiltShift;
    IMEXTK_CHECK(pelco_imextk_get_view_angle(_viewGenerator, &curPanShift, &curTiltShift));
    if (deltaZ > 0) {
        std::pair<float, float> pantilt = pointToPanTiltPair(point);
        curPanShift /= 2.0f;
        pantilt.first -= curPanShift;
        pantilt.second -= curTiltShift;
        float sMin = static_cast<float>(min((zoom * -1.0f), -0.05));
        float sMax = -sMin;
        shiftX = max(sMin, min(sMax, pantilt.first));
        shiftY = max(sMin, min(sMax, pantilt.second));
    }
    else {
        if (zoom == 0.0f) {
            shiftX = -(curPanShift);
            shiftY = -(curTiltShift);
        }
        else {
            shiftX = -(curPanShift * 0.2f);
            shiftY = -(curTiltShift * 0.2f);
        }
    }
    IMEXTK_CHECK(pelco_imextk_set_view_angle(_viewGenerator, shiftX, shiftY));
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawCalibrationLine(void)
{
	if (_isShowingCalLine) {
		int x = (int)clientWidth() / 2;
		int y = (int)clientHeight() / 2;
		if (pelco_imextk_view_to_spherical(_viewGenerator, x, y, &_calPanAngleRadians,
			&_calTiltAngleRadians) == PELCO_IMEXTK_NO_ERROR) {
			drawDataBoundsMarker(_blueCalLine, 0, 0, _calPanAngleRadians, _calTiltAngleRadians);
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::clearCalibrationLine(void)
{
	if (_isShowingCalLine) {
		int x, y;
		// clear old cal line
		if (pelco_imextk_spherical_to_view(_viewGenerator,
			_calPanAngleRadians, _calTiltAngleRadians,
			&x, &y) == PELCO_IMEXTK_NO_ERROR) {
			drawDataBoundsMarker(_noCalLine, 0, 0, _calPanAngleRadians, _calTiltAngleRadians);
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::drawCalibrationMark()
{
	// clear previous red marker
	clearCalibrationMark();

	// draw new red marker
	int x = _calMark.x;
	int y = _calMark.y;
	int xPos = 0;
	int yPos = 0;
	xPos = x - (_redMarker._width / 2);
	yPos = y - (_redMarker._width / 2);
	addBitmapToOverlay(xPos, yPos, _redMarker._width, _redMarker._height, _redMarker._bmpBits);
	_lastCalMark.x = x;
	_lastCalMark.y = y;
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindow::clearCalibrationMark()
{
	// clear red marker
	int x = _lastCalMark.x;
	int y = _lastCalMark.y;
	int xPos = 0;
	int yPos = 0;
	xPos = x - (_noMarker._width / 2);
	yPos = y - (_noMarker._width / 2);
	addBitmapToOverlay(xPos, yPos, _noMarker._width, _noMarker._height, _noMarker._bmpBits);
}

void VideoWindow::handleSituationAwarenessMode(SituationControl::SAControlMode mode)
{
    if (_sitControl.get()) {
		_sitControl->setControlMode(mode);
		if ((mode == SituationControl::SA_CALIBRATE_PTZ_PAN_OLD) ||
			(mode == SituationControl::SA_CALIBRATE_PTZ_PAN) ||
			(mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION) ||
			(mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND) ||
			(mode == SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS) ||
			(mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS)) {
			// save current ptz position
			_sitControl->getPtzLocation(_rawPtzPanNorm, _rawPtzTiltNorm, _rawPtzZoomNorm);
			// determine initial state
			SituationControl::CalibrationState initialState = SituationControl::CAL_PAN_STATE;
			if ((mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND) ||
				(mode == SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS) ||
				(mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS)) {
				initialState = SituationControl::CAL_POINT_1_STATE;
			}
			_sitControl->setCalibrationState(initialState);
			drawCalibrationText(false);
		}
		else {
			drawCalibrationText(true);
		}
		if (mode != SituationControl::SA_ZOOM_TO_BOX) {
			clearBoxForZoom();
		}
	}

	if ((mode == SituationControl::SA_CALIBRATE_PTZ_PAN_OLD) ||
		(mode == SituationControl::SA_CALIBRATE_PTZ_PAN) ||
		(mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION) ||
		(mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND) ||
		(mode == SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS) ||
		(mode == SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS)) {
		int width, height;
		IMEXTK_CHECK(pelco_imextk_get_view_size(_viewGenerator, &width, &height));
		_calMark.x = width / 2;
		_calMark.y = height / 2;
		_isShowingCalMark = true;
	}
	else {
		clearCalibrationMark();
		_isShowingCalMark = false;
	}

	drawNewView();
}

void VideoWindow::clearBoxForZoom(void)
{
    if (_zoomMarker._drawing) {
        // todo, create an array for clearing
        memset(_zoomMarker._bmpBits, 0, _zoomMarker._width * _zoomMarker._height * 4);
        // clear old drawing
        addBitmapToOverlay(_zoomMarker._xPos, _zoomMarker._yPos, _zoomMarker._width, _zoomMarker._height, _zoomMarker._bmpBits);
        _zoomMarker._drawing = false;
    }
}

// clear old box and draw new box based on new location
void VideoWindow::drawBoxForZoom(const POINTS& pt)
{
    clearBoxForZoom();
    int width = abs(pt.x - _zoomMarker._xAnchor);
    int height = abs(pt.y - _zoomMarker._yAnchor);
    if (width > 3 && height > 3) {
        _zoomMarker._xPos = min<int>(pt.x, _zoomMarker._xAnchor);
        _zoomMarker._yPos = min<int>(pt.y, _zoomMarker._yAnchor);
        drawRectangle(_zoomMarker, width, height, RGB(255, 255, 255));
        addBitmapToOverlay(_zoomMarker._xPos, _zoomMarker._yPos, _zoomMarker._width, _zoomMarker._height, _zoomMarker._bmpBits);
        _zoomMarker._drawing = true;
    }
}
