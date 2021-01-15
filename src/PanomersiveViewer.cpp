//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================

#include "stdafx.h"
#include "PanomersiveViewer.hpp"
#include "Resource.h"
#include "ErrorHandling.hpp"
#include <Shlobj.h>
#include <atlstr.h>
#include <cassert>
#include <fstream>
#include <winsock2.h>
#include <sstream>
#include <limits.h>
#include <RtpFrameSource.hpp>
#include <OpenH264FrameSource.hpp>

#pragma comment(lib, "gdiplus.lib")
#include <gdiplus.h>

#include "soapDeviceBindingProxy.h"
#include "soapMediaBindingProxy.h"
#include "soapPTZBindingProxy.h"
#include "numeric_calibrate.h"

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

//#define TEST_MOTION_GRID

#ifdef TEST_MOTION_GRID
static unsigned char activeMotionBlocksTest[22 * 15] = { 0 };
static bool toggle = false;
static int testIndex = 0;
#endif

extern void ReplaceOrAddCameraTiltAngle(std::string& streamLayoutString, float cameraTiltAngle);

using namespace std;

const string deviceServiceEndpointSuffix = "/onvif/device_service";
const string mediaServiceEndpointSuffix = "/onvif/media_service";
const string ptzServiceEndpointSuffix = "/onvif/ptz_service";

inline float degreesToRadians(float angle) {
    return angle * float(M_PI) / 180.0f;
}

inline float radiansToDegrees(float angle) {
    return angle * 180.0f / float(M_PI);
}

#define MAX_LOADSTRING 100
vector<wchar_t> PanomersiveViewer::_appTitle(MAX_LOADSTRING, '\0');

#define WM_USER_INIT_STREAM   (WM_USER + 1)
#define WM_USER_UPDATE_FRAME  (WM_USER + 2)
#define WM_USER_UPDATE_CAMERA_DLG_SELECTION (WM_USER + 3)

const size_t PanomersiveViewer::_kFramePoolCapacity = 2;

//////////////////////////////////////////////////////////////////////////////////
const wchar_t* PanomersiveViewer::loadTitleString(HINSTANCE hInstance)
{
    // Initialize global strings
    LoadString(hInstance, IDS_APP_TITLE,
            &_appTitle[0], static_cast<int>(_appTitle.size()));
    return &_appTitle[0];
}

//////////////////////////////////////////////////////////////////////////////////
HMENU PanomersiveViewer::loadMainMenu(HINSTANCE hInstance)
{
    HMENU hMenu = LoadMenu(hInstance, MAKEINTRESOURCE(IDC_DEMOAPP));
    if (hMenu == NULL) {
        throw runtime_error("Unable to load main menu");
    }
    return hMenu;
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::setMenusForCameraType(bool connectAsNonOpteraType, bool useSitAware)
{
    if (connectAsNonOpteraType == true) {
        //Use non opteras menu and settings
        SetMenu(_hWnd, NULL);
        DestroyMenu(_hMenu);
        HMENU hMenu = LoadMenu(_hInstance, MAKEINTRESOURCE(IDC_DEMOAPP_ANYCAM));
        if (hMenu == NULL) {
            throw runtime_error("Unable to load main menu");
        }
        _hMenu = hMenu;
        SetMenu(_hWnd, _hMenu);

        UserOptions::PtzLimitOptions opts = _options.getPtzLimitOptions();
        if (_ptzServiceEndpoint.empty() != true) {
            changePtzLimitMode(PELCO_IMEXTK_ONVIF_PTZ_SELECTED);
            ModifyMenu(_hMenu, IDM_LIMIT_NONE, MF_BYCOMMAND | MF_STRING, IDM_LIMIT_NONE, L"Digital PTZ Control");
            ModifyMenu(_hMenu, IDM_LIMIT_VIEW_CENTER, MF_BYCOMMAND | MF_STRING, IDM_LIMIT_VIEW_CENTER, L"Motorized PTZ Control");
        } else if ((opts._mode != PELCO_IMEXTK_PTZ_LIMIT_NONE) && (opts._mode != PELCO_IMEXTK_PTZ_LIMIT_VIEW_CENTER)) {
            changePtzLimitMode(PELCO_IMEXTK_PTZ_LIMIT_NONE);
        }
        syncPtzLimitOptions();
        changeViewLayout(UserOptions::VL_SINGLE);
    }
    else if (useSitAware == true) {
        SetMenu(_hWnd, NULL);
        DestroyMenu(_hMenu);
        HMENU hMenu = LoadMenu(_hInstance, MAKEINTRESOURCE(IDC_DEMOAPP_SIT_AWARE));
        if (hMenu == NULL) {
            throw runtime_error("Unable to load main menu");
        }
        _hMenu = hMenu;
        SetMenu(_hWnd, _hMenu);

        // To enable digital zoom on the Spectra HD view
        changePtzLimitMode(PELCO_IMEXTK_PTZ_LIMIT_NONE);
        CheckMenuItem(_hMenu, IDM_SIT_AWARE_AUTO, MF_CHECKED);

        syncViewLayout();
        syncViewLayoutPanoModId();
    }
    else {
        //use standard optera menu and settings
        SetMenu(_hWnd, NULL);
        DestroyMenu(_hMenu);
        HMENU hMenu = LoadMenu(_hInstance, MAKEINTRESOURCE(IDC_DEMOAPP));
        if (hMenu == NULL) {
            throw runtime_error("Unable to load main menu");
        }
        _hMenu = hMenu;
        SetMenu(_hWnd, _hMenu);
        syncViewLayout();
        syncViewLayoutPanoModId();
    }

    CheckMenuItem(_hMenu, IDM_FRAME_COUNT_OVERLAY, MF_BYCOMMAND |
        (_options.getFrameCountOverlayMode() ? MF_CHECKED : NULL));

    CheckMenuItem(_hMenu, IDM_BITRATE_OVERLAY, MF_BYCOMMAND |
        (_options.getBitrateOverlayMode() ? MF_CHECKED : NULL));

    CheckMenuRadioItem(_hMenu, IDM_RENDER_RAW, IDM_RENDER_FRAMEBUFFER,
        (IDM_RENDER_RAW + _options.getRenderMode()), MF_BYCOMMAND);

    CheckMenuRadioItem(_hMenu, IDM_DECODE_VLC, IDM_DECODE_METADATA,
        (IDM_DECODE_VLC + _options.getDecodeMode()), MF_BYCOMMAND);

    CheckMenuRadioItem(_hMenu, IDM_RENDER_OPTION_QUALITY, IDM_RENDER_OPTION_AUTOMATIC,
        (IDM_RENDER_OPTION_QUALITY + _options.getRenderOption()), MF_BYCOMMAND);


    DrawMenuBar(_hWnd);

}
HWND hwndOverall;
HDC deviceContextOverall;
//////////////////////////////////////////////////////////////////////////////////
PanomersiveViewer::PanomersiveViewer(HINSTANCE hInstance)
    : Window(defaultWindowClassInfo(hInstance), NULL, WS_OVERLAPPEDWINDOW,
    { CW_USEDEFAULT, 0, CW_USEDEFAULT, 0 },
    loadTitleString(hInstance),
    loadMainMenu(hInstance)),
    _imextkContext(PelcoImextk_kInvalidContext),
    _sitAwarePtzImextkContext(PelcoImextk_kInvalidContext),
    _streams(7), // 6 cube faces: max #streams for camera should be <= this. AND one for sit awareness PTZ
    _streamer(NULL),
    _streamerSitAware(NULL),
    _hGdiplus(NULL),
    _viewLayout(this),
    _stats(make_shared<PerformanceStats>()),
	_situationTracking(make_shared<SituationTracking>(_options)),
    _situationControl(make_shared<SituationControl>(_situationTracking))
{
    soap_init(&_soap);

    _viewPopUpSaved.cbSize = sizeof(MENUITEMINFO);
    // initialize networking
    WSADATA wsaDat;
    if (WSAStartup(MAKEWORD(2,2), &wsaDat) !=0 ) {
        throw runtime_error("Winsock error - Winsock initialization failed");
    }

    // initialize graphics lib used to read fixed image files.
    Gdiplus::GdiplusStartupInput gsi;
    Gdiplus::GdiplusStartup(&_hGdiplus, &gsi, NULL);

    /*
     * TODO - when we get appropriate graphics
     HBITMAP hUnchecked = LoadBitmap(hInstance, MAKEINTRESOURCE(IDB_LAYOUT_SINGLE));
     HBITMAP hChecked = LoadBitmap(hInstance, MAKEINTRESOURCE(IDB_LAYOUT_SINGLE_SELECTED));
     SetMenuItemBitmaps(_hMenu, IDM_VIEW_SINGLE,
     MF_BYCOMMAND, hChecked, hUnchecked);
    */

    CheckMenuRadioItem(_hMenu, IDM_VIEW_SINGLE, IDM_VIEW_ALL_MONITORS,
            IDM_VIEW_SINGLE, MF_BYCOMMAND);

    // create an opengl context for the top-level application window,
    // and make it current.  child VideoWindows will share resources
    // with this context.
    _hGLRC = createGlContext(_hDC, NULL);
    // There is an exception when we want to use a PTZ camera
    hwndOverall = ::CreateWindowA("STATIC", "dummy", WS_TILED, 0, 0, 400, 400, NULL, NULL, NULL, NULL);
    deviceContextOverall = GetDC(hwndOverall);
    _hSitAwarePtzGLRC = createGlContext(deviceContextOverall, NULL);

    CheckMenuItem(_hMenu, IDM_FRAME_COUNT_OVERLAY, MF_BYCOMMAND |
            (_options.getFrameCountOverlayMode() ? MF_CHECKED : NULL));

    CheckMenuItem(_hMenu, IDM_BITRATE_OVERLAY, MF_BYCOMMAND |
        (_options.getBitrateOverlayMode() ? MF_CHECKED : NULL));

    CheckMenuRadioItem(_hMenu, IDM_RENDER_RAW, IDM_RENDER_FRAMEBUFFER,
            (IDM_RENDER_RAW + _options.getRenderMode()), MF_BYCOMMAND);

    CheckMenuRadioItem(_hMenu, IDM_DECODE_VLC, IDM_DECODE_METADATA,
        (IDM_DECODE_VLC + _options.getDecodeMode()), MF_BYCOMMAND);

    CheckMenuRadioItem(_hMenu, IDM_RENDER_OPTION_QUALITY, IDM_RENDER_OPTION_AUTOMATIC,
        (IDM_RENDER_OPTION_QUALITY + _options.getRenderOption()), MF_BYCOMMAND);

    // TODO: command-line arguments to launch this app against a specific source
    // -- remove internal hard-coded sources.
    wchar_t path[MAX_PATH];
    GetModuleFileNameW(NULL, path, MAX_PATH);
    PathRemoveFileSpecW(path);
    loadFixedViewDir(wstring(path) + L"\\testImages\\NissiBeach2");

#ifdef USE_DEMO_BITMAP
#ifdef USE_DEMO_DATETIME_BITMAP
    std::string testFontStr = "Arial\0";
    std::wstring stemp = std::wstring(testFontStr.begin(), testFontStr.end());
    LPCWSTR sw = stemp.c_str();
    _testFont = CreateFont(16, 0, 0, 0, FW_MEDIUM, 0, 0, 0, 0, 0, 0, 0, 0, sw);
#endif
#endif
}

//////////////////////////////////////////////////////////////////////////////////
PanomersiveViewer::~PanomersiveViewer() {
    if (_hGdiplus != NULL) {
        Gdiplus::GdiplusShutdown(_hGdiplus);
    }
    closeSourcesViewsAndLayouts();
    soap_destroy(&_soap);
    soap_end(&_soap);
    soap_done(&_soap);

    WSACleanup();
}

//////////////////////////////////////////////////////////////////////////////////
bool PanomersiveViewer::OnCloseRequest()
{
    closeSourcesViewsAndLayouts();
    PostQuitMessage(0);
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
bool PanomersiveViewer::OnMenuCommand(int menuID)
{
    bool handled = true;
    switch (menuID) {
        case IDM_EXIT:
            PostMessage(_hWnd, WM_CLOSE, 0, 0);
            break;
        case IDM_ABOUT:
            DialogBox(_hInstance, MAKEINTRESOURCE(IDD_ABOUTBOX), _hWnd, cbAboutDlg);
            break;
        case IDM_GET_CAMERA_IP:
            DialogBox(_hInstance, MAKEINTRESOURCE(IDD_GET_CAMERA_IP), _hWnd, cbAskForCameraIPDlg);
            break;
        case IDM_GET_FIXED_VIEW_DIR:
            interactiveLoadFixedViewDir();
            break;

        case IDM_VIEW_SINGLE:
        case IDM_VIEW_DOUBLE:
        case IDM_VIEW_SA:
        case IDM_VIEW_SA_1:
        case IDM_VIEW_SA_2:
        case IDM_VIEW_ALL_MONITORS:
            // assume resource ID values are contiguous and in same order
            // as enum ViewLayout
            changeViewLayout(static_cast<UserOptions::ViewLayoutId>(
                        UserOptions::VL_SINGLE + (menuID - IDM_VIEW_SINGLE)));
            break;
        case IDM_VIEW_MOD_270_OPTIMIZED:
            toggleViewLayoutPanoMod();
            break;
        case IDM_LIMIT_NONE:
        case IDM_LIMIT_VIEW_EDGE:
        case IDM_LIMIT_VIEW_CENTER:
            changePtzLimitMode(static_cast<PelcoImextk_PtzLimitMode>(PELCO_IMEXTK_PTZ_LIMIT_NONE +
                            (menuID - IDM_LIMIT_NONE)));
            break;

        case IDM_AUTO_ZOOM_IN:
            toggleAutoZoomInWithPanTilt();
            break;
        case IDM_AUTO_ZOOM_OUT:
            toggleAutoZoomOutWithPanTilt();
            break;
        case IDM_AUTO_PAN_TILT:
            toggleAutoPanTiltWithZoomOut();
            break;

        case IDM_FRAME_COUNT_OVERLAY: {
            bool newMode = !_options.getFrameCountOverlayMode();
            _options.setFrameCountOverlayMode(newMode);
            for (size_t i = 0; i < _views.size(); ++i) {
                _views[i]->setFrameCountOverlayMode(newMode);
            }
            CheckMenuItem(_hMenu, IDM_FRAME_COUNT_OVERLAY, MF_BYCOMMAND |
                    (newMode ? MF_CHECKED : NULL));
        }
            break;

        case IDM_BITRATE_OVERLAY: {
            bool newMode = !_options.getBitrateOverlayMode();
            _options.setBitrateOverlayMode(newMode);
            for (size_t i = 0; i < _views.size(); ++i) {
                _views[i]->setBitrateOverlayMode(newMode);
            }
            CheckMenuItem(_hMenu, IDM_BITRATE_OVERLAY, MF_BYCOMMAND |
                (newMode ? MF_CHECKED : NULL));
        }
            break;

        case IDM_RENDER_RAW:
        case IDM_RENDER_DIRECT:
        case IDM_RENDER_FRAMEBUFFER:
            changeRenderMode(static_cast<UserOptions::RenderMode>(UserOptions::RM_RAW +
                            (menuID - IDM_RENDER_RAW)));
            break;
        case IDM_DECODE_VLC:
        case IDM_DECODE_METADATA:
            changeDecodeMode(static_cast<UserOptions::DecodeMode>(UserOptions::DM_VLC +
                            (menuID - IDM_DECODE_VLC)));
            break;
        case IDM_RENDER_OPTION_AUTOMATIC:
            changeRenderOption(PELCO_IMEXTK_OPTIMIZED_AUTOMATIC);
            break;
        case IDM_RENDER_OPTION_PERFORMANCE:
            changeRenderOption(PELCO_IMEXTK_OPTIMIZED_PERFORMANCE);
            break;
        case IDM_RENDER_OPTION_QUALITY:
            changeRenderOption(PELCO_IMEXTK_OPTIMIZED_QUALITY);
            break;

        case IDM_CAMERA_TILT: 
            {
            // launch the adjust-tilt modeless dialog
            HWND hDlg = CreateDialog(_hInstance, MAKEINTRESOURCE(IDD_ADJUST_CAMERA_TILT), _hWnd,
                    cbAdjustCameraTiltDlg);
            // disable the menu item that launched the dialog.  It'll be re-enabled upon closing.
            EnableMenuItem(_hMenu, IDM_CAMERA_TILT, MF_BYCOMMAND | MF_GRAYED);
            ShowWindow(hDlg, SW_SHOW);
            }
			break;
		case IDM_CAMERA_WEBPAGE:
			if (_options.getCurrentSourceInfo()._isCamera == true) {
				std::string cmd = "start http://";
				std::string ip = ws2s(_options.getCurrentSourceInfo()._id);
				cmd.append(ip);
				system(cmd.c_str());
			}
			break;
		case IDM_TEST:
		{
			test_bench();
			break;
		}
		case IDM_DEBUG:
		{
			// launch the debug modeless dialog
			HWND hDlg = CreateDialog(_hInstance, MAKEINTRESOURCE(IDD_GET_DEBUG_PARAMETER), _hWnd,
				cbDebugDlg);
			ShowWindow(hDlg, SW_SHOW);
		}
		break;
        case IDM_SIT_AWARE_AUTO:
        case IDM_SIT_AWARE_POINT_LOCATION:
        case IDM_SIT_AWARE_FOLLOW:
		case IDM_SIT_AWARE_ZOOM_TO_BOX:
		case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_OLD:
		case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN:
		case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION:
		case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION_BLIND:
		case IDM_SIT_AWARE_CALIBRATE_PTZ_POSITION_CAMERA_TILTS:
		case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS:
            HandleSituationalAwareChange(menuID);
            break;
		default:
            handled = false;
            break;
    }

    return handled;
}

void PanomersiveViewer::HandleSituationalAwareChange(int menuID)
{
    // Convert the IDM define to a enumerated type
    // This will also return if the menu type is not understood
    SituationControl::SAControlMode controlMode = SituationControl::SA_OFF;
    switch (menuID) {
    case IDM_SIT_AWARE_AUTO:
        controlMode = SituationControl::SA_AUTO;
        break;
    case IDM_SIT_AWARE_POINT_LOCATION:
        controlMode = SituationControl::SA_POINT_LOCATION;
        break;
    case IDM_SIT_AWARE_FOLLOW:
        controlMode = SituationControl::SA_FOLLOW_OBJECT;
        break;
    case IDM_SIT_AWARE_ZOOM_TO_BOX:
        controlMode = SituationControl::SA_ZOOM_TO_BOX;
		break;
	case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_OLD:
		controlMode = SituationControl::SA_CALIBRATE_PTZ_PAN_OLD;
		break;
	case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN:
		controlMode = SituationControl::SA_CALIBRATE_PTZ_PAN;
		break;
	case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION:
		controlMode = SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION;
		break;
	case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION_BLIND:
		controlMode = SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND;
		break;
	case IDM_SIT_AWARE_CALIBRATE_PTZ_POSITION_CAMERA_TILTS:
		controlMode = SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS;
		break;
	case IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS:
		controlMode = SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS;
		break;

    default:
        // Well, this is not being called in the right place
        controlMode = SituationControl::SA_OFF;
        break;
    }
    if (controlMode == SituationControl::SA_OFF) {
        return;
    }

    // There are some assumptions here
    //   the first view (index 0) is the mercator view
    //   the second view (index 1) is the PTZ view
    if (_views.size() > 0) {
        // The VideoWindow will take care to update the mode
        _views[0]->handleSituationAwarenessMode(controlMode);
    }
    if (_views.size() > 1) {
        _views[1]->handleSituationAwarenessMode(controlMode);
    }

    CheckMenuItem(_hMenu, IDM_SIT_AWARE_AUTO, FALSE);
    CheckMenuItem(_hMenu, IDM_SIT_AWARE_POINT_LOCATION, FALSE);
    CheckMenuItem(_hMenu, IDM_SIT_AWARE_FOLLOW, FALSE);
	CheckMenuItem(_hMenu, IDM_SIT_AWARE_ZOOM_TO_BOX, FALSE);
	CheckMenuItem(_hMenu, IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_OLD, FALSE);
	CheckMenuItem(_hMenu, IDM_SIT_AWARE_CALIBRATE_PTZ_PAN, FALSE);
	CheckMenuItem(_hMenu, IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION, FALSE);
	CheckMenuItem(_hMenu, IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION_BLIND, FALSE);
	CheckMenuItem(_hMenu, IDM_SIT_AWARE_CALIBRATE_PTZ_POSITION_CAMERA_TILTS, FALSE);
	CheckMenuItem(_hMenu, IDM_SIT_AWARE_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS, FALSE);
    CheckMenuItem(_hMenu, menuID, MF_CHECKED);

    // Have to do this last because it reads the value from 
    //  the class which is not updated until after the views are updated
    updateWindowTitle();
}

std::string PanomersiveViewer::ws2s(const std::wstring& s)
{
    int len;
    int slength = (int)s.length();// +1;
    len = WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, 0, 0, 0, 0);
    std::string r(len, ' ');
    WideCharToMultiByte(CP_ACP, 0, s.c_str(), slength, &r[0], len, 0, 0);
    return r;
}
//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::aboutToShowErrorPopup()
{
    try {
        closeSourcesViewsAndLayouts();
    }
    catch (...) {
        // just in case: swallow any error-within-error-handler problems (unlikely)
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::OnResize(int width, int height)
{
    if (_viewLayout == this) {
        if (!_views.empty()) {
            _views[0]->move(clientRect());
        }
    }
    else {
        _viewLayout->move(clientRect()); //{0, 0, width, height});
    }

#ifdef USE_DEMO_BITMAP
    loadTestBitmap(_views.size() - 1);
#endif
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::changeViewLayout(UserOptions::ViewLayoutId v)
{
    if (_options.getViewLayout() != v)
    {
        if (_options.setViewLayout(v)) {
            CheckMenuRadioItem(_hMenu, IDM_VIEW_SINGLE, IDM_VIEW_ALL_MONITORS,
                IDM_VIEW_SINGLE + v, MF_BYCOMMAND);

            // If in sit awareness mode, need to reset the cameras to get the other stream
            UserOptions::SourceHistoryRecord src = _options.getCurrentSourceInfo();
            if ((src._isCamera == true) && (src._useSitAwareness == true) &&
                ((v == UserOptions::VL_SA_2) || (v == UserOptions::VL_SA_2))) {
                viewCamera(src._id, src._logicalName, src._highRes, src._connectAsNonOpteraType, src._uniStream, src._primaryStream, src._multicast,
					src._useSitAwareness, src._sitAwarePtzId, src._separatedCameras, src._ptzHeight, src._opteraHeight, src._distanceToOptera, src._minMoveDistance,
					src._ptzGlobalTiltX, src._ptzGlobalTiltZ, src._ptzConeError, src._opteraGlobalTiltX, src._opteraGlobalTiltZ, src._opteraConeError);
            }
            else {
                resetViewsAndLayouts();
            }

#ifdef USE_DEMO_BITMAP
            loadTestBitmap(_views.size() - 1);
#endif
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::toggleViewLayoutPanoMod(void)
{
    UserOptions::ViewLayoutPanoModId newSetting;
    if (_options.getViewLayoutPanoMod() == UserOptions::VLPM_DEFAULT) {
        newSetting = UserOptions::VLPM_PELCO270_OPTIMIZED;
    }
    else {
        newSetting = UserOptions::VLPM_DEFAULT;
    }
    _options.setViewLayoutPanoMod(newSetting);

    //update menuitem checked or not
    CheckMenuItem(_hMenu, IDM_VIEW_MOD_270_OPTIMIZED, MF_BYCOMMAND | ((newSetting == UserOptions::VLPM_PELCO270_OPTIMIZED) ? MF_CHECKED : NULL));

    resetViewsAndLayouts();
}
//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::changeRenderMode(UserOptions::RenderMode mode)
{
    if (mode != _options.getRenderMode()) {
        bool toFromRawMode = (mode == UserOptions::RM_RAW) ||
            (_options.getRenderMode() == UserOptions::RM_RAW);

        _options.setRenderMode(mode);
        CheckMenuRadioItem(_hMenu, IDM_RENDER_RAW, IDM_RENDER_FRAMEBUFFER,
                (IDM_RENDER_RAW + mode), MF_BYCOMMAND);

        resetViewsAndLayouts();

#ifdef USE_DEMO_BITMAP
        loadTestBitmap(_views.size() - 1);
#endif
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::changeDecodeMode(UserOptions::DecodeMode mode)
{
    if (mode != _options.getDecodeMode()) {
        _options.setDecodeMode(mode);
        CheckMenuRadioItem(_hMenu, IDM_DECODE_VLC, IDM_DECODE_METADATA,
            (IDM_DECODE_VLC + _options.getDecodeMode()), MF_BYCOMMAND);

        // reload the current source -- need to restart since the decoding option is a
        // construction parameter to the _stream
        UserOptions::SourceHistoryRecord src = _options.getCurrentSourceInfo();
        if (src._isCamera) {
            viewCamera(src._id, src._logicalName, src._highRes, src._connectAsNonOpteraType, src._uniStream, src._primaryStream, src._multicast,
				src._useSitAwareness, src._sitAwarePtzId, src._separatedCameras, src._ptzHeight, src._opteraHeight, src._distanceToOptera, src._minMoveDistance,
				src._ptzGlobalTiltX, src._ptzGlobalTiltZ, src._ptzConeError, src._opteraGlobalTiltX, src._opteraGlobalTiltZ, src._opteraConeError);
        }

#ifdef USE_DEMO_BITMAP
        loadTestBitmap(_views.size() - 1);
#endif
    }
}
//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::changeRenderOption(PelcoImextk_StreamOptimizedType type)
{
    if (type != _options.getRenderOption()) {
        _options.setRenderOption(type);
        CheckMenuRadioItem(_hMenu, IDM_RENDER_OPTION_QUALITY, IDM_RENDER_OPTION_AUTOMATIC,
                           (IDM_RENDER_OPTION_QUALITY + _options.getRenderOption()), MF_BYCOMMAND);

        // reload the current source -- need to restart since the rendering option is a
        // construction parameter to the imextk context.
        UserOptions::SourceHistoryRecord src = _options.getCurrentSourceInfo();
        if (src._isCamera) {
            viewCamera(src._id, src._logicalName, src._highRes, src._connectAsNonOpteraType, src._uniStream, src._primaryStream, src._multicast,
				src._useSitAwareness, src._sitAwarePtzId, src._separatedCameras, src._ptzHeight, src._opteraHeight, src._distanceToOptera, src._minMoveDistance,
				src._ptzGlobalTiltX, src._ptzGlobalTiltZ, src._ptzConeError, src._opteraGlobalTiltX, src._opteraGlobalTiltZ, src._opteraConeError);
        }
        else {
            loadFixedViewDir(src._id);
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::updateViewLayouts() {
    const double VideoWindowSplit = 0.4;

    SplitterLayout* pSplitter1 = NULL;
    SplitterLayout* pSplitter2 = NULL;

    UserOptions::ViewLayoutId curLayout = _options.getViewLayout();

    bool corridorMode = false;
    SplitterLayout::Type normHorz_corrVert = SplitterLayout::Type::HORIZONTAL;
    SplitterLayout::Type normVert_corrHorz = SplitterLayout::Type::VERTICAL;

    pelco_imextk_get_corridor_mode(_imextkContext, &corridorMode);
    if (corridorMode) {
        normHorz_corrVert = SplitterLayout::Type::VERTICAL;
        normVert_corrHorz = SplitterLayout::Type::HORIZONTAL;
    }

    switch (curLayout) {
        case UserOptions::VL_SINGLE:
        case UserOptions::VL_SA:
        case UserOptions::VL_ALL_MONITORS:
            _viewLayout = this;
            if (curLayout == UserOptions::VL_ALL_MONITORS) {
                // full screen on all monitors
                for (size_t i = 0; i < _views.size(); ++i) {
                    _views[i]->move(getMonitorArea(i));
                }
            }
            else {
                _views[0]->move(clientRect());
            }
            break;
        case UserOptions::VL_DOUBLE:
            // Two side-by-side views
            _viewLayout = new SplitterLayout(normHorz_corrVert,
                    _hInstance, _hWnd,
                    clientRect(), _views[0], _views[1]);
            break;

        case UserOptions::VL_SA_1:
            // S.A. view is top X%
            pSplitter1 = new SplitterLayout(normVert_corrHorz,
                    _hInstance, _hWnd,
                    clientRect(), _views[0], _views[1]);
            pSplitter1->setSplit(VideoWindowSplit);
            _viewLayout = pSplitter1;
            break;

        case UserOptions::VL_SA_2:
            // bottom is two side-by-side immersive views
            pSplitter1 = new SplitterLayout(normHorz_corrVert,
                    _hInstance, _hWnd,
                    clientRect(), _views[1], _views[2]);

            // top is S.A. view
            pSplitter2 = new SplitterLayout(normVert_corrHorz,
                    _hInstance, _hWnd,
                    clientRect(), _views[0], pSplitter1);

            pSplitter2->setSplit(VideoWindowSplit);
            _viewLayout = pSplitter2;
            break;

        default:
            // programming error
            assert(false);
    }

    if (_viewLayout == this) {
        for (size_t i = 0; i < _views.size(); ++i) {
            _views[i]->show();
        }
    }
#ifdef USE_DEMO_BITMAP
    loadTestBitmap(_views.size() - 1);
#endif

    _viewLayout->show();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::changePtzLimitMode(PelcoImextk_PtzLimitMode mode)
{
    UserOptions::PtzLimitOptions opts = _options.getPtzLimitOptions();
    opts._mode = mode;
    if (_options.setPtzLimitOptions(opts)) {
        syncPtzLimitOptions();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::toggleAutoZoomInWithPanTilt()
{
    UserOptions::PtzLimitOptions opts = _options.getPtzLimitOptions();
    opts._autoZoomInWithPanTilt = !opts._autoZoomInWithPanTilt;
    _options.setPtzLimitOptions(opts);
    syncPtzLimitOptions();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::toggleAutoZoomOutWithPanTilt()
{
    UserOptions::PtzLimitOptions opts = _options.getPtzLimitOptions();
    opts._autoZoomOutWithPanTilt = !opts._autoZoomOutWithPanTilt;
    _options.setPtzLimitOptions(opts);
    syncPtzLimitOptions();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::toggleAutoPanTiltWithZoomOut()
{
    UserOptions::PtzLimitOptions opts = _options.getPtzLimitOptions();
    opts._autoPanTiltWithZoomOut = !opts._autoPanTiltWithZoomOut;
    _options.setPtzLimitOptions(opts);
    syncPtzLimitOptions();
}


//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::syncPtzLimitOptions()
{
    UserOptions::PtzLimitOptions opts = _options.getPtzLimitOptions();
    CheckMenuRadioItem(_hMenu, IDM_LIMIT_NONE, IDM_LIMIT_VIEW_CENTER,
            IDM_LIMIT_NONE + opts._mode, MF_BYCOMMAND);

    bool extraOptions = opts._mode == PELCO_IMEXTK_PTZ_LIMIT_VIEW_EDGE;
    EnableMenuItem(_hMenu, IDM_LIMIT_VIEW_EDGE_OPTIONS, MF_BYCOMMAND |
            (extraOptions ? MF_ENABLED : MF_GRAYED));
    if (extraOptions) {
        CheckMenuItem(_hMenu, IDM_AUTO_ZOOM_IN, MF_BYCOMMAND |
                (opts._autoZoomInWithPanTilt ? MF_CHECKED : NULL));

        CheckMenuItem(_hMenu, IDM_AUTO_ZOOM_OUT, MF_BYCOMMAND |
                (opts._autoZoomOutWithPanTilt ? MF_CHECKED : NULL));

        CheckMenuItem(_hMenu, IDM_AUTO_PAN_TILT, MF_BYCOMMAND |
                (opts._autoPanTiltWithZoomOut ? MF_CHECKED : NULL));
    }
    setPtzLimitOptionsInAllViews(opts);
}

void PanomersiveViewer::syncViewLayout(void)
{
    UserOptions::ViewLayoutId v = _options.getViewLayout();
    CheckMenuRadioItem(_hMenu, IDM_VIEW_SINGLE, IDM_VIEW_ALL_MONITORS,
        IDM_VIEW_SINGLE + v, MF_BYCOMMAND);
    resetViewsAndLayouts();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::setPtzLimitOptionsInAllViews(const UserOptions::PtzLimitOptions& opts)
{
    for (size_t i = 0; i < _views.size(); ++i) {
        _views[i]->setPtzLimitOptions(opts);
    }
}


//////////////////////////////////////////////////////////////////////////////////
RECT PanomersiveViewer::getMonitorArea(size_t monitorIndex) const
{
    struct QueryData {
            size_t count;
            size_t requestedIndex;
            RECT rect;
    };

    MONITORENUMPROC cb = [] (HMONITOR hMon, HDC hDCMon, LPRECT pMonRect, LPARAM userData) {
        QueryData* pQuery = reinterpret_cast<QueryData*>(userData);
        BOOL retval = true;
        if (pQuery->count == pQuery->requestedIndex) {
            pQuery->rect = *pMonRect;
            retval = false; // done searching
        }
        else {
            ++pQuery->count;
        }
        return retval;
    };

    QueryData query = {0};
    query.requestedIndex = monitorIndex;
    EnumDisplayMonitors(NULL, NULL, cb, reinterpret_cast<LPARAM>(&query));
    return query.rect;
//    return {query.rect.left, query.rect.top, query.rect.right/2, query.rect.bottom/2};
}

//////////////////////////////////////////////////////////////////////////////////
size_t PanomersiveViewer::getNumMonitors() const
{
    MONITORENUMPROC cb = [] (HMONITOR hMon, HDC hDCMon, LPRECT pMonRect, LPARAM userData) {
        int* count = reinterpret_cast<int*>(userData);
        *count = *count + 1;
        return BOOL(1);
    };
    int count = 0;
    EnumDisplayMonitors(NULL, NULL, cb, reinterpret_cast<LPARAM>(&count));
    return count;
}

//////////////////////////////////////////////////////////////////////////////////
size_t PanomersiveViewer::getPrimaryMonitorIndex() const
{
    MONITORENUMPROC cb = [] (HMONITOR hMon, HDC hDCMon, LPRECT pMonRect, LPARAM userData) {
        int* count = reinterpret_cast<int*>(userData);
        BOOL retval = 1;
        MONITORINFO info;
        info.cbSize = sizeof(info);
        if (GetMonitorInfo(hMon, &info) &&
                (info.dwFlags == MONITORINFOF_PRIMARY)) {
            retval = 0; // stop looking
        }
        else {
            *count = *count + 1;
        }
        return retval;
    };
    int mainMonitorIndex = 0;
    EnumDisplayMonitors(NULL, NULL, cb, reinterpret_cast<LPARAM>(&mainMonitorIndex));
    return mainMonitorIndex;
}

//////////////////////////////////////////////////////////////////////////////////
INT_PTR CALLBACK PanomersiveViewer::cbGetEncoderChannelDlg(HWND hDlg,
    UINT message, WPARAM wParam, LPARAM lParam)
{
    HWND hMainWindow = GetParent(hDlg);
    PanomersiveViewer* pApp =
        dynamic_cast<PanomersiveViewer*>((Window*)GetWindowLongPtr(hMainWindow, GWLP_USERDATA));
    assert(pApp != NULL);

    switch (message) {
        case WM_INITDIALOG: {
            // center the dialog in the parent window
            RECT inner;
            RECT outer = pApp->clientRect();
            GetWindowRect(hDlg, &inner);
            int innerWidth = inner.right - inner.left;
            int innerHeight = inner.bottom - inner.top;
            MoveWindow(hDlg,
                outer.left + ((outer.right - outer.left - innerWidth) / 2),
                outer.top + ((outer.bottom - outer.top - innerHeight) / 2),
                innerWidth, innerHeight, true);

            HWND encoderChannel = GetDlgItem(hDlg, IDC_ENCODER_CHANNEL_COMBO);
            int numberChannels = (pApp->_numberOfEncoderChannels > 99) ? 99 : pApp->_numberOfEncoderChannels;
            for (int i = 0; i < numberChannels; i++) {
                wstringstream itemText;
                itemText << (i + 1);
                SendMessage(encoderChannel, CB_ADDSTRING, 0, (LPARAM)itemText.str().c_str());
            } 

            // Select Channel one by default
            SendMessage(encoderChannel, CB_SETCURSEL, 0, 0);
        }
        return (INT_PTR)TRUE;
        break;

        case WM_COMMAND:
            if (HIWORD(wParam) == 0) {
                switch (LOWORD(wParam)) {
                    case IDOK:
                        pApp->_encoderChannelSelected = GetDlgItemInt(hDlg, IDC_ENCODER_CHANNEL_COMBO, FALSE, FALSE);
                        EndDialog(hDlg, (INT_PTR)NULL);
                        return (INT_PTR)TRUE;
                }
            }

        default:
            break;
    }
    return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////////////
INT_PTR CALLBACK PanomersiveViewer::cbAskForCameraIPDlg(HWND hDlg,
        UINT message, WPARAM wParam, LPARAM lParam)
{
    HWND hMainWindow = GetParent(hDlg);
    PanomersiveViewer* pApp =
        dynamic_cast<PanomersiveViewer*>((Window*)GetWindowLongPtr(hMainWindow, GWLP_USERDATA));
    assert(pApp != NULL);

    if (pApp->_selectingEncoderChannel == true) {
        return (INT_PTR)FALSE;
    }

    UNREFERENCED_PARAMETER(lParam);
    UserOptions::SourceHistoryRecord opts = pApp->_options.getCurrentSourceInfo();
    bool useSitAwareness = false;
	bool separatedCameras = false;
    switch (message) {
        case WM_INITDIALOG:
            pApp->InitCameraDlg(hDlg);
            return (INT_PTR)TRUE;

        case WM_COMMAND:
            if (HIWORD(wParam) == CBN_SELENDOK) {
                // since we want the text of the IP combo to be formatted differently
                // than the items in the list, we need to handle this function late.
                // Otherwise, the combobox will reset its text on us after we return.
                PostMessage(pApp->_hWnd, WM_USER_UPDATE_CAMERA_DLG_SELECTION, 0, (LPARAM)hDlg);
                return (INT_PTR)TRUE;
            }
            else if (HIWORD(wParam) == 0) {
                switch (LOWORD(wParam)) {
                    case IDOK:
                        if (!pApp->OnCameraDlgOk(hDlg)) {
                            return (INT_PTR) FALSE;
                        }
                        // fall through
                    case IDCANCEL:
                        EndDialog(hDlg, (INT_PTR)NULL);
                        return (INT_PTR)TRUE;

                    case IDC_NON_OPTERA_TYPE_RADIO:
                        CheckRadioButton(hDlg, IDC_NON_OPTERA_TYPE_RADIO, IDC_OPTERA_TYPE_RADIO, IDC_NON_OPTERA_TYPE_RADIO);
                        EnableWindow(GetDlgItem(hDlg, IDC_PRIMARY_STREAM_RADIO), TRUE);
                        EnableWindow(GetDlgItem(hDlg, IDC_SECONDARY_STREAM_RADIO), TRUE);

                        EnableWindow(GetDlgItem(hDlg, IDC_HIRES_CHECKBOX), FALSE);
                        EnableWindow(GetDlgItem(hDlg, IDC_UNISTREAM_CHECKBOX), FALSE);

                        if (opts._primaryStream == true) {
                            CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_PRIMARY_STREAM_RADIO);
                        }
                        else {
                            CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO);
                        }

                        EnableWindow(GetDlgItem(hDlg, IDC_SITAWARE_CHECKBOX), FALSE);
                        EnableWindow(GetDlgItem(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_PTZ_HEIGHT_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_HEIGHT_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_X_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_Z_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_PTZ_CONE_ERROR_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_X_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_Z_EDIT), FALSE);
						EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_CONE_ERROR_EDIT), FALSE);
						CheckDlgButton(hDlg, IDC_SITAWARE_CHECKBOX, FALSE);
						CheckDlgButton(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX, FALSE);
                        break;
                    case IDC_OPTERA_TYPE_RADIO:
                        CheckRadioButton(hDlg, IDC_NON_OPTERA_TYPE_RADIO, IDC_OPTERA_TYPE_RADIO, IDC_OPTERA_TYPE_RADIO);
                        EnableWindow(GetDlgItem(hDlg, IDC_HIRES_CHECKBOX), TRUE);
                        EnableWindow(GetDlgItem(hDlg, IDC_UNISTREAM_CHECKBOX), TRUE);

                        EnableWindow(GetDlgItem(hDlg, IDC_PRIMARY_STREAM_RADIO), FALSE);
                        EnableWindow(GetDlgItem(hDlg, IDC_SECONDARY_STREAM_RADIO), FALSE);
                        
                        if (opts._primaryStream == true) {
                            CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_PRIMARY_STREAM_RADIO);
                        }
                        else {
                            CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO);
                        }

                        EnableWindow(GetDlgItem(hDlg, IDC_SITAWARE_CHECKBOX), TRUE);
                        break;
                    case IDC_HIRES_CHECKBOX:
                        break;
                    case IDC_UNISTREAM_CHECKBOX:
                        break;
                    case IDC_PRIMARY_STREAM_RADIO:
                        CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_PRIMARY_STREAM_RADIO);
                        opts._primaryStream = true;
                        pApp->_options.updateCurrentSourceInfo(opts._logicalName, opts._highRes, opts._connectAsNonOpteraType, opts._uniStream, opts._primaryStream, opts._multicast,
							opts._useSitAwareness, opts._sitAwarePtzId, opts._separatedCameras, opts._ptzHeight, opts._opteraHeight, opts._distanceToOptera, opts._minMoveDistance, opts._ptzGlobalTiltX,
							opts._ptzConeError, opts._ptzGlobalTiltZ, opts._opteraGlobalTiltX, opts._opteraGlobalTiltZ, opts._opteraConeError);
                        break;

                    case IDC_SECONDARY_STREAM_RADIO:
                        CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO);
                        opts._primaryStream = false;
                        pApp->_options.updateCurrentSourceInfo(opts._logicalName, opts._highRes, opts._connectAsNonOpteraType, opts._uniStream, opts._primaryStream, opts._multicast,
							opts._useSitAwareness, opts._sitAwarePtzId, opts._separatedCameras, opts._ptzHeight, opts._opteraHeight, opts._distanceToOptera, opts._minMoveDistance, opts._ptzGlobalTiltX,
							opts._ptzGlobalTiltZ, opts._ptzConeError, opts._opteraGlobalTiltX, opts._opteraGlobalTiltZ, opts._opteraConeError);

                        break;
                    case IDC_MULTICAST_CHECKBOX:
                        break;
                    case IDC_SITAWARE_CHECKBOX:
                        useSitAwareness = (SendDlgItemMessage(hDlg, IDC_SITAWARE_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);
                        if (useSitAwareness == true) {
                            EnableWindow(GetDlgItem(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_PTZ_HEIGHT_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_HEIGHT_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_X_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_Z_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_PTZ_CONE_ERROR_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_X_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_Z_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_CONE_ERROR_EDIT), TRUE);
							EnableWindow(GetDlgItem(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX), TRUE);
							bool separatedCamera = (SendDlgItemMessage(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);
							if (separatedCamera) {
								EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), TRUE);
							}
							else {
								EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), FALSE);
							}
                        }
                        else {
                            EnableWindow(GetDlgItem(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT), FALSE);
                            EnableWindow(GetDlgItem(hDlg, IDC_PTZ_HEIGHT_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_HEIGHT_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_X_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_Z_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_PTZ_CONE_ERROR_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_X_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_Z_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_CONE_ERROR_EDIT), FALSE);
							EnableWindow(GetDlgItem(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX), FALSE);
                        }
						break;
					case IDC_CAMERAS_SEPARATED_CHECKBOX:
						separatedCameras = (SendDlgItemMessage(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);
						if (separatedCameras) {
							EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), TRUE);
							LPCWSTR emptyStr = L"";
							SetDlgItemText(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT, emptyStr);
						}
						else {
							EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), FALSE);
							LPCWSTR emptyStr = L"0";
							SetDlgItemText(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT, emptyStr);
						}
                    default:
                        break;
                }
            }
            break;

        default:
            break;
    }
    return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::InitCameraDlg(HWND hDlg)
{
    // center the dialog in the parent window
    RECT outer = clientRect();
    RECT inner;
    GetWindowRect(hDlg, &inner);
    int innerWidth = inner.right - inner.left;
    int innerHeight = inner.bottom - inner.top;
    MoveWindow(hDlg,
            outer.left + ((outer.right - outer.left - innerWidth) / 2),
            outer.top + ((outer.bottom - outer.top - innerHeight) / 2),
            innerWidth, innerHeight, true);

    HWND ipCombo = GetDlgItem(hDlg, IDC_IPADDR_COMBO);
    list<UserOptions::SourceHistoryRecord> history = _options.getHistory();
    list<UserOptions::SourceHistoryRecord>::const_iterator itr;
    for (itr = history.begin(); itr != history.end(); ++itr) {
        if (itr->_isCamera) {
            wstringstream itemText;
            if (!itr->_logicalName.empty()) {
                itemText << itr->_logicalName << L" ("
                         << itr->_id << L")";
            }
            else {
                itemText << itr->_id;
            }
            SendMessage(ipCombo, CB_ADDSTRING, 0, (LPARAM)itemText.str().c_str());
        }
    }
    if (!history.empty()) {
        // select the first (most recent) item in the list
        SendMessage(ipCombo, CB_SETCURSEL, 0, 0);
        // we don't get a selection message when we selected it programatically,
        // so call the handler from here.
        OnCameraDlgSelection(hDlg);
    }
    else {
        EnableWindow(GetDlgItem(hDlg, IDC_SITAWARE_CHECKBOX), FALSE);
        EnableWindow(GetDlgItem(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT), FALSE);
        EnableWindow(GetDlgItem(hDlg, IDC_PTZ_HEIGHT_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_HEIGHT_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_X_EDIT), FALSE); 
		EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_Z_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_PTZ_CONE_ERROR_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_X_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_Z_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_CONE_ERROR_EDIT), FALSE);
		EnableWindow(GetDlgItem(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX), FALSE);
        CheckDlgButton(hDlg, IDC_SITAWARE_CHECKBOX, FALSE);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::OnCameraDlgSelection(HWND hDlg)
{
    LPARAM selection = SendDlgItemMessage(hDlg, IDC_IPADDR_COMBO,
            CB_GETCURSEL, 0, 0);
    if (selection != CB_ERR) {
        list<UserOptions::SourceHistoryRecord> history = _options.getHistory();
        list<UserOptions::SourceHistoryRecord>::const_iterator itr = history.begin();
        size_t index = 0;
        while ((itr != history.end())) {
            if (itr->_isCamera) {
                if (index == selection) {
                    break;
                }
                ++index;
            }
            ++itr;
        }
        assert(itr != history.end()); // otherwise, we initialized the dlg wrong

        SetDlgItemText(hDlg, IDC_IPADDR_COMBO, itr->_id.c_str());

        SetDlgItemText(hDlg, IDC_CAMERA_NAME_EDIT, itr->_logicalName.c_str());

        SendDlgItemMessage(hDlg, IDC_HIRES_CHECKBOX, BM_SETCHECK, itr->_highRes ? BST_CHECKED : BST_UNCHECKED, 0);
        SendDlgItemMessage(hDlg, IDC_UNISTREAM_CHECKBOX, BM_SETCHECK, itr->_uniStream ? BST_CHECKED : BST_UNCHECKED, 0);
        SendDlgItemMessage(hDlg, IDC_MULTICAST_CHECKBOX, BM_SETCHECK, itr->_multicast ? BST_CHECKED : BST_UNCHECKED, 0);

        if ((itr->_connectAsNonOpteraType) == true) {
            CheckRadioButton(hDlg, IDC_NON_OPTERA_TYPE_RADIO, IDC_OPTERA_TYPE_RADIO, IDC_NON_OPTERA_TYPE_RADIO);
            EnableWindow(GetDlgItem(hDlg, IDC_HIRES_CHECKBOX), FALSE);
            EnableWindow(GetDlgItem(hDlg, IDC_UNISTREAM_CHECKBOX), FALSE);
            EnableWindow(GetDlgItem(hDlg, IDC_PRIMARY_STREAM_RADIO), TRUE);
            EnableWindow(GetDlgItem(hDlg, IDC_SECONDARY_STREAM_RADIO), TRUE);
            //primary/secondary radio button setting based on history
            if ((itr->_primaryStream) == true) {
                CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_PRIMARY_STREAM_RADIO);
            }
            else {
                CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO);
            }
            EnableWindow(GetDlgItem(hDlg, IDC_SITAWARE_CHECKBOX), FALSE);
            EnableWindow(GetDlgItem(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX), FALSE);
            EnableWindow(GetDlgItem(hDlg, IDC_PTZ_HEIGHT_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_HEIGHT_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_X_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_Z_EDIT), FALSE); 
			EnableWindow(GetDlgItem(hDlg, IDC_PTZ_CONE_ERROR_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_X_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_Z_EDIT), FALSE);
			EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_CONE_ERROR_EDIT), FALSE);
            CheckDlgButton(hDlg, IDC_SITAWARE_CHECKBOX, FALSE);
        }
        else {
            CheckRadioButton(hDlg, IDC_NON_OPTERA_TYPE_RADIO, IDC_OPTERA_TYPE_RADIO, IDC_OPTERA_TYPE_RADIO);
            EnableWindow(GetDlgItem(hDlg, IDC_HIRES_CHECKBOX), TRUE);
            EnableWindow(GetDlgItem(hDlg, IDC_UNISTREAM_CHECKBOX), TRUE);
            EnableWindow(GetDlgItem(hDlg, IDC_PRIMARY_STREAM_RADIO), TRUE);
            EnableWindow(GetDlgItem(hDlg, IDC_SECONDARY_STREAM_RADIO), TRUE);
            //primary/secondary radio button setting based on history
            if ((itr->_primaryStream) == true) {
                CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_PRIMARY_STREAM_RADIO);
            }
            else {
                CheckRadioButton(hDlg, IDC_PRIMARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO, IDC_SECONDARY_STREAM_RADIO);
            }
            EnableWindow(GetDlgItem(hDlg, IDC_PRIMARY_STREAM_RADIO), FALSE);
            EnableWindow(GetDlgItem(hDlg, IDC_SECONDARY_STREAM_RADIO), FALSE);

            if (itr->_useSitAwareness == true) {
                EnableWindow(GetDlgItem(hDlg, IDC_SITAWARE_CHECKBOX), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_HEIGHT_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_HEIGHT_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), itr->_separatedCameras);
				EnableWindow(GetDlgItem(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_X_EDIT), TRUE); 
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_Z_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_CONE_ERROR_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_X_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_Z_EDIT), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_CONE_ERROR_EDIT), TRUE);
				CheckDlgButton(hDlg, IDC_SITAWARE_CHECKBOX, TRUE);
				SetDlgItemText(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT, itr->_sitAwarePtzId.c_str());
				CheckDlgButton(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX, itr->_separatedCameras);
				SetDlgItemText(hDlg, IDC_PTZ_HEIGHT_EDIT, itr->_ptzHeight.c_str());
				SetDlgItemText(hDlg, IDC_OPTERA_HEIGHT_EDIT, itr->_opteraHeight.c_str());
				SetDlgItemText(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT, itr->_distanceToOptera.c_str());
				SetDlgItemText(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT, itr->_minMoveDistance.c_str());
				SetDlgItemText(hDlg, IDC_PTZ_TILT_X_EDIT, itr->_ptzGlobalTiltX.c_str());
				SetDlgItemText(hDlg, IDC_PTZ_TILT_Z_EDIT, itr->_ptzGlobalTiltZ.c_str());
				SetDlgItemText(hDlg, IDC_PTZ_CONE_ERROR_EDIT, itr->_ptzConeError.c_str());
				SetDlgItemText(hDlg, IDC_OPTERA_TILT_X_EDIT, itr->_opteraGlobalTiltX.c_str());
				SetDlgItemText(hDlg, IDC_OPTERA_TILT_Z_EDIT, itr->_opteraGlobalTiltZ.c_str());
				SetDlgItemText(hDlg, IDC_OPTERA_CONE_ERROR_EDIT, itr->_opteraConeError.c_str());
            }
            else {
                EnableWindow(GetDlgItem(hDlg, IDC_SITAWARE_CHECKBOX), TRUE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_HEIGHT_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_HEIGHT_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT), FALSE); 
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_X_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_TILT_Z_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_PTZ_CONE_ERROR_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_X_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_TILT_Z_EDIT), FALSE);
				EnableWindow(GetDlgItem(hDlg, IDC_OPTERA_CONE_ERROR_EDIT), FALSE);
                CheckDlgButton(hDlg, IDC_SITAWARE_CHECKBOX, FALSE);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool PanomersiveViewer::OnCameraDlgOk(HWND hDlg)
{
	_selectingEncoderChannel = false;
	wchar_t ipBuf[100];
	int ipLen = GetDlgItemText(hDlg, IDC_IPADDR_COMBO, ipBuf, 100);

    wchar_t nameBuf[100];
    int nameLen = GetDlgItemText(hDlg, IDC_CAMERA_NAME_EDIT, nameBuf, 100);

    bool highRes   = (SendDlgItemMessage(hDlg, IDC_HIRES_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);
    bool uniStream = (SendDlgItemMessage(hDlg, IDC_UNISTREAM_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);
    bool primaryStream = (SendDlgItemMessage(hDlg, IDC_PRIMARY_STREAM_RADIO, BM_GETCHECK, 0, 0) == BST_CHECKED);

    bool connectAsNonOpteraType = (SendDlgItemMessage(hDlg, IDC_NON_OPTERA_TYPE_RADIO, BM_GETCHECK, 0, 0) == BST_CHECKED);

    bool multicast = (SendDlgItemMessage(hDlg, IDC_MULTICAST_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);

    bool useSitAwareness = (SendDlgItemMessage(hDlg, IDC_SITAWARE_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);

    wchar_t sitAwarenessIpBuf[100];
    int sitAwarenessIpLen = GetDlgItemText(hDlg, IDC_PTZ_SITAWARE_IPADDR_EDIT, sitAwarenessIpBuf, 100);
	bool separatedCameras = (SendDlgItemMessage(hDlg, IDC_CAMERAS_SEPARATED_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);
    wchar_t ptzHeightBuf[100];
	int ptzHeightLen = GetDlgItemText(hDlg, IDC_PTZ_HEIGHT_EDIT, ptzHeightBuf, 100);
	wchar_t opteraHeightBuf[100];
	int opteraHeightLen = GetDlgItemText(hDlg, IDC_OPTERA_HEIGHT_EDIT, opteraHeightBuf, 100);
	wchar_t distanceToOpteraBuf[100];
	int distanceToOpteraLen = GetDlgItemText(hDlg, IDC_DISTANCE_TO_OPTERA_EDIT, distanceToOpteraBuf, 100);
	wchar_t minMoveDistanceBuf[100];
	int minMoveDistanceLen = GetDlgItemText(hDlg, IDC_MIN_MOVE_DISTANCE_EDIT, minMoveDistanceBuf, 100);
	wchar_t ptzTiltXBuf[100];
	int ptzTiltXLen = GetDlgItemText(hDlg, IDC_PTZ_TILT_X_EDIT, ptzTiltXBuf, 100);
	wchar_t ptzTiltZBuf[100];
	int ptzTiltZLen = GetDlgItemText(hDlg, IDC_PTZ_TILT_Z_EDIT, ptzTiltZBuf, 100);
	wchar_t ptzConeErrorBuf[100];
	int ptzConeErrorLen = GetDlgItemText(hDlg, IDC_PTZ_CONE_ERROR_EDIT, ptzConeErrorBuf, 100);
	wchar_t opteraTiltXBuf[100];
	int opteraTiltXLen = GetDlgItemText(hDlg, IDC_OPTERA_TILT_X_EDIT, opteraTiltXBuf, 100);
	wchar_t opteraTiltZBuf[100];
	int opteraTiltZLen = GetDlgItemText(hDlg, IDC_OPTERA_TILT_Z_EDIT, opteraTiltZBuf, 100);
	wchar_t opteraConeErrorBuf[100];
	int opteraConeErrorLen = GetDlgItemText(hDlg, IDC_OPTERA_CONE_ERROR_EDIT, opteraConeErrorBuf, 100);

    bool retval = false;
    try {
        //determine type
		std::string ipaddr(ipBuf, ipBuf + ipLen);
		std::string sitAwarenessIp(sitAwarenessIpBuf, sitAwarenessIpBuf + sitAwarenessIpLen);
        bool orgConnectType = connectAsNonOpteraType;
        determineIfNonOpteraTypeCameraAndNet55(ipaddr, connectAsNonOpteraType, _numberOfEncoderChannels);
        if (orgConnectType != connectAsNonOpteraType) {
            primaryStream = true;
        }

        _encoderChannelSelected = 0;
        if (_numberOfEncoderChannels == 1) {
            _encoderChannelSelected = 1;
        }
        if (_numberOfEncoderChannels > 1) {
            // This means we are attached to a NET55
            // For a one channel encoder, we do not need to put up a dialog box - can assume one
            // This is a little clumsy - but to populate the combo box, I need to know how many 
            //   encoder channels are available - so use a member variable
            _selectingEncoderChannel = true;
            DialogBox(_hInstance, MAKEINTRESOURCE(IDD_SELECT_ENCODER_CHANNEL), _hWnd, cbGetEncoderChannelDlg);
            _selectingEncoderChannel = false;
        }

		// determine if ptz availible (i.e. endpoint will not return empty if we can ptz)
		determinePTZEndpoint(ipaddr, _ptzServiceEndpoint, _PTZToken);

        // If in situational awareness with PTZ, go ahead and start in immersive view
        if ( (useSitAwareness) && (_options.getViewLayout() != UserOptions::VL_SA_1) && (_options.getViewLayout() != UserOptions::VL_SA_2) ) {
            _options.setViewLayout(UserOptions::VL_SA_1);
        }

        viewCamera(wstring(ipBuf, ipBuf + ipLen),
            wstring(nameBuf, nameBuf + nameLen), highRes, connectAsNonOpteraType, uniStream, primaryStream, multicast,
			useSitAwareness, wstring(sitAwarenessIpBuf, sitAwarenessIpBuf + sitAwarenessIpLen), separatedCameras,
			wstring(ptzHeightBuf, ptzHeightBuf + ptzHeightLen), wstring(opteraHeightBuf, opteraHeightBuf + opteraHeightLen),
			wstring(distanceToOpteraBuf, distanceToOpteraBuf + distanceToOpteraLen),
			wstring(minMoveDistanceBuf, minMoveDistanceBuf + minMoveDistanceLen),
			wstring(ptzTiltXBuf, ptzTiltXBuf + ptzTiltXLen),
			wstring(ptzTiltZBuf, ptzTiltZBuf + ptzTiltZLen),
			wstring(ptzConeErrorBuf, ptzConeErrorBuf + ptzConeErrorLen),
			wstring(opteraTiltXBuf, opteraTiltXBuf + opteraTiltXLen),
			wstring(opteraTiltZBuf, opteraTiltZBuf + opteraTiltZLen),
			wstring(opteraConeErrorBuf, opteraConeErrorBuf + opteraConeErrorLen));

		//setup menus based on camera type connecting
		setMenusForCameraType(connectAsNonOpteraType, useSitAwareness);

		// save endpoint in _view struct
		if (!useSitAwareness) {
			updatePTZEndpointInAllViews(ipaddr);
		}
		else {
			updatePTZEndpointForEachView(ipaddr, sitAwarenessIp);  // need ipaddr for each view
		}

        // After the views have been initialized, you can call this to set up 
        // the menu checkmarks and set the views to the correct viewing mode
        // Order is important here
        if (useSitAwareness == true) {
            HandleSituationalAwareChange(IDM_SIT_AWARE_AUTO);
        }
        retval = true;
    }
    catch (const std::exception& e) {
        showErrorPopup(hDlg, L"Camera Connection Error",
                       wstring(e.what(), e.what() + strlen(e.what())));
    }
    catch (...) {
        showErrorPopup(hDlg, L"Camera Connection Error",
                       L"Unknown error starting video stream");
    }
    return retval;
}

bool g_debugEnabled = false;
float g_debugParam1 = 0.0;
float g_debugParam2 = 0.0;
float g_debugParam3 = 0.0;
float g_debugParam4 = 0.0;
float g_debugParam5 = 0.0;
float g_debugParam6 = 0.0;

//////////////////////////////////////////////////////////////////////////////////
INT_PTR CALLBACK PanomersiveViewer::cbDebugDlg(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	HWND hMainWindow = GetParent(hDlg);
	PanomersiveViewer* pApp =
		dynamic_cast<PanomersiveViewer*>((Window*)GetWindowLongPtr(hMainWindow, GWLP_USERDATA));
	assert(pApp != NULL);

	UNREFERENCED_PARAMETER(lParam);
	switch (message) {
	case WM_INITDIALOG: {
		// center the dialog in the parent window
		RECT outer = pApp->clientRect();
		RECT inner;
		GetWindowRect(hDlg, &inner);
		int innerWidth = inner.right - inner.left;
		int innerHeight = inner.bottom - inner.top;
		MoveWindow(hDlg,
			outer.left + ((outer.right - outer.left - innerWidth) / 2),
			outer.top + ((outer.bottom - outer.top - innerHeight) / 2),
			innerWidth, innerHeight, true);

		EnableWindow(GetDlgItem(hDlg, IDC_ENABLE_DEBUG_CHECKBOX), TRUE);
		CheckDlgButton(hDlg, IDC_ENABLE_DEBUG_CHECKBOX, g_debugEnabled);
		EnableWindow(GetDlgItem(hDlg, IDC_DEBUG_PARAMETER_1_EDIT), TRUE);

		CString str;
		str.Format(_T("%g"), g_debugParam1);
		SetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_1_EDIT, (LPCWSTR)str);
		EnableWindow(GetDlgItem(hDlg, IDC_DEBUG_PARAMETER_2_EDIT), TRUE);
		str.Format(_T("%g"), g_debugParam2);
		SetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_2_EDIT, (LPCWSTR)str);
		EnableWindow(GetDlgItem(hDlg, IDC_DEBUG_PARAMETER_3_EDIT), TRUE);
		str.Format(_T("%g"), g_debugParam3);
		SetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_3_EDIT, (LPCWSTR)str);
		EnableWindow(GetDlgItem(hDlg, IDC_DEBUG_PARAMETER_4_EDIT), TRUE);
		str.Format(_T("%g"), g_debugParam4);
		SetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_4_EDIT, (LPCWSTR)str);
		EnableWindow(GetDlgItem(hDlg, IDC_DEBUG_PARAMETER_5_EDIT), TRUE);
		str.Format(_T("%g"), g_debugParam5);
		SetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_5_EDIT, (LPCWSTR)str);
		EnableWindow(GetDlgItem(hDlg, IDC_DEBUG_PARAMETER_6_EDIT), TRUE);
		str.Format(_T("%g"), g_debugParam6);
		SetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_6_EDIT, (LPCWSTR)str);
		}
		return (INT_PTR)TRUE;
		break;

	case WM_COMMAND:
		if (HIWORD(wParam) == 0) {
			switch (LOWORD(wParam)) {
			case IDAPPLY:
				pApp->OnDebugDlgOk(hDlg);
				return (INT_PTR)TRUE;

			case IDOK:
				pApp->OnDebugDlgOk(hDlg);
				// fall through

			case IDCANCEL:
				EndDialog(hDlg, (INT_PTR)NULL);
				return (INT_PTR)TRUE;

			case IDC_ENABLE_DEBUG_CHECKBOX: {
				g_debugEnabled = (SendDlgItemMessage(hDlg, IDC_ENABLE_DEBUG_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);
				return (INT_PTR)TRUE;
				}

			default:
				break;
			}
		}
		break;

	default:
		break;
	}
	return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////////////
bool PanomersiveViewer::OnDebugDlgOk(HWND hDlg)
{
	wchar_t debugParamBuf[100];
	int debugParamLen = GetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_1_EDIT, debugParamBuf, 100);
	wstring& debugParamWstr = wstring(debugParamBuf, debugParamBuf + debugParamLen);
	g_debugParam1 = (float)_wtof(debugParamWstr.c_str());
	debugParamLen = GetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_2_EDIT, debugParamBuf, 100);
	debugParamWstr = wstring(debugParamBuf, debugParamBuf + debugParamLen);
	g_debugParam2 = (float)_wtof(debugParamWstr.c_str());
	debugParamLen = GetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_3_EDIT, debugParamBuf, 100);
	debugParamWstr = wstring(debugParamBuf, debugParamBuf + debugParamLen);
	g_debugParam3 = (float)_wtof(debugParamWstr.c_str());
	debugParamLen = GetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_4_EDIT, debugParamBuf, 100);
	debugParamWstr = wstring(debugParamBuf, debugParamBuf + debugParamLen);
	g_debugParam4 = (float)_wtof(debugParamWstr.c_str());
	debugParamLen = GetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_5_EDIT, debugParamBuf, 100);
	debugParamWstr = wstring(debugParamBuf, debugParamBuf + debugParamLen);
	g_debugParam5 = (float)_wtof(debugParamWstr.c_str());
	debugParamLen = GetDlgItemText(hDlg, IDC_DEBUG_PARAMETER_6_EDIT, debugParamBuf, 100);
	debugParamWstr = wstring(debugParamBuf, debugParamBuf + debugParamLen);
	g_debugParam6 = (float)_wtof(debugParamWstr.c_str());

	g_debugEnabled = (SendDlgItemMessage(hDlg, IDC_ENABLE_DEBUG_CHECKBOX, BM_GETCHECK, 0, 0) == BST_CHECKED);

	return true;
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::determinePTZEndpoint(std::string ipaddr, std::string& endpoint, std::string& token)
{
    PTZBindingProxy ptzproxy(&_soap);

    _tptz__GetConfigurations gc;
    _tptz__GetConfigurationsResponse gcr;
    endpoint = "http://" + ipaddr + ptzServiceEndpointSuffix;
    int result = SOAP_ERR;
    result = ptzproxy.GetConfigurations(endpoint.c_str(), NULL, &gc, gcr);
    if ((result != SOAP_OK) || (gcr.PTZConfiguration.size() == 0)) {
		endpoint.clear();
    }
    else {
		token = gcr.PTZConfiguration[0]->token;
    }
	return;
}
//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::determineIfNonOpteraTypeCameraAndNet55(string ipaddr, bool& connectAsNonOpteraType, int& numberChannels)
{
    DeviceBindingProxy proxy(&_soap);

    _tds__GetDeviceInformation gdi;
    _tds__GetDeviceInformationResponse gdi_resp;
    std::string endpoint = "http://" + ipaddr + deviceServiceEndpointSuffix;

    int result = SOAP_ERR;

    result = proxy.GetDeviceInformation(endpoint.c_str(), NULL, &gdi, gdi_resp);
    if (result == SOAP_OK) {
        if ((gdi_resp.Model.find("IMM") == 0) || (gdi_resp.Model.find("PC_VIRT") == 0)) {
            //override - we have an optera camera.
            connectAsNonOpteraType = false;
        }
        else {
            connectAsNonOpteraType = true;
        }

        if (gdi_resp.Model.find("NET55") == 0) {
            // the numbers after NET55 are the number of channels
            string deviceName = gdi_resp.Model;
            string numChannels = deviceName.substr(5, deviceName.size() - 5);
            numberChannels = atoi(numChannels.c_str());
        }
        else {
            numberChannels = 0;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
// Helper for cbAdjustCameraTiltDlg
void enableCameraTiltDlgOverrideCtrls(HWND hDlg, bool enable)
{
    EnableWindow(GetDlgItem(hDlg, IDC_CAMERA_TILT_TRACKBAR), enable);
    EnableWindow(GetDlgItem(hDlg, IDC_CAMERA_TILT_OVERRIDE_EDIT), enable);
    EnableWindow(GetDlgItem(hDlg, IDC_CAMERA_TILT_STATIC), enable);
    EnableWindow(GetDlgItem(hDlg, IDC_CAMERA_TILT_INSTRUCTIONS_STATIC), enable);
    EnableWindow(GetDlgItem(hDlg, IDC_CAMERA_TILT_P90_STATIC), enable);
    EnableWindow(GetDlgItem(hDlg, IDC_CAMERA_TILT_N90_STATIC), enable);
}

//////////////////////////////////////////////////////////////////////////////////
INT_PTR CALLBACK PanomersiveViewer::cbAdjustCameraTiltDlg(HWND hDlg,
        UINT message, WPARAM wParam, LPARAM lParam)
{
    HWND hMainWindow = GetParent(hDlg);
    PanomersiveViewer* pApp =
        dynamic_cast<PanomersiveViewer*>((Window*)GetWindowLongPtr(hMainWindow, GWLP_USERDATA));
    assert(pApp != NULL);


    UNREFERENCED_PARAMETER(lParam);
    switch (message) {
        case WM_INITDIALOG: {
            SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_TRACKBAR,
                    TBM_SETRANGE, (WPARAM)true, MAKELPARAM(0, 180));
            SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_TRACKBAR,
                    TBM_SETPAGESIZE, 0, (LPARAM)5);

            UserOptions::SourceHistoryRecord opts = pApp->_options.getCurrentSourceInfo();

            SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_OVERRIDE_CHECKBOX, BM_SETCHECK,
                    opts._overrideCameraTiltAngle ? BST_CHECKED : BST_UNCHECKED, 0);
            enableCameraTiltDlgOverrideCtrls(hDlg, opts._overrideCameraTiltAngle);

            float angleRadians = opts._cameraTiltAngle;
            float angleDegrees = radiansToDegrees(angleRadians);
            SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_TRACKBAR,
                    TBM_SETPOS, (WPARAM)true, (LPARAM)int(90 - angleDegrees));
            SetDlgItemText(hDlg, IDC_CAMERA_TILT_OVERRIDE_EDIT,
                    (to_wstring(int(angleDegrees)) + L"°").c_str());

            float reportedAngleRadians = 0.0f;
            pelco_imextk_get_camera_tilt_angle(pApp->_imextkContext, &reportedAngleRadians);
            SetDlgItemText(hDlg, IDC_CAMERA_TILT_EDIT,
                    (L"(" + to_wstring(int(radiansToDegrees(reportedAngleRadians))) + L"°)").c_str());

            return (INT_PTR)TRUE;
        }

        case WM_COMMAND:
            if ((HIWORD(wParam) == BN_CLICKED) &&
                    (LOWORD(wParam) == IDC_CAMERA_TILT_OVERRIDE_CHECKBOX)) {
                bool overrideCamera =
                    SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_OVERRIDE_CHECKBOX, BM_GETCHECK,
                            0, 0) == BST_CHECKED;
                enableCameraTiltDlgOverrideCtrls(hDlg, overrideCamera);
                LRESULT pos = SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_TRACKBAR,
                        TBM_GETPOS, 0, 0);
                float angle = degreesToRadians(90.0f - float(pos));
                pApp->syncCameraAngle(overrideCamera, angle);
            }
            else if (HIWORD(wParam) == 0) {
                switch (LOWORD(wParam)) {
                    case IDOK: {
                        LRESULT pos = SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_TRACKBAR,
                                TBM_GETPOS, 0, 0);
                        float angle = degreesToRadians(90.0f - float(pos));
                        bool overrideCamera =
                            SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_OVERRIDE_CHECKBOX, BM_GETCHECK,
                                    0, 0) == BST_CHECKED;

                        pApp->_options.updateCameraTiltAngle(overrideCamera, angle);
                    }
                        // fall through
                    case IDCANCEL:
                        // restore views to persistent settings
                        pApp->syncCameraAngle();
                        // re-enable the menu item that lauched this modeless dialog
                        EnableMenuItem(pApp->_hMenu, IDM_CAMERA_TILT, MF_BYCOMMAND | MF_ENABLED);
                        DestroyWindow(hDlg);
                        return (INT_PTR)TRUE;
                    default:
                        break;
                }
            }
            break;

        case WM_VSCROLL:
            if (((HWND)lParam) == GetDlgItem(hDlg, IDC_CAMERA_TILT_TRACKBAR)) {
                LRESULT pos = SendDlgItemMessage(hDlg, IDC_CAMERA_TILT_TRACKBAR,
                        TBM_GETPOS, 0, 0);
                float angleDegrees = 90.0f - float(pos);
                float angleRadians = degreesToRadians(angleDegrees);
                pApp->setCameraAngleInAllViews(angleRadians);
                SetDlgItemText(hDlg, IDC_CAMERA_TILT_OVERRIDE_EDIT,
                        (to_wstring(int(angleDegrees)) + L"°").c_str());
                return (INT_PTR)TRUE;
            }
            break;

        default:
            break;
    }
    return (INT_PTR)FALSE;
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::closeSourcesViewsAndLayouts()
{
    // close views first, so that outstanding FrameBuffers will be released.
    // (avoids visible tearing during RtpStream->stop())
    closeViewsAndLayouts();

    if (_streamer != NULL) {
        // a streamer is currently running and making callbacks to us on
        // its background thread(s). Before we can stop the streamer, we
        // need to make sure that none of its threads are blocked waiting
        // for frame buffers:
        try {
            // 1. prevent any new messages from being posted to the UI thread
            unique_lock<recursive_mutex> lk(_stoppingMutex);
            for (size_t i = 0; i < _streams.size(); ++i) {
                _streams[i]._stopping = true;
            }
            // 2. handle any WM_USER_UPDATE_FRAME messages that
            // have already been posted.  These need to return frame data to imextk.
            // All of these messages had to have been posted prior to setting _stopping
            // above, so this should get them all.
            MSG msg;
            while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            }
        }
        catch(...) {
            // swallow errors -- makes sure we unlock
        }

        // 3. stop (and join) all the steamer threads.
        if (_streamer != NULL) {
            _streamer->stop();
            delete _streamer;
        }
        _streamer = NULL;

        if (_streamerSitAware != NULL) {
            _streamerSitAware->stop();
            delete _streamerSitAware;
        }
        setHourglass(false);
    }

    // get rid of corresponding imextk objects as well.
    for (size_t i = 0; i < _streams.size(); ++i) {
        if (_streams[i]._id != PelcoImextk_kInvalidVideoStream) {
            // return all outstanding buffers
            while (!_streams[i]._bufferPool.willReadBlock()) {
                StreamInfo::Frame frame;
                _streams[i]._bufferPool.read(frame);
                pelco_imextk_gl_release_frame_buffer(_streams[i]._id,
                        frame._yPlane, false);
            }

            pelco_imextk_gl_delete_video_stream(_streams[i]._id);
            _streams[i]._id = PelcoImextk_kInvalidVideoStream;
            _stats->resetStreamStats(i);
        }
    }

    if (_imextkContext != PelcoImextk_kInvalidContext) {
        pelco_imextk_gl_delete_context(_imextkContext);
        _imextkContext = PelcoImextk_kInvalidContext;
    }
    if (_sitAwarePtzImextkContext != PelcoImextk_kInvalidContext) {
        pelco_imextk_gl_delete_context(_sitAwarePtzImextkContext);
        _sitAwarePtzImextkContext = PelcoImextk_kInvalidContext;
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::deleteLayouts(WindowArea* pLayout)
{
    SplitterLayout* sp = dynamic_cast<SplitterLayout*>(pLayout);
    if (sp != NULL) {
        deleteLayouts(sp->getTopOrLeft());
        deleteLayouts(sp->getBottomOrRight());
        delete sp;
    }
    // don't delete the things held by the layout, they're owned by PanomersiveViewer.
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::closeViewsAndLayouts()
{
    deleteLayouts(_viewLayout);
    _viewLayout = this;

    deleteViews();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::deleteViews()
{
    for (size_t i = 0; i < _views.size(); ++i) {
        delete _views[i];
    }
    _views.clear();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::initViewsAndLayouts()
{
    // don't care about initial positioning - we'll move widets into place before
    // we show them
    RECT r { 0, 0, 1, 1 };

    size_t numViews = 1;
    size_t panoramicIndex = INT_MAX;
    size_t separateWindowStartIndex = INT_MAX;
    switch (_options.getViewLayout()) {
        case UserOptions::VL_SINGLE:
            numViews = 1;
            break;
        case UserOptions::VL_DOUBLE:
            numViews = 2;
            break;
        case UserOptions::VL_SA:
            numViews = 1;
            panoramicIndex = 0;
            break;
        // This view has mercator on the top, and immersive view on the bottom
        //  When using situational awareness with PTZ, you will need two contexts
        case UserOptions::VL_SA_1:
            numViews = 2;
            panoramicIndex = 0;
            break;
        case UserOptions::VL_SA_2:
            numViews = 3;
            panoramicIndex = 0;
            break;
        case UserOptions::VL_ALL_MONITORS:
            separateWindowStartIndex = 0;
            numViews = getNumMonitors();
            if (numViews > 1) {
                // if there's more than one monitor, make one panoramic, and the
                // others immersive
                panoramicIndex = getPrimaryMonitorIndex();
            }
            break;
        default:
            // programming error
            assert(false);
    }

    for (size_t i = 0; i < numViews; ++i) {
        PelcoImextk_ViewType vt = PELCO_IMEXTK_IMMERSIVE_VIEW;
        wstring viewTitle;
        if (i == panoramicIndex) {
            if (_options.getViewLayoutPanoMod() == UserOptions::VLPM_DEFAULT) {
                vt = PELCO_IMEXTK_MERCATOR_VIEW;
            }
            else {
                vt = PELCO_IMEXTK_270_SITUATIONAL_VIEW;
            }
            viewTitle = L"Panoramic View";
        }
        else {
            size_t immersiveCount = i + 1;
            if (i > panoramicIndex) {
                --immersiveCount;
            }
            viewTitle = wstring(L"Immersive View #") + to_wstring(immersiveCount);
        }
        bool useSitAwareness = (numViews <= 1) ? false : _options.getCurrentSourceInfo()._useSitAwareness;

        // Make the first situational view in pano over immersive mode to ptz camera if appropriate
        if ((useSitAwareness == true) && (i == 1)) {
            viewTitle = wstring(L"PTZ Camera View (") + _options.getCurrentSourceInfo()._sitAwarePtzId + wstring(L")");
            OpenGlGuard grd(deviceContextOverall, _hSitAwarePtzGLRC);
            _views.push_back(new VideoWindow(vt,
                                _hSitAwarePtzGLRC, 
                                _sitAwarePtzImextkContext,
                                _hInstance, _hWnd, r,
                                i >= separateWindowStartIndex,
                                viewTitle,
                                [this](VideoWindow::TrackingEvent evt, float pan, float tilt, float zoomFactor) {
									onPositionTrackingEvent(evt, pan, tilt, zoomFactor);
								},
								_options.getRenderMode(),
								_stats, &_soap, NULL));
        }
        else {
            bool setSituationControl = (useSitAwareness == true) && (i == 0);
            OpenGlGuard grd(_hDC, _hGLRC);
            _views.push_back(new VideoWindow(vt, 
                                _hGLRC, 
                                _imextkContext,
                                _hInstance, _hWnd, r,
                                i >= separateWindowStartIndex,
                                viewTitle,
                                [this](VideoWindow::TrackingEvent evt, float pan, float tilt, float zoomFactor) {
                                    onPositionTrackingEvent(evt, pan, tilt, zoomFactor);
                                },
                                _options.getRenderMode(),
                                _stats, &_soap, (setSituationControl ? _situationControl : NULL)));
            _views[i]->setFrameCountOverlayMode(_options.getFrameCountOverlayMode());
            _views[i]->setBitrateOverlayMode(_options.getBitrateOverlayMode());
        }
    }
    syncPtzLimitOptions();
    updateViewLayouts();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::onPositionTrackingEvent(VideoWindow::TrackingEvent evt,
        float panAngleRadians, float tiltAngleRadians, float /*zoomFactor*/) {
    bool trackingOn = true;
    static bool display = false;

#ifdef TEST_MOTION_GRID
    size_t width = 0;
    size_t height = 0;
    size_t cols = 0;
    size_t rows = 0;
    size_t rem = 0;
    unsigned char* pCamMotionArray = NULL;
#endif

    for (size_t i = 0; i < _views.size(); ++i) {
        switch (evt) {
            case VideoWindow::END_TRACKING:
                trackingOn = false;

#ifdef TEST_MOTION_GRID
                if (_views[i]->getViewType() != PELCO_IMEXTK_IMMERSIVE_VIEW) {
                    if (toggle == true) {
                        IMEXTK_CHECK(pelco_imextk_get_mosaic_dim(_imextkContext, &width, &height));
                        if ((width > 1) && (height > 1)) {
                            cols = width / 16;
                            rem = width % 16;
                            if (rem) {
                                cols++;
                            }

                            rows = height / 16;
                            rem = height % 16;
                            if (rem) {
                                rows++;
                            }

                            pCamMotionArray = new unsigned char[(rows*cols)];
                            memset(pCamMotionArray, 0, (rows*cols));
                            if (pCamMotionArray != NULL) {
                                unsigned int blocksWritten = 0;
                                IMEXTK_CHECK(pelco_imextk_gl_load_camera_motion_block_array(*(_views[i]->getViewGenerator()), pCamMotionArray, ((rows-1)*(cols-1)), 16, 16, &blocksWritten));
                                IMEXTK_CHECK(pelco_imextk_gl_apply_motion_grid(*(_views[i]->getViewGenerator()), pCamMotionArray, static_cast<unsigned int>(cols), static_cast<unsigned int>(rows), true));
                                delete pCamMotionArray;
                                pCamMotionArray = NULL;
                            }
                        }
                        toggle = false;
                    }
                    else {
                        
                        for (unsigned int row = 0; row < 15; row++) {
                            for (unsigned int col = 0; col < 22; col++) {
                                activeMotionBlocksTest[(row * 22) + col] = 1;
                            }
                        }

                        activeMotionBlocksTest[testIndex] = 1;
                        IMEXTK_CHECK(pelco_imextk_gl_apply_motion_grid(*(_views[i]->getViewGenerator()), activeMotionBlocksTest, 22, 15, true));
                        activeMotionBlocksTest[testIndex] = 0;
                        testIndex++; 
                        if (testIndex >= (22 * 15)) {
                            testIndex = 0;
                        }
                        toggle = true;
                    }
                }
#endif

                // fall through
            case VideoWindow::TRACK:
                _views[i]->trackPosition(trackingOn, panAngleRadians,
                        tiltAngleRadians);
                break;
            case VideoWindow::POINT_OF_INTEREST:
                // rotate (and maybe zoom) all views to the P.O.I.
                _views[i]->rotateTo(panAngleRadians, tiltAngleRadians, true);
                break;

            default:
                break;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::resetViewsAndLayouts()
{
    closeViewsAndLayouts();
    initViewsAndLayouts();
    PostRedraw();

    if ((_options.getRenderMode() == UserOptions::RM_RAW) &&
            (!_options.getCurrentSourceInfo()._isCamera)) {
        // If we're in RAW mode, and our source isn't
        // a camera (for which new frames keep coming), then we need to
        // reload source images into each of the new views.
        loadFixedViewDir(_options.getCurrentSourceInfo()._id);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::resetSourcesViewsAndLayouts(bool sourceIsYUV)
{
    closeSourcesViewsAndLayouts();
    // create a new context for sources and views

    {
        OpenGlGuard grd(_hDC, _hGLRC);
        IMEXTK_CHECK(pelco_imextk_gl_create_context(&_imextkContext, _options.getRenderOption(),
            sourceIsYUV ? PELCO_IMEXTK_I420 : PELCO_IMEXTK_BGRA));
    }
    if (sourceIsYUV == true) {
        OpenGlGuard grd(deviceContextOverall, _hSitAwarePtzGLRC);
        IMEXTK_CHECK(pelco_imextk_gl_create_context(&_sitAwarePtzImextkContext, _options.getRenderOption(),
            sourceIsYUV ? PELCO_IMEXTK_I420 : PELCO_IMEXTK_BGRA));
    }

    // enable optional features
    IMEXTK_CHECK(pelco_imextk_enable_camera_tilt_correction(_imextkContext, true));
    IMEXTK_CHECK(pelco_imextk_enable_corridor_mode_correction(_imextkContext, true));

    for (size_t i = 0; i < _streams.size(); ++i) {
        _streams[i]._stopping = false;
        _streams[i]._updating = false;
    }
    initViewsAndLayouts();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::viewCamera(const wstring& ip, const wstring& cameraName,
                                   bool highRes, bool connectAsNonOpteraType, bool uniStream, bool primaryStream, bool multicast,
								   bool useSitAwareness, const wstring& sitAwarePTZAddress, bool separatedCameras, const wstring& ptzHeight,
								   const wstring& opteraHeight, const wstring& distanceToOptera, const wstring& minMoveDistance, 
								   const wstring& ptzTiltX, const wstring& ptzTiltZ, const wstring& ptzConeError, const wstring& opteraTiltX, 
								   const wstring& opteraTiltZ, const wstring& opteraConeError)
{
    resetSourcesViewsAndLayouts(true);

	// Save history
	if (_options.selectCamera(ip)) {
		_options.updateCurrentSourceInfo(cameraName, highRes, connectAsNonOpteraType, uniStream, primaryStream, multicast, useSitAwareness,
			sitAwarePTZAddress, separatedCameras, ptzHeight, opteraHeight, distanceToOptera, minMoveDistance, ptzTiltX, ptzTiltZ, ptzConeError, opteraTiltX, opteraTiltZ, opteraConeError);
	}
	else {
		_options.addCamera(ip, cameraName, highRes, connectAsNonOpteraType, uniStream, primaryStream, useSitAwareness, sitAwarePTZAddress, separatedCameras,
			ptzHeight, opteraHeight, distanceToOptera, minMoveDistance, ptzTiltX, ptzTiltZ, ptzConeError, opteraTiltX, opteraTiltZ, opteraConeError);
	}
	
	// StreamSetupCallback for _streamer
    auto setupHandler = [this](size_t streamIndex,
            size_t frameWidth, size_t frameHeight,
            const string& layoutMetadata, bool sitAwarenessStreamer) {
        // one-time init of the layout metadata for this stream.
        size_t retval = 0;
        unique_lock<recursive_mutex> stopLock(_stoppingMutex);
        if (!_streams[streamIndex]._stopping) {
            unique_lock<mutex> initLock(_streams[streamIndex]._initMutex);
            _streams[streamIndex]._layoutMetadata = layoutMetadata;
            _streams[streamIndex]._frameWidth = frameWidth;
            _streams[streamIndex]._frameHeight = frameHeight;

            // actual stream setup needs to happen on UI thread (since it's operating
            // on an opengl context).
            PostMessage(_hWnd, WM_USER_INIT_STREAM, WPARAM(streamIndex), sitAwarenessStreamer);

            // block here until stream is initialized (successfully or not)
            stopLock.unlock();
            _streams[streamIndex]._initComplete.wait(initLock);

            if (_streams[streamIndex]._id == PelcoImextk_kInvalidVideoStream) {
                stopLock.lock();
                _streams[streamIndex]._stopping = true;
                retval = true;
            }
            else {
                retval = _kFramePoolCapacity;
            }
        }
        return retval;
    };

    // GetBufferCallback for _streamer
    auto getBufferHandler = [this](size_t streamIndex, unsigned char*& pYPlane,
            unsigned char*& pUPlane, unsigned char*& pVPlane) {
        StreamInfo::Frame frame = { NULL, NULL, NULL };
        _streams[streamIndex]._bufferPool.read(frame);
        pYPlane = frame._yPlane;
        pUPlane = frame._uPlane;
        pVPlane = frame._vPlane;
    };

    // ReleaseBufferCallback for _streamer
    auto releaseBufferHandler = [this](size_t streamIndex, unsigned char* yPlane,
            unsigned char* uPlane, unsigned char* vPlane, bool lastInGroup, double avgBitRate, double rollingBitRate) {
        // Check if we're trying to stop, or if we're still processing the previous
        // frame (i.e., the GPU is unable to keep up).
        unique_lock<recursive_mutex> lock(_stoppingMutex);

        _stats->setAverageBitrate(streamIndex, avgBitRate);
        _stats->setRollingBitrate(streamIndex, rollingBitRate);

        if (_streams[streamIndex]._stopping || _streams[streamIndex]._updating) {
            // Skip display of this frame by simply returning it to the pool.
            StreamInfo::Frame frame = { yPlane, uPlane, vPlane };
            _streams[streamIndex]._bufferPool.write(frame);
            _stats->incrementDroppedFrames(streamIndex);
        }
        else {
            // post a message to the UI thread to schedule return of
            // this buffer to imextk, and refresh of all views.
            _streams[streamIndex]._updating = true;
            WPARAM doRender = lastInGroup ? 0x10 : 0x00;
            PostMessage(_hWnd, WM_USER_UPDATE_FRAME, WPARAM(streamIndex) | doRender, LPARAM(yPlane));
        }
    };

    vector<string> videoURLs;
    //if (connectAsNonOpteraType == true) {
        videoURLs = findNonOpteraStreamURL(ip, primaryStream, multicast);
    //}    

    // Construct _streamer, binding to above callbacks
    if (_options.getDecodeMode() == UserOptions::DM_VLC) {
        _streamer = new RtpFrameSource(string(ip.begin(), ip.end()), setupHandler, getBufferHandler, releaseBufferHandler, highRes, connectAsNonOpteraType, uniStream, primaryStream,
			videoURLs, multicast, false, _encoderChannelSelected);
    }
    else {
        // To use Live555 and Openh264
        _streamer = new OpenH264FrameSource(string(ip.begin(), ip.end()), setupHandler, getBufferHandler, releaseBufferHandler, highRes, connectAsNonOpteraType, uniStream, primaryStream, 
			videoURLs, multicast, useSitAwareness, string(sitAwarePTZAddress.begin(), sitAwarePTZAddress.end()), separatedCameras, (float)_wtof(opteraHeight.c_str()), (float)_wtof(ptzHeight.c_str()),
			(float)_wtof(distanceToOptera.c_str()), (float)_wtof(minMoveDistance.c_str()), (float)_wtof(ptzTiltX.c_str()), (float)_wtof(ptzTiltZ.c_str()), (float)_wtof(ptzConeError.c_str()),
			(float)_wtof(opteraTiltX.c_str()), (float)_wtof(opteraTiltZ.c_str()), (float)_wtof(opteraConeError.c_str()), false, _situationTracking, _encoderChannelSelected);

        // To get the situational awareness view, use the VLC decoder since the OpenH264 is not architected for more than one camera
        if ((useSitAwareness == true) && ((_options.getViewLayout() == UserOptions::VL_SA_1) || (_options.getViewLayout() == UserOptions::VL_SA_2)) ) {
            _streamerSitAware = new RtpFrameSource(string(sitAwarePTZAddress.begin(), sitAwarePTZAddress.end()), setupHandler, getBufferHandler, releaseBufferHandler, highRes, true, false, true, videoURLs, false, true, _encoderChannelSelected);
            // Experiment that is not working yet - work to be done later
            //_streamerSitAware = new OpenH264FrameSource(string(sitAwarePTZAddress.begin(), sitAwarePTZAddress.end()), setupHandler, getBufferHandler, releaseBufferHandler, highRes, true, false, true, videoURLs, false,
            //                                    false, string(sitAwarePTZAddress.begin(), sitAwarePTZAddress.end()), true );
        }
    }
    setHourglass(true);
    if (_streamer != NULL)
        _streamer->start();
    if (_streamerSitAware != NULL) {
        _streamerSitAware->start();
    }

    syncViewLayoutPanoModId();
    updateWindowTitle();
}

//////////////////////////////////////////////////////////////////////////////////
vector<string> PanomersiveViewer::findNonOpteraStreamURL(wstring ip, bool& primaryStream, bool multicast)
{
    MediaBindingProxy proxy(&_soap);

    _trt__GetProfiles gp;
    _trt__GetProfilesResponse gpr;
    std::string camip = ws2s(ip);
    std::string endpoint = "http://" + camip + mediaServiceEndpointSuffix;
    vector<std::string> videoURLs;
    unsigned int index = 0;

    int result = SOAP_ERR;
    result = proxy.GetProfiles(endpoint.c_str(), NULL, &gp, gpr);
    if (result == SOAP_OK) {
        if (primaryStream == false && (gpr.Profiles.size() < 2)) {
            primaryStream = true; //secondary not available
        }

        for (size_t index = 0; index < gpr.Profiles.size(); ++index) {
            _trt__GetStreamUri gsu;
            _trt__GetStreamUriResponse gsur;
            tt__StreamSetup streamSetup;
            tt__Transport transport;
            gsu.ProfileToken = gpr.Profiles[index]->token;
            gsu.StreamSetup = &streamSetup;
            gsu.StreamSetup->Transport = &transport;
            gsu.StreamSetup->Transport->Protocol = tt__TransportProtocol__RTSP;
            if (multicast == true) {
                gsu.StreamSetup->Stream = tt__StreamType__RTP_Multicast;
            }
            else {
                gsu.StreamSetup->Stream = tt__StreamType__RTP_Unicast;
            }
            result = proxy.GetStreamUri(endpoint.c_str(), NULL, &gsu, gsur);
            if (result == SOAP_OK) {
                videoURLs.push_back(gsur.MediaUri->Uri);
            }
        }
    }
    else {
        soap_print_fault(&_soap, stderr);
    }

    return videoURLs;
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::updateWindowTitle()
{
    // format for main window title is
    // "<app-name> <camera name | camera IP> (high-res|low-res)"
    wstringstream title;
    title << &_appTitle[0];

    if (_options.isSourceSelected()) {
        UserOptions::SourceHistoryRecord curSource = _options.getCurrentSourceInfo();
        if (curSource._isCamera) {
            if (curSource._logicalName.empty()) {
                title << L"  -  " << curSource._id;
            }
            else {
                title << L"  -  " << curSource._logicalName
                      << L" (" << curSource._id << L")";
            }
            if (_options.getConnectAsNonOpteraType() == false) {
                title << L"  -  " << (curSource._highRes ? L"high resolution" :
                    L"low resolution");
            }
        }
        else {
            title << L" - " << curSource._id;
        }
    }

    if (_options.getPTZSituationalAwarenessMode() == true) {
        SituationControl::SAControlMode controlMode = SituationControl::SA_OFF;
        if (_situationControl.get()) {
            controlMode = _situationControl->getControlMode();
        }
        title << L"     PTZ Control Mode - ";
        switch (controlMode) {
            case SituationControl::SA_AUTO:
                title << L" Auto";
                break;
            case SituationControl::SA_FOLLOW_OBJECT:
                title << L" Follow Object";
				break;
			case SituationControl::SA_CALIBRATE_PTZ_PAN_OLD:
				title << L" Calibrate PTZ Pan (Old)";
				break;
			case SituationControl::SA_CALIBRATE_PTZ_PAN:
				title << L" Calibrate PTZ Pan";
				break;
			case SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION:
				title << L" Calibrate PTZ Pan & Position";
				break;
			case SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_BLIND:
				title << L" Calibrate PTZ Pan & Position (Blind)";
				break;
			case SituationControl::SA_CALIBRATE_PTZ_POSITION_CAMERA_TILTS:
				title << L" Calibrate PTZ Position & Camera Tilts";
				break;
			case SituationControl::SA_CALIBRATE_PTZ_PAN_POSITION_CAMERA_TILTS:
				title << L" Calibrate PTZ Pan, Position & Camera Tilts";
				break;
            case SituationControl::SA_POINT_LOCATION:
                title << L" Click To Point";
                break;
            case SituationControl::SA_ZOOM_TO_BOX:
                title << L" Zoom To Box";
                break;
            default:
                break;
        }
    }

    SetWindowText(_hWnd, title.str().c_str());
}

//////////////////////////////////////////////////////////////////////////////////
LRESULT PanomersiveViewer::OnUserMessage(UINT msgID, WPARAM wparam, LPARAM lparam)
{
    LRESULT retval = 0;
    if (msgID == WM_USER_INIT_STREAM) {
        size_t streamIndex = wparam;
        bool sitAwareStream = (lparam == 0) ? false : true;
        initStream(streamIndex, true, sitAwareStream);
    }
    else if (msgID == WM_USER_UPDATE_FRAME) {
        bool doRender = (wparam & 0xF0) != 0;
        size_t streamIndex = wparam & 0x0F;
        const unsigned char* pFrame = reinterpret_cast<unsigned char*>(lparam);
        commitStreamData(streamIndex, pFrame, doRender);
        _stats->incrementRenderedFrames(streamIndex);

#ifdef USE_DEMO_BITMAP
#ifdef USE_DEMO_DATETIME_BITMAP
        //if we update stats update our bitmap if required time has passed
        static time_t lastTime;
        time_t now;
        time(&now);
        double secs;

        secs = difftime(now, lastTime);
        if (secs > 1.0) {
            loadTestBitmap(_views.size() - 1);
            lastTime = now;
        }
#endif
#endif

        // clear the updating flag.  As long as this happens before the next
        // frame is ready to be displayed, then the GPU is keeping up.
        // Otherwise, we'll have to drop frames to prevent the UI from hanging.
        _streams[streamIndex]._updating = false;
    }
    else if (msgID == WM_USER_UPDATE_CAMERA_DLG_SELECTION) {
        OnCameraDlgSelection((HWND)lparam);
    }
    return retval;
}


//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::OnKeyUp(WPARAM virtualKeyCode, LPARAM /*repeatFlags*/)
{
    if (virtualKeyCode == VK_ESCAPE) {
        if (_options.getViewLayout() == UserOptions::VL_ALL_MONITORS) {
            changeViewLayout(UserOptions::VL_SINGLE);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::initStream(size_t streamIndex, bool canSyncCameraAngle, bool sitAwareStream)
{
    if (sitAwareStream == false) {
        // OpenGlGuard grd(_hDC, _hGLRC);
        wglMakeCurrent(_hDC, _hGLRC);
    }
    else {
        // OpenGlGuard grd(deviceContextOverall, _hSitAwarePtzGLRC);
        wglMakeCurrent(deviceContextOverall, _hSitAwarePtzGLRC);
    }

    StreamInfo& strm = _streams[streamIndex];

    unique_lock<mutex> lock(strm._initMutex);

    // create a new stream object
    try {
        if (sitAwareStream == false) {
            if (_options.getConnectAsNonOpteraType() == true)  {
                IMEXTK_CHECK(pelco_imextk_set_non_optera_cam_mode_setting(_imextkContext, true));
            }
            IMEXTK_CHECK(pelco_imextk_gl_create_video_stream(_imextkContext,
                &strm._id,
                strm._layoutMetadata.c_str(),
                strm._frameWidth, strm._frameHeight,
                _kFramePoolCapacity));
        } 
        else {
            IMEXTK_CHECK(pelco_imextk_set_non_optera_cam_mode_setting(_sitAwarePtzImextkContext, true));
            IMEXTK_CHECK(pelco_imextk_gl_create_video_stream(_sitAwarePtzImextkContext,
                &strm._id,
                strm._layoutMetadata.c_str(),
                strm._frameWidth, strm._frameHeight,
                _kFramePoolCapacity));
        }
        // fill the stream's associated fifo with buffer pointers
        for (size_t i = 0; i < _kFramePoolCapacity; ++i) {
            StreamInfo::Frame frame;
            IMEXTK_CHECK(pelco_imextk_gl_get_frame_buffer(strm._id,
                            &frame._yPlane, &frame._uPlane, &frame._vPlane));
            strm._bufferPool.write(frame);
        }

        // Now that at least one stream has been constructed with
        // camera-supplied layout info, it's ok to apply any override
        // (If we'd done that sooner, it might have been ovewritten here.)
        if (canSyncCameraAngle &&
                _options.getCurrentSourceInfo()._overrideCameraTiltAngle) {
            syncCameraAngle();
        }

        // As with camera angle, the context should now know whether the
        // camera is in corridor mode.  If it is, update the layout (which
        // was previously initialized assuming standard camera orientation)
        bool corridorMode;
        pelco_imextk_get_corridor_mode(_imextkContext, &corridorMode);
        if (corridorMode) {
            resetViewsAndLayouts();
        }

        if ((_options.getConnectAsNonOpteraType() == true) || (sitAwareStream == true)) {
            float facetspacing;
            //update layout dim based on stream dim
            if (strm._frameHeight > strm._frameWidth) {
                facetspacing = static_cast<float>(strm._frameWidth) / strm._frameHeight;
            }
            else {
                facetspacing = static_cast<float>(strm._frameHeight) / strm._frameWidth;
            }
            facetspacing = (1.0f - facetspacing) / 2.0f;

            if (strm._id != PelcoImextk_kInvalidVideoStream) {

                if (strm._frameHeight > strm._frameWidth) {

                    IMEXTK_CHECK(pelco_imextk_gl_update_layout_dest(strm._id, strm._layoutMetadata.c_str(), IMEXTK_ALL_FACETS, false, facetspacing, 1.0f - (2.0f*facetspacing), strm._frameWidth, strm._frameHeight));
                }
                else {
                    IMEXTK_CHECK(pelco_imextk_gl_update_layout_dest(strm._id, strm._layoutMetadata.c_str(), IMEXTK_ALL_FACETS, true, facetspacing, 1.0f - (2.0f*facetspacing), strm._frameWidth, strm._frameHeight));
                }
                this->updateViewLayouts();
            }
        }
    }
    catch (...) {
        // ugh. partially initialized stream.  clean up whatever
        // we managed to create so far before bailing.
        if (strm._id != PelcoImextk_kInvalidVideoStream) {
            // return all outstanding buffers
            while (!strm._bufferPool.willReadBlock()) {
                StreamInfo::Frame frame;
                strm._bufferPool.read(frame);
                pelco_imextk_gl_release_frame_buffer(strm._id,
                        frame._yPlane, false);
            }
            pelco_imextk_gl_delete_video_stream(strm._id);
            strm._id = PelcoImextk_kInvalidVideoStream;
        }
        strm._initComplete.notify_all();
        throw;
    }

    _stats->activateStream(streamIndex);
    strm._initComplete.notify_all();
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::commitStreamData(size_t streamIndex,
        const unsigned char* pFrame, bool doRender)
{
    try {
        if (streamIndex != 6) {
//            OpenGlGuard grd(_hDC, _hGLRC);
            wglMakeCurrent(_hDC, _hGLRC);
        }
        else {
//            OpenGlGuard grd(deviceContextOverall, _hSitAwarePtzGLRC);
            wglMakeCurrent(deviceContextOverall, _hSitAwarePtzGLRC);
        }
        StreamInfo& strm = _streams[streamIndex];

        bool bypassOpengl = _options.getRenderMode() == UserOptions::RM_RAW;

        {
            unique_lock<recursive_mutex> lk(_stoppingMutex);
            if (_streams[streamIndex]._stopping) {
                bypassOpengl = true;
            }
        }

        if (bypassOpengl) {
            // In "raw" mode, we don't combine frames into an immersive view.
            // Instead, we just draw the raw frame data. So instead of using
            // multiple VideoWindows to show different PTZ views of the same data,
            // we use them to show the different raw video streams from the camera.
            if (streamIndex < _views.size()) {
                // when streaming from a live source, the data is in I420 format.
                // Just show the Y (illuminance) plane rather than converting
                // to RGB.
                bool isYUV = _streamer != NULL;
                _views[streamIndex]->UpdateStreamImage(pFrame,
                        strm._frameWidth, strm._frameHeight,
                        isYUV);
            }
        }

        // return video frames to the stream object.
        IMEXTK_CHECK(pelco_imextk_gl_release_frame_buffer(strm._id,
                        pFrame,
                        !bypassOpengl)); // doUpdate

        // fetch another buffer to replenish the pool
        StreamInfo::Frame frameBufferReplacement;
        IMEXTK_CHECK(pelco_imextk_gl_get_frame_buffer(strm._id,
                        &frameBufferReplacement._yPlane,
                        &frameBufferReplacement._uPlane,
                        &frameBufferReplacement._vPlane));
        strm._bufferPool.write(frameBufferReplacement);

        // tell all views to repaint
        if ((doRender) && (!bypassOpengl)) {
            setHourglass(false); // clear the hour-glass
            /*
            if (_views.size() == 2) {
                _views[1]->notifyContentUpdated();
            }
            else {
                for (size_t i = 0; i < _views.size(); ++i) {
                    _views[i]->notifyContentUpdated();
                }
            }
            */
            for (size_t i = 0; i < _views.size(); ++i) {
                _views[i]->notifyContentUpdated();
            }
        }
    }
    catch (const std::exception& e) {
        wstringstream msg;
        msg << "Critical Error: " << e.what();
        showErrorPopup(NULL, L"Critical Error", msg.str());
        PostThreadMessage(_windowThreadId, WM_QUIT, 0, 0);
        PostQuitMessage(0);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::setCameraAngleInAllViews(float angleRadians)
{
    IMEXTK_CHECK(pelco_imextk_override_camera_tilt_angle(_imextkContext, angleRadians));
    for (size_t i = 0; i < _views.size(); ++i) {
        _views[i]->notifyContentUpdated();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::syncCameraAngle()
{
    if (_options.isSourceSelected()) {
        syncCameraAngle(_options.getCurrentSourceInfo()._overrideCameraTiltAngle,
                _options.getCurrentSourceInfo()._cameraTiltAngle);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::syncCameraAngle(bool overrideCamera, float angle)
{
    if (overrideCamera) {
        IMEXTK_CHECK(pelco_imextk_override_camera_tilt_angle(_imextkContext, angle));
    }
    else {
        OpenGlGuard grd(_hDC, _hGLRC);
        for (size_t i = 0; i < _streams.size(); ++i) {
            const StreamInfo& strm = _streams[i];
            if (strm._id != PelcoImextk_kInvalidVideoStream) {
                IMEXTK_CHECK(pelco_imextk_gl_update_stream_layout(strm._id,
                                strm._layoutMetadata.c_str(),
                                strm._frameWidth, strm._frameHeight));
            }
        }
    }
    for (size_t i = 0; i < _views.size(); ++i) {
        _views[i]->notifyContentUpdated();
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool PanomersiveViewer::getNonOpteraCamModeSetting() const
{
    bool nonOpteraSetting = false;
    IMEXTK_CHECK(pelco_imextk_get_non_optera_cam_mode_setting(_imextkContext, &nonOpteraSetting));
    return nonOpteraSetting;
}

void PanomersiveViewer::updatePTZEndpointInAllViews(std::string ipaddr)
{
	std::string endpoint, token;

	// determine if ptz availible (i.e. endpoint will not return empty if we can ptz)
	determinePTZEndpoint(ipaddr, endpoint, token);

	for (size_t i = 0; i < _views.size(); ++i) {
		_views[i]->setPTZEndpoint(endpoint, token);
	}
}

void PanomersiveViewer::updatePTZEndpointForEachView(std::string ipaddr, std::string sitAwareIP)
{
	std::string endpoint, token;

	// determine if ptz availible (i.e. endpoint will not return empty if we can ptz)

	// determine and save 1st view endpoint/token
	determinePTZEndpoint(ipaddr, endpoint, token);
	_views[0]->setPTZEndpoint(endpoint, token);

	// save 2nd view endpoint/token
	determinePTZEndpoint(sitAwareIP, endpoint, token);
	_views[1]->setPTZEndpoint(endpoint, token);
}

string PanomersiveViewer::getPTZEndpoint(void)
{
    return _ptzServiceEndpoint;
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::syncViewLayoutPanoModId()
{
    MENUITEMINFO info = {0};
    info.cbSize = sizeof(info);
    DWORD err = 0;

    info.fMask = MIIM_SUBMENU;
    if (!GetMenuItemInfo(_hMenu, 1, TRUE, &info)) {
        err = GetLastError();
    }

    info.fMask = MIIM_STATE;
    if (!GetMenuItemInfo(info.hSubMenu, 0, TRUE, &info)) {
        err = GetLastError();
    }
    bool shouldBeChecked = _options.getViewLayoutPanoMod() == UserOptions::VLPM_PELCO270_OPTIMIZED;
    bool isChecked = (info.fState & MFS_CHECKED) != 0;
    if (shouldBeChecked != isChecked) {
        CheckMenuItem(_hMenu, IDM_VIEW_MOD_270_OPTIMIZED, MF_BYCOMMAND | (shouldBeChecked ? MF_CHECKED : NULL));
        resetViewsAndLayouts();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::interactiveLoadFixedViewDir()
{
    UserOptions::SourceHistoryRecord source;
    if (!_options.findLatestFixedViewDir(source)) {
        // no previously viewed directory.  Use current directory.
        wchar_t curDir[MAX_PATH + 1];
        GetCurrentDirectory(sizeof(curDir)/sizeof(wchar_t), curDir);

        source._isCamera = false;
        source._id = wstring(curDir, curDir + wcslen(curDir));
    }

    BROWSEINFO bi = { 0 };
    bi.hwndOwner = _hWnd;
    bi.lpszTitle = L"Specify a directory containing negx, negy, posz, etc. image files";
    bi.ulFlags = BIF_NEWDIALOGSTYLE;
    bi.lpfn = cbBrowseFolderProc;
    bi.lParam = (LPARAM)&source._id;

    PIDLIST_ABSOLUTE result = SHBrowseForFolder(&bi);
    if (result != 0) {
        wchar_t buf[MAX_PATH + 1];
        buf[0] = '\0';
        SHGetPathFromIDList(result, buf);
        CoTaskMemFree(result);
        loadFixedViewDir(wstring(buf, buf + wcslen(buf)));
    }
}

//////////////////////////////////////////////////////////////////////////////////
int CALLBACK PanomersiveViewer::cbBrowseFolderProc(HWND hWnd, UINT uMsg,
        LPARAM lParam, LPARAM lpData)
{
    if (uMsg == BFFM_INITIALIZED) {
        SendMessage(hWnd, BFFM_SETSELECTION, TRUE, (LPARAM)((wstring*)lpData)->c_str());
    }

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::loadFixedViewDir(const wstring& dir)
{
    try {
        resetSourcesViewsAndLayouts(false);
    }
    catch (std::exception& e) {
        showErrorPopup(NULL, L"Error loading fixed view directory",
                wstring(e.what(), e.what() + strlen(e.what())));
        return;
    }

    vector<StreamSetupInfo> streamSetups = {
        { L"negx.jpg", L"negx_layout.txt" },
        { L"posx.jpg", L"posx_layout.txt" },
        { L"negy.jpg", L"negy_layout.txt" },
        { L"posy.jpg", L"posy_layout.txt" },
        { L"negz.jpg", L"negz_layout.txt" },
        { L"posz.jpg", L"posz_layout.txt" }
    };

    vector<StreamSetupInfo> mosaicStreamSetups = {
        { L"mosaic.jpg", L"mosaic_layout.txt" }
    };

    try {
        loadFixedViewDirHelper(dir, streamSetups);
    } catch (...) {
        // try again with mosaic format
        try {
            loadFixedViewDirHelper(dir, mosaicStreamSetups);
        } catch (std::exception& e) {
            showErrorPopup(NULL, L"Error loading fixed view directory",
                    wstring(e.what(), e.what() + strlen(e.what())));
        }
    }

    if (!_options.selectFixedViewDir(dir)) {
        _options.addFixedViewDir(dir, dir);
    }

    // camera angle override is a user-options feature, and since we
    // don't select/add the source dir until it's successfully loaded,
    // we delay this part of unitialization until now.
    if (_options.getCurrentSourceInfo()._overrideCameraTiltAngle) {
        syncCameraAngle();
    }
    syncViewLayoutPanoModId();
    updateWindowTitle();
}


//////////////////////////////////////////////////////////////////////////////////
void PanomersiveViewer::loadFixedViewDirHelper(const wstring& dir,
        const vector<StreamSetupInfo>& streamSetups)
{
    wstring separator(L"\\");

    for (size_t i = 0; i < streamSetups.size(); ++i) {
        //
        // use Gdi+ to load the image file
        //
        wstring filename = dir + separator + streamSetups[i]._facetFilename;
        Gdiplus::Bitmap* img = Gdiplus::Bitmap::FromFile(filename.c_str());
        assert(img != NULL); // it's never supposed to return NULL
        if ((img->GetWidth() > 0) && (img->GetHeight() > 0)) {
            //
            // Read the whole layout info file into a string
            //
            filename = dir + separator + streamSetups[i]._layoutInfoFilename;
            fstream layoutStrm(filename);
            if (!layoutStrm.good()) {
                throw runtime_error("unable to load layout info file");
            }
            layoutStrm.seekg(0, layoutStrm.end);
            size_t fileSize = (size_t)layoutStrm.tellg();
            layoutStrm.seekg(0, layoutStrm.beg);
            vector<char> layoutDescription(fileSize + 1, '\0');
            layoutStrm.read(&layoutDescription[0], fileSize);
            _streams[i]._layoutMetadata = string(&layoutDescription[0]);
            _streams[i]._frameWidth = img->GetWidth();
            _streams[i]._frameHeight = img->GetHeight();
            initStream(i, false);

            StreamInfo::Frame frame;
            _streams[i]._bufferPool.read(frame);
            unsigned char* pBuffer = frame._yPlane;

            Gdiplus::BitmapData data;
            data.Scan0 = (void*)pBuffer;
            data.Width = img->GetWidth();
            data.Height = img->GetHeight();
            data.PixelFormat = PixelFormat32bppARGB;
            data.Stride = 4 * data.Width;
            Gdiplus::Rect rect(0,0, img->GetWidth(), img->GetHeight());
            img->LockBits(&rect, Gdiplus::ImageLockModeRead | Gdiplus::ImageLockModeUserInputBuf,
                    PixelFormat32bppARGB, &data);

            img->UnlockBits(&data);
            delete img;

            commitStreamData(i, pBuffer,
                    // only render after the last frame is copied
                    (i == (streamSetups.size() - 1)));
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////
INT_PTR CALLBACK PanomersiveViewer::cbAboutDlg(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
        case WM_INITDIALOG:
            return (INT_PTR)TRUE;

        case WM_COMMAND:
            if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
            {
                EndDialog(hDlg, LOWORD(wParam));
                return (INT_PTR)TRUE;
            }
            break;
    }
    return (INT_PTR)FALSE;
}

#ifdef USE_DEMO_BITMAP
#ifdef USE_DEMO_DATETIME_BITMAP
void PanomersiveViewer::loadTestBitmap(int viewIndex) {
    if (viewIndex >= (int)_views.size()) {
        return;
    }

    time_t rawtime;
    time(&rawtime);
    char timebuf[40];
    ctime_s(timebuf, 40, &rawtime);
    string demoText(timebuf);   
    demoText.clear();
    if (_streamer != NULL) {
        if (_streamer->hasUserMetadataText() == true)
        {
            demoText = _streamer->userMetadataText();
        }
        else {
            demoText.clear();
        }
    }

    std::wstring stemp = std::wstring(demoText.begin(), demoText.end());
    LPCWSTR sw = stemp.c_str();

    HDC hTextDC = CreateCompatibleDC(_hDC);
    if (hTextDC == NULL) {
        return;
    }
    HFONT hOldFont = (HFONT)SelectObject(hTextDC, _testFont);
    if ((hOldFont == NULL) || (hOldFont == HGDI_ERROR)) {
        return;
    }
    HBITMAP hMyDIB = NULL;
    COLORREF color = 0x00FFFFFF;
    BYTE modAlpha = 0xFF;

    //Get text area
    RECT TextArea = { 0, 0, 0, 0 };
    DrawText(hTextDC, sw, demoText.length(), &TextArea, DT_CALCRECT);
    if ((TextArea.right > TextArea.left) && (TextArea.bottom > TextArea.top)) {
        BITMAPINFOHEADER BMIH;
        memset(&BMIH, 0x0, sizeof(BITMAPINFOHEADER));
        void *pvBits = NULL;
        // Specify DIB setup JUST THE SIZE OF THE TEXT bounding-box...
        BMIH.biSize = sizeof(BMIH);
        BMIH.biWidth = TextArea.right - TextArea.left;
        BMIH.biHeight = (TextArea.bottom - TextArea.top); // ?? flip vertical so we are right-side-up for GE SDK
        BMIH.biPlanes = 1;
        BMIH.biBitCount = 32;
        BMIH.biCompression = BI_RGB;
        size_t overlayWidth = BMIH.biWidth;
        size_t overlayHeight = BMIH.biHeight;
        std::vector<unsigned char> testOverlayImage(overlayWidth * overlayHeight * 4);
        // Create and select DIB into DC
        hMyDIB = CreateDIBSection(hTextDC, (LPBITMAPINFO)&BMIH, 0, (LPVOID*)&pvBits, NULL, 0);

        if ((hMyDIB == NULL) && (pvBits == NULL)) {
            return;
        }
        HBITMAP hOldBMP = (HBITMAP)SelectObject(hTextDC, hMyDIB);
        if (hOldBMP != NULL)
        {
            // Set up DC properties
            SetTextColor(hTextDC, 0x00FFFFFF);
            SetBkColor(hTextDC, 0x00000000);
            SetBkMode(hTextDC, OPAQUE); // set the background to its color (black)
            // Draw text to buffer
            DrawText(hTextDC, sw, demoText.length(), &TextArea, DT_NOCLIP);
            // Change the location of the 'alpha' so format is correct for OncamGE API
            BYTE* DataPtr = (BYTE*)pvBits;
            BYTE FillR = GetRValue(color);
            BYTE FillG = GetGValue(color);
            BYTE FillB = GetBValue(color);
            BYTE ThisA;
            int rowWidth = BMIH.biWidth;
            int rowHeight = BMIH.biHeight;
            int rowOffset = 0;
            int rowBytes = BMIH.biWidth * 4; // 32 bits
            int destRowBytes = rowWidth * 4;
            int destRowOffset = 0 * destRowBytes;// take us to the row we want to write at..
            int destColOffset = 0 * 4;// take us to the COlumn we want to write at..

            for (int LoopY = 0; LoopY < BMIH.biHeight; LoopY++) {
                rowOffset = (BMIH.biHeight - LoopY - 1)*rowBytes;
                for (int LoopX = 0; LoopX < BMIH.biWidth; LoopX++) {
                    ThisA = DataPtr[rowOffset + (LoopX * 4)];
                    // Move alpha and pre-multiply it with the R,G,B aspects of the color we want:
                    int pixOffset = destRowOffset + (LoopX * 4);
                    testOverlayImage[pixOffset] = FillB; // Blue
                    testOverlayImage[pixOffset + 1] = FillG; // Green
                    testOverlayImage[pixOffset + 2] = FillR; // Red
                    testOverlayImage[pixOffset + 3] = ThisA*modAlpha >> 8; // Alpha
                }
                destRowOffset += destRowBytes;
            }

            if (viewIndex < 0) {
                //do all views
                for (unsigned int i = 0; i < _views.size(); i++) {
                    _views[i]->addBitmapToOverlay(0, 0, overlayWidth, overlayHeight, &testOverlayImage[0]);
                }
            }
            else {
                _views[viewIndex]->addBitmapToOverlay(0, 0, overlayWidth, overlayHeight, &testOverlayImage[0]);
            }

            // De-select bitmap from the hTextDC
            SelectObject(hTextDC, hOldBMP);
            // Delete the BITMAP, we've already altered our overlay bits
            DeleteObject(hMyDIB);

        }
    }
}
#else
void PanomersiveViewer::loadTestBitmap(int viewIndex) {
    if (viewIndex >= (int)_views.size()) {
        return;
    }

    unsigned int overlayWidth = 200;
    unsigned int overlayHeight = 150;

    /*
    //add this to use a resized demo bitmap based on view size
    if (viewIndex >= 0) {
        overlayWidth = _views[viewIndex]->clientWidth()/5;
        overlayHeight = _views[viewIndex]->clientHeight()/5;
    }
    */
    std::vector<unsigned char> testOverlayImage(overlayWidth * overlayHeight * 4);
    for (size_t row = 0; row < overlayHeight; row++) {
        for (size_t col = 0; col < overlayWidth; col++) {
            size_t pix = ((row * overlayWidth) + col) * 4;
            testOverlayImage[pix] = ((col / 20) % 2) * 0xFF; //Blue
            testOverlayImage[pix + 1] = 0; // Green
            testOverlayImage[pix + 2] = ((row / 20) % 2) * 0xFF; // Red
            testOverlayImage[pix + 3] = 2 * row; //Alpha
        }
    }

    if (viewIndex < 0) {
        //do all views
        for (unsigned int i = 0; i < _views.size(); i++) {
            _views[i]->addBitmapToOverlay(0, 0, overlayWidth, overlayHeight, &testOverlayImage[0]);
        }
    }
    else {
        _views[viewIndex]->addBitmapToOverlay(0, 0, overlayWidth, overlayHeight, &testOverlayImage[0]);
    }
}
#endif
#endif
