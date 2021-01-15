//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_VIDEO_WINDOW_HPP__
#define __PELCO_IMEXTK_VIDEO_WINDOW_HPP__

#include <pelco/imextk/ViewGenerator.h>
#include <pelco/imextk/VideoStream.h>
#include "UserOptions.hpp"
#include "Window.hpp"
#include "MultiThreadFifo.hpp"
#include "PerformanceStats.hpp"
#include "SituationControl.hpp"
#include <vector>
#include <memory>

//
// Manages an independent opengl view area within larger window.
//
class VideoWindow : public Window {
    public:
        VideoWindow(const VideoWindow&) = delete;
        VideoWindow& operator=(const VideoWindow&) = delete;

        // Callback for indicating when a position in this view should be
        // tracked in other views.

        enum TrackingEvent {
            TRACK,
            END_TRACKING,
            POINT_OF_INTEREST, // used for e.g., "swarming"
        };

        typedef std::function<void (TrackingEvent evt,
            float panAngleRadians, float tiltAngleRadians,
            float zoomFactor)> PositionTrackingCallback;

        VideoWindow(PelcoImextk_ViewType vt,
                HGLRC streamsGlContext,
                PelcoImextk_Context imextkContext,
                HINSTANCE hInstance, HWND hParentWnd, const RECT& area,
                bool separateWindow, // true == popup, false == child
                const std::wstring& windowTitle,
                PositionTrackingCallback cb,
                UserOptions::RenderMode mode,
                std::shared_ptr<PerformanceStats> stats, struct soap* psoap,
                std::shared_ptr<SituationControl> sitControl);
        virtual ~VideoWindow();

        // Only used in RM_RAW mode.
        void UpdateStreamImage(const unsigned char* pFrame,
                size_t frameWidth, size_t frameHeight,
                bool isYUV);

        // In modes other than RM_RAW, this is called whenever the content has
        // been updated
        void notifyContentUpdated();

        // (called by background thread)
        void notifyDroppedFrame();

        PelcoImextk_ViewType getViewType() const { return _viewType; }

        //
        // Position tracking.  When the user selects a track point in
        // this view, the PositionTrackingCallback is called with the
        // spherical coordinates of the selected point.
        // The recipient can then call trackPosition on this and other
        // views, so each view can draw an indicator of the tracked point.
        void trackPosition(bool trackingOn, float panRadians, float tiltRadians);

        void transPositionToMosaic(const POINTS& pt, POINTS& transPT);
        void transSphericalToMosaic(float panRadians, float tiltRadians, int& xPos, int& yPos);

        void centerOnMouse();

        void rotateTo(float panRadians, float tiltRadians,
                bool autoZoom); // zooms in if PTZ limits otherwise prevent pan/tilt
        void rotateToDataCenter();

        virtual void move(const RECT& r);

        UserOptions::PtzLimitOptions getPtzLimitOptions() const;
        void setPtzLimitOptions(const UserOptions::PtzLimitOptions& opts);

        // Frame-count-overlay mode: the current frame count is displayed
        // over the image in the upper-left corner
        bool getFrameCountOverlayMode() const { return _frameCountOverlayMode; }
        void setFrameCountOverlayMode(bool overlayOn);
        bool getBitrateOverlayMode() const { return _bitrateOverlayMode; }
        void setBitrateOverlayMode(bool overlayOn);

        // So bitrate knows where frameCount overlay stops
        const unsigned int _frameCountOverlayHeight = 180;


        void addBitmapToOverlay(unsigned int xoffset, unsigned int yoffset, unsigned int pixelWidth,
            unsigned int pixelHeight, const unsigned char * bmpData);
        void setLetterboxingColor(unsigned char red, unsigned char green, unsigned char blue);
        void getLetterboxingColor(unsigned char& red, unsigned char& green, unsigned char &blue);

        inline PelcoImextk_ViewGen* getViewGenerator(void) { return &_viewGenerator; };

        void setPTZEndpoint(std::string ptzEndpoint, std::string ptzToken);

        void clearTrackedPositionMarker(void);
        void clearDataBoundsMarkers(void);

		void drawCalibrationLine(void);
		void clearCalibrationLine(void);
		void drawCalibrationMark(void);
		void clearCalibrationMark(void);
        void handleSituationAwarenessMode(SituationControl::SAControlMode mode);

    protected:
        virtual void OnPaint(const PAINTSTRUCT& ps);
        virtual bool EraseBackground();
        virtual void OnResize(int width, int height);
        virtual void OnLMouseDown(const POINTS& pt);
        virtual void OnLMouseUp(const POINTS& pt);
        virtual void OnLMouseDblClk(const POINTS& pt);
        virtual void OnMMouseDown(const POINTS& pt);
        virtual void OnMMouseUp(const POINTS& pt);
        virtual void OnRMouseDown(const POINTS& pt);
        virtual void OnRMouseUp(const POINTS& pt);
        virtual void OnMouseMove(const POINTS& pt);
        virtual void OnMouseWheel(short wheelValue, const POINTS& pt);
        virtual void OnKeyUp(WPARAM virtualKeyCode, LPARAM repeatFlags);
        virtual LRESULT OnUserMessage(UINT msgID, WPARAM wparam, LPARAM lparam);

    private:
        void cleanup();

        void emitPositionTrackingEvent(const POINTS& pt, TrackingEvent evt);
		void requestSwarmToMouse();
		void drawPositionTrackingText(bool undraw);
		void drawCalibrationText(bool undraw);
        void drawDataBounds();
        void drawPerformanceStats(bool undraw);
        void drawBitrateStats(bool undraw);
        void drawNewView();

		int doOnvifGetStatus(float &pan, float &tilt, float &zoom);
		int doOnvifPanTiltCont(float pan, float tilt);
		int doOnvifPanTiltRel(float deltaPanNorm, float deltaTiltNorm, float zoomNorm, float speedNorm);
		int doOnvifPanTiltAbs(float panNorm, float tiltNorm, float zoomNorm, float speedNorm);
        int doOnvifZoom(float zoomVel);
        std::pair<float, float> pointToPanTiltPair(const POINTS& pt);
        bool isNonOpteraCam(void);
        void doDigitalZoomPT(const POINTS& point, float deltaZ);
        void drawBoxForZoom(const POINTS& pt);
        void clearBoxForZoom(void);

        inline bool onvifPTZable(void)
        {
            if (_ptzEndpoint.size() != 0) { return true; }            
            return false;
        }

        const PelcoImextk_ViewType _viewType;

        // keep track of our logical parent, even if it's not our actual
        // parent (as is the case in full-screen mode)
        HWND _hParent;

        PelcoImextk_Context _imextkContext;
        PelcoImextk_ViewGen _viewGenerator;
        HGLRC _hGLRC;

        bool _panTilting;
        bool _zooming;
        POINTS _ptzAnchor;

        bool _isTrackingPosition;
        bool _isControllingPositionTracking;
        float _trackedPanAngleRadians;
        float _trackedTiltAngleRadians;
        float _trackedZoomFactor;
        int _trackedRawMosaicPosX;
        int _trackedRawMosaicPosY;

        bool _isShowingDataBounds;
        bool _moveMouseAfterRotateTo;

        bool _frameCountOverlayMode;
        bool _bitrateOverlayMode;
        const UserOptions::RenderMode _renderMode;

		static float _rawPtzPanNorm;
		static float _rawPtzTiltNorm;
		static float _rawPtzZoomNorm;

        // data used to control offscreen buffers used by RM_RAW or RM_FRAMEBUFFER modes
        struct Offscreen {
                HWND _hWnd;
                HDC _hDC;
                unsigned int _width;
                unsigned int _height;
                bool _isYUV;
                HBITMAP _bmp;
                uint8_t* _bmpBits;
        };

        struct OffscreenRect : public Offscreen {
            int   _xAnchor;
            int   _yAnchor;
            int   _xPos;
            int   _yPos;
            int   _clientWidth;
            int   _clientHeight;
            bool  _drawing;
        };

        static void drawMarker(Offscreen& img, unsigned int width, unsigned int height,
                COLORREF clr);
        static void drawRectangle(Offscreen& img, unsigned int width, unsigned int height,
            COLORREF clr);
        static void resizeOffscreenStorage(Offscreen& img,
                unsigned int width, unsigned int height, bool isYUV);
        static void cleanupOffscreenStorage(Offscreen& img);
        void drawDataBoundsMarker(Offscreen& img, float panAngle, float tiltAngle,
                float centerPan, float centerTilt);

        Offscreen _offscreen;

        static Offscreen _noMarker; // hides previously shown marker
        static Offscreen _redMarker;
        static Offscreen _greenMarker;
        static Offscreen _blueMarker;
        static Offscreen _darkRedMarker;
        static Offscreen _darkBlueMarker;

		Offscreen _positionTrackingTextImg;
		Offscreen _calibrationTextImg;
        Offscreen _statsTextImg;
        Offscreen _bitrateGraphImg;

        // for rendering to off-screen DC
        HDC _glRenderDC;

        PositionTrackingCallback _positionTracker;
        std::string _ptzEndpoint;
        std::string _ptzToken;
        struct soap* _pSoap;
        HANDLE _hZTimer;
        LARGE_INTEGER _liDueTime;
        LARGE_INTEGER _liDueTimeForClearTimer;
        void resetAndCancelTimer(HANDLE& handle);

        std::shared_ptr<PerformanceStats> _stats;
        const int _statsViewId;

        static int _instanceCount;
		std::shared_ptr<SituationControl> _sitControl;
		bool _isShowingCalLine;
		bool _isShowingCalMark;
        Offscreen _noCalLine; // hides previously shown calibration line
        Offscreen _blueCalLine;
        float _calPanAngleRadians;
        float _calTiltAngleRadians;
        static OffscreenRect _zoomMarker;
		bool  _drawZoom;
		POINT _calMark;
		POINT _lastCalMark;

        // This is used to change the font to display the bit rate overaly
        LOGFONT _logFont;
        HFONT   _hfont;
};

#endif // __PELCO_IMEXTK_VIDEO_WINDOW_HPP__
