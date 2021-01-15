//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_DEMO_HPP__
#define __PELCO_IMEXTK_DEMO_HPP__

#include <pelco/imextk/Context.h>
#include "wgl.hpp"
#include "Window.hpp"
#include "VideoWindow.hpp"
#include "SplitterLayout.hpp"
#include <BaseFrameSource.hpp>
#include "UserOptions.hpp"
#include "PerformanceStats.hpp"
#include "SituationControl.hpp"
#include <mutex>
#include <memory>

#define USE_DEMO_BITMAP
#define USE_DEMO_DATETIME_BITMAP

#ifdef USE_DEMO_BITMAP
#ifdef USE_DEMO_DATETIME_BITMAP
#include <windows.h>
#include <time.h>
#endif
#endif

class PanomersiveViewer : public Window {
    public:
        PanomersiveViewer(const PanomersiveViewer&) = delete;
        PanomersiveViewer& operator=(const PanomersiveViewer&) = delete;

        PanomersiveViewer(HINSTANCE hInstance);
        virtual ~PanomersiveViewer();

    protected:
        virtual bool OnMenuCommand(int menuID);
        virtual void OnResize(int width, int height);
        virtual bool OnCloseRequest();
        virtual LRESULT OnUserMessage(UINT msgID,
                WPARAM wparam, LPARAM lparam);
        virtual void OnKeyUp(WPARAM virtualKeyCode, LPARAM repeatFlags);

        // override to add extended information from current _imextkContext
        virtual void aboutToShowErrorPopup();

    private:
        static const wchar_t* loadTitleString(HINSTANCE hInstance);
        static HMENU loadMainMenu(HINSTANCE hInstance);
        void setMenusForCameraType(bool connectAsNonOpteraType, bool useSitAwareness);

        
        static INT_PTR CALLBACK cbGetEncoderChannelDlg(HWND hDlg, UINT message,
            WPARAM wParam, LPARAM lParam);

        static INT_PTR CALLBACK cbAskForCameraIPDlg(HWND hDlg, UINT message,
                WPARAM wParam, LPARAM lParam);
        // helpers for cbAskForCameraIPDlg
        void InitCameraDlg(HWND hDlg);
        void OnCameraDlgSelection(HWND hDlg);
        bool OnCameraDlgOk(HWND hDlg);
		bool OnDebugDlgOk(HWND hDlg);

        static INT_PTR CALLBACK cbAdjustCameraTiltDlg(HWND hWnd, UINT uMsg,
		        WPARAM wParam, LPARAM lpData);
		static INT_PTR CALLBACK cbDebugDlg(HWND hWnd, UINT uMsg,
                WPARAM wParam, LPARAM lpData);
        static INT_PTR CALLBACK cbAboutDlg(HWND hDlg, UINT message,
                WPARAM wParam, LPARAM lParam);


        void initViewsAndLayouts();
        void closeViewsAndLayouts();

        // helpers for closeViewsAndLayouts
        void deleteLayouts(WindowArea* pLayout);
        void deleteViews();

        // close everything
        void closeSourcesViewsAndLayouts();

        // re-create current views and layouts - used when
        // changing layouts or render options
        void resetViewsAndLayouts();

        // re-create evertying - used when changing video sources
        void resetSourcesViewsAndLayouts(bool sourceIsYUV);

        void updateViewLayouts();
        size_t getNumMonitors() const;
        size_t getPrimaryMonitorIndex() const;
        RECT getMonitorArea(size_t monitorIndex) const;
        void interactiveLoadFixedViewDir();
        void loadFixedViewDir(const std::wstring& directoryPath);
        static int CALLBACK cbBrowseFolderProc(HWND hWnd, UINT uMsg,
                LPARAM lParam, LPARAM lpData);

        struct StreamSetupInfo {
                std::wstring _facetFilename;
                std::wstring _layoutInfoFilename;
        };
        void loadFixedViewDirHelper(const std::wstring& directoryPath,
                const std::vector<StreamSetupInfo>& streamSetups);
        void viewCamera(const std::wstring& ip, const std::wstring& logicalName,
                        bool highRes, bool connectAsNonOpteraType, bool uniStream, bool primaryStream, bool multicast,
						bool useSitAwareness, const std::wstring& sitAwarePTZAddress, bool separatedCameras, const std::wstring& ptzHeight,
						const std::wstring& opteraHeight, const std::wstring& distanceToOptera, const std::wstring& minMoveDistance,
						const std::wstring& ptzTiltX, const std::wstring& ptzTiltZ, const std::wstring& ptzConeError, 
						const std::wstring& opteraTiltX, const std::wstring& opteraTiltZ, const std::wstring& opteraConeError);
        void setCameraAngleInAllViews(float angleRadians);
        void syncCameraAngle();
        void syncCameraAngle(bool overrideCamera, float angle);
        void syncViewLayoutPanoModId(void);
        void syncViewLayout(void);

        bool getNonOpteraCamModeSetting() const;

		void updatePTZEndpointInAllViews(std::string ipaddr);
		void updatePTZEndpointForEachView(std::string ipaddr1, std::string ipaddr2);

        std::string getPTZEndpoint();

        void onPositionTrackingEvent(VideoWindow::TrackingEvent evt,
                float panAngleRadians, float tiltAngleRadians, float zoomFactor);

        // returns stream url for non optera camera based on primary stream selection
        // if selection is secondary but not available primaryStream will be updated to false
        // to reflect that.
        std::vector<std::string> findNonOpteraStreamURL(std::wstring ip, bool& primaryStream, bool multicast);
        void determineIfNonOpteraTypeCameraAndNet55(std::string ipaddr, bool& connectAsNonOpteraType, int& numberChannels);
		//std::string determinePTZEndpoint(std::string ipaddr);
		void determinePTZEndpoint(std::string ipaddr, std::string& endpoint, std::string& token);

#ifdef USE_DEMO_BITMAP
        //FOR TEST ONLY
        void loadTestBitmap(int viewIndex);
#endif

        HGLRC _hGLRC;
        PelcoImextk_Context _imextkContext;

        // For situational awareness overPtzView
        HGLRC _hSitAwarePtzGLRC;
        PelcoImextk_Context _sitAwarePtzImextkContext;

        UserOptions _options;

        static const size_t _kFramePoolCapacity;

        // Per-stream information.
        struct StreamInfo {
                StreamInfo() :
                    _stopping(false),
                    _id(PelcoImextk_kInvalidVideoStream),
                    _bufferPool(int(_kFramePoolCapacity)),
                    _updating(false),
                    _usingPTZSitAware(false) {
                }
                bool _stopping;

                PelcoImextk_VideoStream _id;
                std::string _layoutMetadata;
                size_t _frameWidth; // N/A unless _id is valid
                size_t _frameHeight;

                struct Frame {
                        // If imextk context is configured for BGRA mode,
                        // only the _yPlane pointer will be used, and it will
                        // point to the sinele BGRA plane.
                        unsigned char* _yPlane;
                        unsigned char* _uPlane;
                        unsigned char* _vPlane;
                };

                // Each stream has its own FIFO of frame buffers.
                // These pointers come from imextk, and allow us
                // to decode directly into opengl memory.
                // We need a thread-safe FIFO here since VLC
                // decodes on background threads, and all imextk
                // calls to get and return buffer pointers
                // are made on the main, UI thread.
                MultiThreadFifo<Frame> _bufferPool;

                // allows setup callback to block until UI thread
                // finishes initializing the stream (either successfully or not)
                std::mutex _initMutex;
                std::condition_variable _initComplete;

                //
                // Variables used to work-around slow GPUs:
                //
                // _updating indicates whether the last frame returned
                // from VLC is still waiting to be copied to the GPU.
                volatile bool _updating;

                //
                //  There may be a seperate PTZ camera used for situational
                //   awareness.  This camera will needs its own stream and contexts
                //
                bool _usingPTZSitAware;
        };

        std::vector<StreamInfo> _streams;
        std::recursive_mutex _stoppingMutex;

        // called on the UI thread to initialize a StreamInfo object,
        // and fill its buffer pool
        void initStream(size_t streamIndex, bool canSyncCameraAngle, bool sitAwareStream = false);

        void commitStreamData(size_t streamIndex,
                const unsigned char* pFrame, bool doRender);

        void updateWindowTitle();
        void changeViewLayout(UserOptions::ViewLayoutId v);
        void changeRenderMode(UserOptions::RenderMode mode);
        void changeDecodeMode(UserOptions::DecodeMode mode);
        void changeRenderOption(PelcoImextk_StreamOptimizedType type);
        void changePtzLimitMode(PelcoImextk_PtzLimitMode mode);
        void toggleAutoZoomInWithPanTilt();
        void toggleAutoZoomOutWithPanTilt();
        void toggleAutoPanTiltWithZoomOut();
        void toggleViewLayoutPanoMod(void);

        void syncPtzLimitOptions();
        void setPtzLimitOptionsInAllViews(const UserOptions::PtzLimitOptions& opts);

        std::string ws2s(const std::wstring& s);

        std::vector<VideoWindow*> _views;

        std::vector<SplitterLayout*> _layouts;

        BaseFrameSource * _streamer;
        BaseFrameSource * _streamerSitAware;
        int _frameCount;

        ULONG_PTR _hGdiplus;
        static std::vector<wchar_t> _appTitle;

        WindowArea* _viewLayout;

        std::shared_ptr<PerformanceStats> _stats;

        MENUITEMINFO _viewPopUpSaved;

        struct soap _soap;
        std::string _ptzServiceEndpoint;
        std::string _camip;
        std::string _PTZToken;
        std::shared_ptr <SituationTracking> _situationTracking;
        std::shared_ptr <SituationControl> _situationControl;

        void HandleSituationalAwareChange(int menuID);

        // used to communicate to combo box on dialog init
        int _numberOfEncoderChannels;
        int _encoderChannelSelected;
        bool _selectingEncoderChannel;


#ifdef USE_DEMO_BITMAP
#ifdef USE_DEMO_DATETIME_BITMAP
        HFONT _testFont;
#endif
#endif
};

#endif // __PELCO_IMEXTK_DEMO_HPP__
