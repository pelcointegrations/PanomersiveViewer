//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_USER_OPTIONS_HPP__
#define __PELCO_IMEXTK_USER_OPTIONS_HPP__

#include <string>
#include <map>
#include <pelco/imextk/ViewGenerator.h>
#include <functional>
#include <list>
#define _USE_MATH_DEFINES
#include <math.h>

class UserOptions {
    public:
        UserOptions(const UserOptions&) = delete;
        UserOptions& operator=(const UserOptions&) = delete;

        // constructor loads from persistent storage
        UserOptions();
        virtual ~UserOptions();

        //**********************************************************************
        // Global, persistent settings
        //**********************************************************************

        // options for how to render graphics
        enum RenderMode {
            // draw raw frames directly (not using ViewGenerator at all)
            RM_RAW,

            // ViewGenerator draws directly to the window
            RM_DIRECT,

            // ViewGenerator draws to an offscreen framebuffer instead of the window
            RM_FRAMEBUFFER
        };

        RenderMode getRenderMode() const;
        bool setRenderMode(const RenderMode& mode);

        // Options on which decoder you want (VLC or live555 for example)
        enum DecodeMode {
            // Use VLC to decode h264 to video
            DM_VLC,
            // Use LIVE555 and Openh264 to decode (can get metadata)
            DM_META
        };
        DecodeMode getDecodeMode() const;
        bool setDecodeMode(const DecodeMode& mode);

        bool getFrameCountOverlayMode() const;
        // All 'set' methods return false if no change is necessary,
        // They also throw if there's a failure to save changes persistently.
        bool setFrameCountOverlayMode(bool mode);
        bool getBitrateOverlayMode() const;
        bool setBitrateOverlayMode(bool mode);

        PelcoImextk_StreamOptimizedType getRenderOption() const;
        bool setRenderOption(const PelcoImextk_StreamOptimizedType type);




        //**********************************************************************
        // Global, non-persistent settings
        //**********************************************************************

        enum ViewLayoutId {
            VL_SINGLE, // single, full-size immersive view
            VL_DOUBLE, // Two side-by-side immersive views
            VL_SA,     // single, full-size situational awareness view
            VL_SA_1,   // situational awareness view along the top, and immersive below
            VL_SA_2,    // situational awareness view along the top, and 2 immersive below
            VL_ALL_MONITORS // 1 situational awareness + immersive on each separate monitor
        };
        ViewLayoutId getViewLayout() const;
        bool setViewLayout(ViewLayoutId id);

        enum ViewLayoutPanoModId {
            VLPM_DEFAULT, //used by 180 pano, mercator view
            VLPM_PELCO270_OPTIMIZED //optimized best view projection on 270 cameras
        };
        ViewLayoutPanoModId getViewLayoutPanoMod() const;
        bool setViewLayoutPanoMod(ViewLayoutPanoModId id);

        bool getConnectAsNonOpteraType(void) const;
        bool getIsPrimaryStreamSelected(void) const;
        bool getPTZSituationalAwarenessMode(void) const;

        struct PtzLimitOptions {
                bool operator==(const PtzLimitOptions& other) const {
                    return (_mode == other._mode) &&
                        (_autoZoomInWithPanTilt == other._autoZoomInWithPanTilt) &&
                        (_autoZoomOutWithPanTilt == other._autoZoomOutWithPanTilt) &&
                        (_autoPanTiltWithZoomOut == other._autoPanTiltWithZoomOut);
                }

                bool operator!=(const PtzLimitOptions& other) const {
                    return !(*this == other);
                }
                PelcoImextk_PtzLimitMode _mode;
                bool _autoZoomInWithPanTilt;
                bool _autoZoomOutWithPanTilt;
                bool _autoPanTiltWithZoomOut;
        };

        PtzLimitOptions getPtzLimitOptions() const;
        bool setPtzLimitOptions(const PtzLimitOptions& opts);

        //**********************************************************************
        // Per-source, persistent options, stored with the source view history,
        // and re-set upon adding a new source
        //**********************************************************************

        // moves the given camera to the top of the source history list.
        // Returns true if the camera is now selected, otherwise (including if
        // the camera is unknown) returns false.
        bool selectCamera(const std::wstring& ipAddress);

        // moves the most recently viewed camera to the top of the source
        // history list.
        // Returns true if the camera is now selected, otherwise (including if
        // the camera is unknown) returns false.
        bool selectLatestCamera();

        // moves the given source file dir to the top of the source history list.
        // Returns true if source is now selected, otherwise (including if
        // the source is unknown) returns false.
        bool selectFixedViewDir(const std::wstring& directory);

        // moves the most recently viewed fixed-view directory to the top
        // of the source history list.
        // Returns true if source is now selected, otherwise (including if
        // the source is unknown) returns false.
        bool selectLatestFixedViewDir();

        // adds a camera to the camera history list and selects it as the
        // current camera.
        void addCamera(const std::wstring& ipAddress, const std::wstring& logicalName, bool viewHighRes, bool connectAsNonOpteraType,
			bool uniStream, bool primaryStream, bool useSitAwareness, const std::wstring& sitAwarePTZAddress, bool separatedCameras, const std::wstring& ptzHeight,
			const std::wstring& opteraHeight, const std::wstring& distanceToOptera, const std::wstring& minMoveDistance,
			const std::wstring& ptzTiltX, const std::wstring& ptzTiltZ, const std::wstring& ptzConeError, const std::wstring& opteraTiltX,
			const std::wstring& opteraTiltZ, const std::wstring& opteraConeError);

        // adds a camera to the camera history list and selects it as the
        // current camera.
        void addFixedViewDir(const std::wstring& sourceDir,
                const std::wstring& logicalName);

        // persistent camera history settings
        struct SourceHistoryRecord {
                bool         _isCamera;
                // _id is either an IP address or a directory depending
                // on the value of _isCamera
                std::wstring _id;
                std::wstring _logicalName;
                bool         _highRes;
                bool         _uniStream;
                bool         _overrideCameraTiltAngle; // in radians
                float        _cameraTiltAngle; // in radians
                ViewLayoutPanoModId _viewLayoutPanoMod; //modifier to panoramic, optimized for 270, default(180), etc...
                bool         _connectAsNonOpteraType;
                bool         _primaryStream; //used with nonoptera type camera stream1/stream2

                // Only for situational awareness with Optera 
                bool         _useSitAwareness;
                std::wstring _sitAwarePtzId;
				bool         _separatedCameras;
				std::wstring _ptzHeight;
				std::wstring _opteraHeight;
				std::wstring _distanceToOptera;
				std::wstring _minMoveDistance;
				std::wstring _ptzGlobalTiltX;
				std::wstring _ptzGlobalTiltZ;
				std::wstring _ptzConeError;
				std::wstring _opteraGlobalTiltX;
				std::wstring _opteraGlobalTiltZ;
				std::wstring _opteraConeError;
				float        _ptzGlobalPositionX;
				float        _ptzGlobalPositionZ;
				float        _ptzPanReference;
                    
                // Keep Last
                bool         _multicast; //connect to multicast stream
        };
        std::list<SourceHistoryRecord> getHistory() const {
            return _sourceHistory;
        }

        static bool findLatestCamera(const std::list<SourceHistoryRecord>& history,
                SourceHistoryRecord& rec);

        bool findLatestCamera(SourceHistoryRecord& rec) const {
            return findLatestCamera(getHistory(), rec);
        }

        static bool findLatestFixedViewDir(const std::list<SourceHistoryRecord>& history,
                SourceHistoryRecord& rec);

        bool findLatestFixedViewDir(SourceHistoryRecord& rec) const {
            return findLatestFixedViewDir(getHistory(), rec);
        }

        //
        // Information about the currently selected source
        //
        // It is an error to call these if no source has yet been selected.
        SourceHistoryRecord getCurrentSourceInfo() const;
        void updateCurrentSourceInfo(const std::wstring& logicalName, bool highRes, bool connectAsNonOpteraType, bool uniStream, bool primaryStream,
			bool multicast, bool useSitAwareness, const std::wstring& sitAwarePTZAddress, bool separatedCameras, const std::wstring& ptzHeight, const std::wstring& opteraHeight,
			const std::wstring& distanceToOptera, const std::wstring& minMoveDistance, const std::wstring& ptzTiltX, const std::wstring& ptzTiltZ, 
			const std::wstring& ptzConeError, const std::wstring& opteraTiltX, const std::wstring& opteraTiltZ, const std::wstring& opteraConeError);
		void updateCameraTiltAngle(bool overrideCamera, float angle);
		void updateCalibrationParams(float *ptzPanReference, float *ptzGlobalPositionX, float *ptzGlobalPositionZ, float *distanceToOptera, float *ptzHeight,
			                         float *opteraTiltX, float *opteraTiltZ, float *ptzTiltX, float *ptxTiltZ);
		void getCalibrationParams(float *ptzGlobalPositionX, float *ptzGlobalPositionZ, float *ptzPanReference, float *distanceToOptera, float *ptzHeight,
			                      float *opteraTiltX, float *opteraTiltZ, float *ptzTiltX, float *ptxTiltZ);

        bool isSourceSelected() const {
            return !_sourceHistory.empty();
        }

    private:
        bool load();
        void save();
        bool selectSource(std::function<bool(const SourceHistoryRecord& rec)> pred);
        static bool findLatestFixedViewDir(const std::list<SourceHistoryRecord>& history,
                std::function<bool(const SourceHistoryRecord& rec)> pred,
                SourceHistoryRecord& rec);

        static const std::wstring _settingsFilename;

        static const std::string _renderModeKey;
        static const std::string _decodeModeKey;
        static const std::string _renderOptionKey;
        static const std::string _frameCountOverlayModeKey;
        static const std::string _bitrateOverlayModeKey;
        static const std::string _ptzLimitModeKey;
        static const std::string _autoZoomInWithPanTiltKey;
        static const std::string _autoZoomOutWithPanTiltKey;
        static const std::string _autoPanTiltWithZoomOutKey;
        static const std::string _sourceHistoryCountKey;
        static const std::string _isCameraPrefix;
        static const std::string _sourceIdPrefix;
        static const std::string _sitAwarePTZIdPrefix;
		static const std::string _sitAwareEnablePrefix;
		static const std::string _distanceToOpteraPrefix;
		static const std::string _separatedCamerasPrefix;
		static const std::string _minMoveDistancePrefix;
        static const std::string _ptzHeightPrefix;
        static const std::string _opteraHeightPrefix;
        static const std::string _ptzGlobalPositionXPrefix;
		static const std::string _ptzGlobalPositionZPrefix;
		static const std::string _ptzPanReferencePrefix;
		static const std::string _ptzGlobalTiltXPrefix;
		static const std::string _ptzGlobalTiltZPrefix;
		static const std::string _ptzConeErrorPrefix;
		static const std::string _opteraGlobalTiltXPrefix;
		static const std::string _opteraGlobalTiltZPrefix;
		static const std::string _opteraConeErrorPrefix;
		static const std::string _logicalNamePrefix;
        static const std::string _highResPrefix;
        static const std::string _uniStreamPrefix;
        static const std::string _multicastPrefix;
        static const std::string _nonOpteraTypePrefix;
        static const std::string _primaryStreamPrefix;
        static const std::string _cameraTiltOverridePrefix;
        static const std::string _cameraTiltPrefix;
        static const std::string _viewLayoutPanoModPrefix;
        static const size_t      _maxHistoryCount;

        // holds persistent global settings
        std::map<std::string, std::string> _opts;

        // non-persistent global settings
        ViewLayoutId _viewLayout;

        std::list<SourceHistoryRecord> _sourceHistory;
};

#endif // __PELCO_IMEXTK_USER_OPTIONS_HPP__
