//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "UserOptions.hpp"
#include <fstream>
#include <sstream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <codecvt>
#include <string>

using namespace std;

const wstring UserOptions::_settingsFilename(L"settings.conf");

const string UserOptions::_renderModeKey("RenderMode");
const string UserOptions::_decodeModeKey("DecodeMode");
const string UserOptions::_renderOptionKey("RenderOption");
const string UserOptions::_frameCountOverlayModeKey("FrameCountOverlayMode");
const string UserOptions::_bitrateOverlayModeKey("BitrateOverlayMode");
const string UserOptions::_ptzLimitModeKey("PtzLimitMode");
const string UserOptions::_autoZoomInWithPanTiltKey("AutoZoomInWithPanTilt");
const string UserOptions::_autoZoomOutWithPanTiltKey("AutoZoomOutWithPanTilt");
const string UserOptions::_autoPanTiltWithZoomOutKey("AutoPanTiltWithZoomOut");

const string UserOptions::_sourceHistoryCountKey("HistoryCount");
const string UserOptions::_isCameraPrefix("SourceIsCamera_");
const string UserOptions::_sourceIdPrefix("SourceID_");
const string UserOptions::_logicalNamePrefix("SourceLogicalName_");
const string UserOptions::_highResPrefix("SourceHighRes_");
const string UserOptions::_uniStreamPrefix("SourceUniStream_");
const string UserOptions::_multicastPrefix("Multicast_");
const string UserOptions::_nonOpteraTypePrefix("NonOpteraType_");
const string UserOptions::_cameraTiltOverridePrefix("SourceCameraOverrideTilt_");
const string UserOptions::_cameraTiltPrefix("SourceCameraTilt_");
const string UserOptions::_viewLayoutPanoModPrefix("panoramicModel_");
const string UserOptions::_primaryStreamPrefix("UsePrimaryStream_");
const string UserOptions::_sitAwareEnablePrefix("SitAwareEnable_");
const string UserOptions::_sitAwarePTZIdPrefix("SitAwarePTZID_");
const string UserOptions::_ptzHeightPrefix("PTZHeight_");
const string UserOptions::_opteraHeightPrefix("OpteraHeight_");
const string UserOptions::_distanceToOpteraPrefix("DistanceToOptera_");
const string UserOptions::_separatedCamerasPrefix("SeparatedCameras_");
const string UserOptions::_minMoveDistancePrefix("MinMoveDistance_");
const string UserOptions::_ptzGlobalPositionXPrefix("PtzGlobalPositionX_");
const string UserOptions::_ptzGlobalPositionZPrefix("PtzGlobalPositionZ_");
const string UserOptions::_ptzPanReferencePrefix("PtzPanReference_");
const string UserOptions::_ptzGlobalTiltXPrefix("PtzGlobalTiltX_");
const string UserOptions::_ptzGlobalTiltZPrefix("PtzGlobalTiltZ_");
const string UserOptions::_ptzConeErrorPrefix("PtzConeError_");
const string UserOptions::_opteraGlobalTiltXPrefix("OpteraGlobalTiltX_");
const string UserOptions::_opteraGlobalTiltZPrefix("OpteraGlobalTiltZ_");
const string UserOptions::_opteraConeErrorPrefix("OpteraConeError_");

const size_t UserOptions::_maxHistoryCount = 9;


//////////////////////////////////////////////////////////////////////////////////
UserOptions::UserOptions()
    : _viewLayout(VL_SINGLE)
{
    if (!load()) {
        // initialize with default values
        _opts[_renderModeKey] = to_string(RM_DIRECT);
        _opts[_decodeModeKey] = to_string(DM_VLC);
        _opts[_renderOptionKey] = to_string(PELCO_IMEXTK_OPTIMIZED_AUTOMATIC);
        _opts[_frameCountOverlayModeKey] = to_string(false);
        _opts[_bitrateOverlayModeKey] = to_string(false);
        _opts[_ptzLimitModeKey] = to_string(PELCO_IMEXTK_PTZ_LIMIT_VIEW_EDGE);
        _opts[_autoZoomInWithPanTiltKey] = to_string(false);
        _opts[_autoZoomOutWithPanTiltKey] = to_string(false);
        _opts[_autoPanTiltWithZoomOutKey] = to_string(true);
    }
}

//////////////////////////////////////////////////////////////////////////////////
UserOptions::~UserOptions()
{
}

//////////////////////////////////////////////////////////////////////////////////
UserOptions::RenderMode UserOptions::getRenderMode() const
{
    return static_cast<RenderMode>(stoi(_opts.at(_renderModeKey)));
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setRenderMode(const RenderMode& mode)
{
    bool retval = false;
    if (mode != getRenderMode()) {
        _opts[_renderModeKey] = to_string(mode);
        save();
        retval = true;
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
UserOptions::DecodeMode UserOptions::getDecodeMode() const
{
    return static_cast<DecodeMode>(stoi(_opts.at(_decodeModeKey)));
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setDecodeMode(const DecodeMode& mode)
{
    bool retval = false;
    if (mode != getDecodeMode()) {
        _opts[_decodeModeKey] = to_string(mode);
        save();
        retval = true;
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
PelcoImextk_StreamOptimizedType UserOptions::getRenderOption() const
{
    return static_cast<PelcoImextk_StreamOptimizedType>(stoi(_opts.at(_renderOptionKey)));
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setRenderOption(const PelcoImextk_StreamOptimizedType type)
{
    bool retval = false;
    if (type != getRenderOption()) {
        _opts[_renderOptionKey] = to_string(type);
        save();
        retval = true;
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::getFrameCountOverlayMode() const
{
    return stoi(_opts.at(_frameCountOverlayModeKey)) != 0;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setFrameCountOverlayMode(bool mode)
{
    bool retval = false;
    if (mode != getFrameCountOverlayMode()) {
        _opts[_frameCountOverlayModeKey] = to_string(mode);
        save();
        retval = true;
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::getBitrateOverlayMode() const
{
    return stoi(_opts.at(_bitrateOverlayModeKey)) != 0;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setBitrateOverlayMode(bool mode)
{
    bool retval = false;
    if (mode != getBitrateOverlayMode()) {
        _opts[_bitrateOverlayModeKey] = to_string(mode);
        save();
        retval = true;
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
UserOptions::ViewLayoutId UserOptions::getViewLayout() const
{
    return _viewLayout;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setViewLayout(ViewLayoutId id)
{
    bool retval = false;
    if (id != getViewLayout()) {
        _viewLayout = id;
        retval = true;
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
UserOptions::ViewLayoutPanoModId UserOptions::getViewLayoutPanoMod() const
{
    if (_sourceHistory.empty()) {
        return VLPM_DEFAULT;
    }
    const SourceHistoryRecord& rec = _sourceHistory.front();
    return rec._viewLayoutPanoMod;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setViewLayoutPanoMod(ViewLayoutPanoModId id)
{
    bool retval = false;
    assert(!_sourceHistory.empty());
    SourceHistoryRecord& rec = _sourceHistory.front();

    if (id != getViewLayoutPanoMod()) {
        rec._viewLayoutPanoMod = id;
        save();
        retval = true;
    }

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::getConnectAsNonOpteraType(void) const
{
    if (_sourceHistory.empty()) {
        return false;
    }
    const SourceHistoryRecord& rec = _sourceHistory.front();
    return rec._connectAsNonOpteraType;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::getIsPrimaryStreamSelected(void) const
{
    if (_sourceHistory.empty()) {
        return false;
    }
    const SourceHistoryRecord& rec = _sourceHistory.front();
    return rec._primaryStream;
}

bool UserOptions::getPTZSituationalAwarenessMode(void) const
{
    if (_sourceHistory.empty()) {
        return false;
    }
    const SourceHistoryRecord& rec = _sourceHistory.front();
    return rec._useSitAwareness;
}
//////////////////////////////////////////////////////////////////////////////////
UserOptions::PtzLimitOptions UserOptions::getPtzLimitOptions() const
{
    PtzLimitOptions ptzOpts;
    ptzOpts._mode = static_cast<PelcoImextk_PtzLimitMode>(
        stoi(_opts.at(_ptzLimitModeKey)));
    ptzOpts._autoZoomInWithPanTilt =
        stoi(_opts.at(_autoZoomInWithPanTiltKey)) != 0;
    ptzOpts._autoZoomOutWithPanTilt =
        stoi(_opts.at(_autoZoomOutWithPanTiltKey)) != 0;
    ptzOpts._autoPanTiltWithZoomOut =
        stoi(_opts.at(_autoPanTiltWithZoomOutKey)) != 0;

    return ptzOpts;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::setPtzLimitOptions(const PtzLimitOptions& opts)
{
    bool retval = false;
    if (opts != getPtzLimitOptions()) {
        _opts[_ptzLimitModeKey] = to_string(opts._mode);
        _opts[_autoZoomInWithPanTiltKey] = to_string(opts._autoZoomInWithPanTilt);
        _opts[_autoZoomOutWithPanTiltKey] = to_string(opts._autoZoomOutWithPanTilt);
        _opts[_autoPanTiltWithZoomOutKey] = to_string(opts._autoPanTiltWithZoomOut);

        save();
        retval = true;
    }
    return retval;
}


//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::selectSource(function<bool(const SourceHistoryRecord& rec)> pred) {
    bool retval = false;
    list<SourceHistoryRecord>::iterator itr =
        find_if(_sourceHistory.begin(), _sourceHistory.end(), pred);

    if (itr != _sourceHistory.end()) {
        // move itr to the front of the list
        _sourceHistory.splice(_sourceHistory.begin(),
                _sourceHistory, itr);
        save();
        retval = true;
    }

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::selectCamera(const std::wstring& ipAddress)
{
    return selectSource([&ipAddress] (const SourceHistoryRecord& rec) {
                return bool(rec._isCamera && (rec._id == ipAddress));
            });
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::selectLatestCamera()
{
    return selectSource([] (const SourceHistoryRecord& rec) {
                return rec._isCamera;
            });
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::selectFixedViewDir(const std::wstring& directory)
{
    return selectSource([&directory] (const SourceHistoryRecord& rec) {
                return (!rec._isCamera) && (rec._id == directory);
            });
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::selectLatestFixedViewDir()
{
    return selectSource([] (const SourceHistoryRecord& rec) {
                return (!rec._isCamera);
            });
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::findLatestFixedViewDir(const list<SourceHistoryRecord>& history,
        function<bool(const SourceHistoryRecord& rec)> pred,
        SourceHistoryRecord& rec)
{
    bool retval = false;
    list<SourceHistoryRecord>::const_iterator itr =
        find_if(history.begin(), history.end(), pred);

    if (itr != history.end()) {
        rec = *itr;
        retval = true;
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::findLatestCamera(const std::list<SourceHistoryRecord>& history,
        SourceHistoryRecord& rec)
{
    return findLatestFixedViewDir(history, [] (const SourceHistoryRecord& rec) {
                return (rec._isCamera);
            },
            rec);
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::findLatestFixedViewDir(const std::list<SourceHistoryRecord>& history,
        SourceHistoryRecord& rec)
{
    return findLatestFixedViewDir(history, [] (const SourceHistoryRecord& rec) {
                return (!rec._isCamera);
            },
            rec);
}

//////////////////////////////////////////////////////////////////////////////////
UserOptions::SourceHistoryRecord UserOptions::getCurrentSourceInfo() const
{
    assert(!_sourceHistory.empty());
    return _sourceHistory.front();
}

//////////////////////////////////////////////////////////////////////////////////
void UserOptions::updateCurrentSourceInfo(const wstring& logicalName, bool highRes, bool connectAsNonOpteraType, bool uniStream, bool primaryStream, bool multicast,
	bool useSitAwareness, const wstring& sitAwarePTZAddress, bool separatedCameras, const wstring& ptzHeight, const wstring& opteraHeight, const wstring& distanceToOptera, const wstring& minMoveDistance,
	const wstring& ptzTiltX, const wstring& ptzTiltZ, const wstring& ptzConeError, const wstring& opteraTiltX, const wstring& opteraTiltZ, const wstring& opteraConeError)
{
    assert(!_sourceHistory.empty());
    SourceHistoryRecord& rec = _sourceHistory.front();
    bool saveNeeded = false;
    if (rec._logicalName != logicalName) {
        rec._logicalName = logicalName;
        saveNeeded = true;
    }
    if (rec._highRes != highRes) {
        rec._highRes = highRes;
        saveNeeded = true;
    }
    if (rec._connectAsNonOpteraType != connectAsNonOpteraType) {
        rec._connectAsNonOpteraType = connectAsNonOpteraType;
        saveNeeded = true;
    }
    if (rec._uniStream != uniStream) {
        rec._uniStream = uniStream;
        saveNeeded = true;
    }
    if (rec._primaryStream != primaryStream) {
        rec._primaryStream = primaryStream;
        saveNeeded = true;
    }
    if (rec._multicast != multicast) {
        rec._multicast = multicast;
        saveNeeded = true;
    }
    if (rec._useSitAwareness != useSitAwareness) {
        rec._useSitAwareness = useSitAwareness;
        saveNeeded = true;
	}
	if (rec._sitAwarePtzId != sitAwarePTZAddress) {
		rec._sitAwarePtzId = sitAwarePTZAddress;
		saveNeeded = true;
	}
	if (rec._separatedCameras != separatedCameras) {
		rec._separatedCameras = separatedCameras;
		saveNeeded = true;
	}
    if (rec._ptzHeight != ptzHeight) {
        rec._ptzHeight = ptzHeight;
        saveNeeded = true;
    }
    if (rec._opteraHeight != opteraHeight) {
        rec._opteraHeight = opteraHeight;
        saveNeeded = true;
	}
	if (rec._distanceToOptera != distanceToOptera) {
		rec._distanceToOptera = distanceToOptera;
		saveNeeded = true;
	}
	if (rec._minMoveDistance != minMoveDistance) {
		rec._minMoveDistance = minMoveDistance;
		saveNeeded = true;
	}
	if (rec._ptzGlobalTiltX != ptzTiltX) {
		rec._ptzGlobalTiltX = ptzTiltX;
		saveNeeded = true;
	}
	if (rec._ptzGlobalTiltZ != ptzTiltZ) {
		rec._ptzGlobalTiltZ = ptzTiltZ;
		saveNeeded = true;
	}
	if (rec._ptzConeError != ptzConeError) {
		rec._ptzConeError = ptzConeError;
		saveNeeded = true;
	}
	if (rec._opteraGlobalTiltX != opteraTiltX) {
		rec._opteraGlobalTiltX = opteraTiltX;
		saveNeeded = true;
	}
	if (rec._opteraGlobalTiltZ != opteraTiltZ) {
		rec._opteraGlobalTiltZ = opteraTiltZ;
		saveNeeded = true;
	}

	if (rec._opteraConeError != opteraConeError) {
		rec._opteraConeError = opteraConeError;
		saveNeeded = true;
	}

	if (saveNeeded) {
        save();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void UserOptions::updateCameraTiltAngle(bool overrideCamera, float angle)
{
	assert(!_sourceHistory.empty());
	SourceHistoryRecord& rec = _sourceHistory.front();
	bool saveNeeded = false;
	if (rec._overrideCameraTiltAngle != overrideCamera) {
		rec._overrideCameraTiltAngle = overrideCamera;
		saveNeeded = true;
	}
	if (rec._cameraTiltAngle != angle) {
		rec._cameraTiltAngle = angle;
		saveNeeded = true;
	}

	if (saveNeeded) {
		save();
	}
}

//////////////////////////////////////////////////////////////////////////////////
void UserOptions::updateCalibrationParams(float *ptzPanReference, float *ptzGlobalPositionX, float *ptzGlobalPositionZ, float *distanceToOptera, float *ptzHeight,
	                                      float *opteraTiltX, float *opteraTiltZ, float *ptzTiltX, float *ptzTiltZ)
{
	assert(!_sourceHistory.empty());
	SourceHistoryRecord& rec = _sourceHistory.front();
	bool saveNeeded = false;
	if (ptzPanReference) {
		if (rec._ptzPanReference != *ptzPanReference) {
			rec._ptzPanReference = *ptzPanReference;
			saveNeeded = true;
		}
	}
	if (ptzGlobalPositionX) {
		if (rec._ptzGlobalPositionX != *ptzGlobalPositionX) {
			rec._ptzGlobalPositionX = *ptzGlobalPositionX;
			saveNeeded = true;
		}
	}
	if (ptzGlobalPositionZ) {
		if (rec._ptzGlobalPositionZ != *ptzGlobalPositionZ) {
			rec._ptzGlobalPositionZ = *ptzGlobalPositionZ;
			saveNeeded = true;
		}
	}
	if (distanceToOptera) {
		std::wstring distanceToOpteraStr = std::to_wstring(*distanceToOptera);
		if (rec._distanceToOptera != distanceToOpteraStr) {
			rec._distanceToOptera = distanceToOpteraStr;
			saveNeeded = true;
		}
	}
	if (ptzHeight) {
		std::wstring ptzHeightStr = std::to_wstring(*ptzHeight);
		if (rec._ptzHeight != ptzHeightStr) {
			rec._ptzHeight = ptzHeightStr;
			saveNeeded = true;
		}
	}
	if (opteraTiltX) {
		std::wstring opteraTiltXStr = std::to_wstring(*opteraTiltX);
		if (rec._opteraGlobalTiltX != opteraTiltXStr) {
			rec._opteraGlobalTiltX = opteraTiltXStr;
			saveNeeded = true;
		}
	}
	if (opteraTiltZ) {
		std::wstring opteraTiltZStr = std::to_wstring(*opteraTiltZ);
		if (rec._opteraGlobalTiltZ != opteraTiltZStr) {
			rec._opteraGlobalTiltZ = opteraTiltZStr;
			saveNeeded = true;
		}
	}
	if (ptzTiltX) {
		std::wstring ptzTiltXStr = std::to_wstring(*ptzTiltX);
		if (rec._ptzGlobalTiltX != ptzTiltXStr) {
			rec._ptzGlobalTiltX = ptzTiltXStr;
			saveNeeded = true;
		}
	}
	if (ptzTiltZ) {
		std::wstring ptzTiltZStr = std::to_wstring(*ptzTiltZ);
		if (rec._ptzGlobalTiltZ != ptzTiltZStr) {
			rec._ptzGlobalTiltZ = ptzTiltZStr;
			saveNeeded = true;
		}
	}

	if (saveNeeded) {
		save();
	}
}

//////////////////////////////////////////////////////////////////////////////////
void UserOptions::getCalibrationParams(float *ptzGlobalPositionX, float *ptzGlobalPositionY, float *ptzPanReference, float *distanceToOptera, float *ptzHeight,
	                                   float *opteraTiltX, float *opteraTiltZ, float *ptzTiltX, float *ptzTiltZ)
{
	assert(!_sourceHistory.empty());
	SourceHistoryRecord& rec = _sourceHistory.front();

	if (ptzGlobalPositionX) {
		*ptzGlobalPositionX = rec._ptzGlobalPositionX;
	}
	if (ptzGlobalPositionY) {
		*ptzGlobalPositionY = rec._ptzGlobalPositionZ;
	}
	if (ptzPanReference) {
		*ptzPanReference = rec._ptzPanReference;
	}
	if (distanceToOptera) {
		*distanceToOptera = (float)_wtof(rec._distanceToOptera.c_str());
	}
	if (ptzHeight) {
		*ptzHeight = (float)_wtof(rec._ptzHeight.c_str());
	}
	if (opteraTiltX) {
		*opteraTiltX = (float)_wtof(rec._opteraGlobalTiltX.c_str());
	}
	if (opteraTiltZ) {
		*opteraTiltZ = (float)_wtof(rec._opteraGlobalTiltZ.c_str());
	}
	if (ptzTiltX) {
		*ptzTiltX = (float)_wtof(rec._ptzGlobalTiltX.c_str());
	}
	if (ptzTiltZ) {
		*ptzTiltZ = (float)_wtof(rec._ptzGlobalTiltZ.c_str());
	}
}

//////////////////////////////////////////////////////////////////////////////////
void UserOptions::addCamera(const wstring& ipAddress, const wstring& logicalName, bool viewHighRes, bool connectAsNonOpteraType, bool uniStream, 
	bool primaryStream, bool useSitAwareness, const wstring& sitAwarePTZAddress, bool separatedCameras, const wstring& ptzHeight, const wstring& opteraHeight,
	const wstring& distanceToOptera, const wstring& minMoveDistance, const wstring& ptzTiltX, const wstring& ptzTiltZ, const wstring& ptzConeErr, 
	const wstring& opteraTiltX, const wstring& opteraTiltZ, const wstring& opteraConeErr)
{
    _sourceHistory.push_front(
	{ true, ipAddress, logicalName, viewHighRes, uniStream, false, 0.0f, VLPM_DEFAULT, connectAsNonOpteraType, primaryStream, useSitAwareness, sitAwarePTZAddress, separatedCameras, ptzHeight, opteraHeight, distanceToOptera, minMoveDistance, ptzTiltX, ptzTiltZ, ptzConeErr, opteraTiltX, opteraTiltZ, opteraConeErr, 0.0, 0.0, 0.0 });
    if (_sourceHistory.size() > _maxHistoryCount) {
        _sourceHistory.pop_back();
    }
    save();
}

//////////////////////////////////////////////////////////////////////////////////
void UserOptions::addFixedViewDir(const wstring& directory,
        const wstring& logicalName)
{
    _sourceHistory.push_front(
        {false, directory, logicalName, true, 0.0f, VLPM_DEFAULT});
    if (_sourceHistory.size() > _maxHistoryCount) {
        _sourceHistory.pop_back();
    }
    save();
}

//////////////////////////////////////////////////////////////////////////////////
bool UserOptions::load()
{
    //
    // A real application would use the registry, but as demo app has no
    // install or uninstall script, we just save a use a simple local
    // file full of "key=value\n" strings.
    //
    ifstream fstrm(_settingsFilename, ios::in);
    if (!fstrm.good()) {
        return false;
    }

    bool retval = false;
    try {
        // parse file contents into a map
        char c;
        stringstream strm;
        string key;
        map<string, string> keyVals;
        while (fstrm.read(&c, 1).good()) {
            if (c == '=') {
                key = strm.str();
                strm.str("");
            }
            else if ((c == '\n') || (c == '\r')) {
                keyVals[key] = strm.str();
                strm.str("");
                // skip passed any multi-char line-end sequence
                while (fstrm.read(&c, 1).good() &&
                        ((c == '\n') || (c == '\r'))) {
                }
                if (fstrm.good()) {
                    fstrm.putback(c);
                }
            }
            else {
                strm << c;
            }
        }

        // extract _opts values from keyVals
        _opts.insert(pair<string, string>(_renderModeKey,
                        keyVals.at(_renderModeKey)));
        _opts.insert(pair<string, string>(_decodeModeKey,
                          keyVals.at(_decodeModeKey)));
        _opts.insert(pair<string, string>(_renderOptionKey,
                        keyVals.at(_renderOptionKey)));
        _opts.insert(pair<string, string>(_frameCountOverlayModeKey,
                        keyVals.at(_frameCountOverlayModeKey)));
        _opts.insert(pair<string, string>(_bitrateOverlayModeKey,
                        keyVals.at(_bitrateOverlayModeKey)));
        _opts.insert(pair<string, string>(_ptzLimitModeKey,
                        keyVals.at(_ptzLimitModeKey)));
        _opts.insert(pair<string, string>(_autoZoomInWithPanTiltKey,
                        keyVals.at(_autoZoomInWithPanTiltKey)));
        _opts.insert(pair<string, string>(_autoZoomOutWithPanTiltKey,
                        keyVals.at(_autoZoomOutWithPanTiltKey)));
        _opts.insert(pair<string, string>(_autoPanTiltWithZoomOutKey,
                        keyVals.at(_autoPanTiltWithZoomOutKey)));

        // load source history
        size_t historyCount = stoi(keyVals.at(_sourceHistoryCountKey));
        wstring_convert<codecvt_utf8<wchar_t>> conv;
        for (size_t i = 0; i < historyCount; ++i) {
            SourceHistoryRecord rec;
            string suffix = to_string(i);
            rec._isCamera = stoi(keyVals.at(_isCameraPrefix + suffix)) != 0;
            rec._id = conv.from_bytes(keyVals.at(_sourceIdPrefix + suffix));
            rec._sitAwarePtzId = conv.from_bytes(keyVals.at(_sitAwarePTZIdPrefix + suffix));
            rec._useSitAwareness = stoi(keyVals.at(_sitAwareEnablePrefix + suffix)) != 0;
			rec._ptzHeight = conv.from_bytes(keyVals.at(_ptzHeightPrefix + suffix));
			rec._opteraHeight = conv.from_bytes(keyVals.at(_opteraHeightPrefix + suffix));
			rec._distanceToOptera = conv.from_bytes(keyVals.at(_distanceToOpteraPrefix + suffix));
			rec._separatedCameras = stoi(keyVals.at(_separatedCamerasPrefix + suffix)) != 0;
			rec._minMoveDistance = conv.from_bytes(keyVals.at(_minMoveDistancePrefix + suffix));
			rec._ptzGlobalTiltX = conv.from_bytes(keyVals.at(_ptzGlobalTiltXPrefix + suffix));
			rec._ptzGlobalTiltZ = conv.from_bytes(keyVals.at(_ptzGlobalTiltZPrefix + suffix));
			rec._ptzConeError = conv.from_bytes(keyVals.at(_ptzConeErrorPrefix + suffix));
			rec._opteraGlobalTiltX = conv.from_bytes(keyVals.at(_opteraGlobalTiltXPrefix + suffix));
			rec._opteraGlobalTiltZ = conv.from_bytes(keyVals.at(_opteraGlobalTiltZPrefix + suffix));
			rec._opteraConeError = conv.from_bytes(keyVals.at(_opteraConeErrorPrefix + suffix));
            rec._logicalName = conv.from_bytes(keyVals.at(_logicalNamePrefix + suffix));
            rec._highRes = stoi(keyVals.at(_highResPrefix + suffix)) != 0;
            rec._cameraTiltAngle = stof(keyVals.at(_cameraTiltPrefix + suffix));
            rec._viewLayoutPanoMod =
                static_cast<ViewLayoutPanoModId>(stoi(keyVals.at(_viewLayoutPanoModPrefix + suffix)));
			rec._ptzGlobalPositionX = stof(keyVals.at(_ptzGlobalPositionXPrefix + suffix));
			rec._ptzGlobalPositionZ = stof(keyVals.at(_ptzGlobalPositionZPrefix + suffix));
			rec._ptzPanReference = stof(keyVals.at(_ptzPanReferencePrefix + suffix));

            // new values, which may not be in old settings files.
            map<string,string>::const_iterator itr = keyVals.find(_cameraTiltOverridePrefix + suffix);
            if (itr == keyVals.end()) {
                rec._overrideCameraTiltAngle = (!rec._isCamera) || (rec._cameraTiltAngle != 0);
            }
            else {
                rec._overrideCameraTiltAngle = stoi(itr->second) != 0;
            }
            itr = keyVals.find(_nonOpteraTypePrefix + suffix);
            if (itr == keyVals.end()) {
                rec._connectAsNonOpteraType = false;
            }
            else {
                rec._connectAsNonOpteraType = stoi(keyVals.at(_nonOpteraTypePrefix + suffix)) != 0;
            }
            itr = keyVals.find(_uniStreamPrefix + suffix);
            if (itr == keyVals.end()) {
                rec._uniStream = false;
            }
            else {
                rec._uniStream = stoi(keyVals.at(_uniStreamPrefix + suffix)) != 0;
            }
            itr = keyVals.find(_primaryStreamPrefix + suffix);
            if (itr == keyVals.end()) {
                rec._primaryStream = true;
            }
            else {
                rec._primaryStream = stoi(keyVals.at(_primaryStreamPrefix + suffix)) != 0;
            }

            itr = keyVals.find(_multicastPrefix + suffix);
            if (itr == keyVals.end()) {
                rec._multicast = false;
            }
            else {
                rec._multicast = stoi(keyVals.at(_multicastPrefix + suffix)) != 0;
            }

            _sourceHistory.push_back(rec);
        }

        retval = true;
    }
    catch (...) {
    }

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
void UserOptions::save()
{
    ofstream strm(_settingsFilename, ios::out | ios::trunc);
    if (!strm.good()) {
        throw runtime_error("Unable to create file for saving user options");
    }

    // save persistent, global settings
    map<string, string>::const_iterator itr;
    for (itr = _opts.begin(); itr != _opts.end(); ++itr) {
        strm << itr->first << '=' << itr->second << endl;
    }

    wstring_convert<codecvt_utf8<wchar_t>> conv;

    // save history
    list<SourceHistoryRecord>::const_iterator itr2;
    size_t index = 0;
    for (itr2 = _sourceHistory.begin(); itr2 != _sourceHistory.end(); ++itr2) {
        strm << _isCameraPrefix << index << '=' << itr2->_isCamera << endl;
        strm << _sourceIdPrefix << index << '=' << conv.to_bytes(itr2->_id) << endl;
        strm << _logicalNamePrefix << index << '=' << conv.to_bytes(itr2->_logicalName) << endl;
        strm << _highResPrefix << index << '=' << itr2->_highRes << endl;
        strm << _uniStreamPrefix << index << '=' << itr2->_uniStream << endl;
        strm << _cameraTiltOverridePrefix << index << '=' << itr2->_overrideCameraTiltAngle << endl;
        strm << _cameraTiltPrefix << index << '=' << itr2->_cameraTiltAngle << endl;
        strm << _viewLayoutPanoModPrefix << index << '=' << itr2->_viewLayoutPanoMod << endl;
        strm << _nonOpteraTypePrefix << index << '=' << itr2->_connectAsNonOpteraType << endl;
        strm << _primaryStreamPrefix << index << '=' << itr2->_primaryStream << endl;
        strm << _multicastPrefix << index << '=' << itr2->_multicast << endl;
        strm << _sitAwareEnablePrefix << index << '=' << itr2->_useSitAwareness << endl;
		strm << _sitAwarePTZIdPrefix << index << '=' << conv.to_bytes(itr2->_sitAwarePtzId) << endl;
		strm << _distanceToOpteraPrefix << index << '=' << conv.to_bytes(itr2->_distanceToOptera) << endl;
		strm << _separatedCamerasPrefix << index << '=' << itr2->_separatedCameras << endl;
		strm << _minMoveDistancePrefix << index << '=' << conv.to_bytes(itr2->_minMoveDistance) << endl;
		strm << _ptzGlobalTiltXPrefix << index << '=' << conv.to_bytes(itr2->_ptzGlobalTiltX) << endl;
		strm << _ptzGlobalTiltZPrefix << index << '=' << conv.to_bytes(itr2->_ptzGlobalTiltZ) << endl;
		strm << _ptzConeErrorPrefix << index << '=' << conv.to_bytes(itr2->_ptzConeError) << endl;
		strm << _opteraGlobalTiltXPrefix << index << '=' << conv.to_bytes(itr2->_opteraGlobalTiltX) << endl;
		strm << _opteraGlobalTiltZPrefix << index << '=' << conv.to_bytes(itr2->_opteraGlobalTiltZ) << endl;
		strm << _opteraConeErrorPrefix << index << '=' << conv.to_bytes(itr2->_opteraConeError) << endl;
        strm << _ptzHeightPrefix << index << '=' << conv.to_bytes(itr2->_ptzHeight) << endl;
        strm << _opteraHeightPrefix << index << '=' << conv.to_bytes(itr2->_opteraHeight) << endl;
		strm << _ptzGlobalPositionXPrefix << index << '=' << itr2->_ptzGlobalPositionX << endl;
		strm << _ptzGlobalPositionZPrefix << index << '=' << itr2->_ptzGlobalPositionZ << endl;
		strm << _ptzPanReferencePrefix << index << '=' << itr2->_ptzPanReference << endl;
        ++index;
    }
    strm << _sourceHistoryCountKey << '=' << index << endl;
}


