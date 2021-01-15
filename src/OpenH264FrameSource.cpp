//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "OpenH264FrameSource.hpp"
#include <Windows.h>
#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>

#include <string>
#include "StreamLayoutMetadata.hpp"

#include <conio.h>
#include <ctime>

#include <pelco/imextk/H264UserData.h>

//#define ENABLE_FRAME_DEBUG
#ifdef ENABLE_FRAME_DEBUG
#define DBOUT( s )             \
{                              \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}
#else
#define DBOUT( s )
#endif

#define FOREVER while(1)
using namespace std;

bool   OpenH264FrameSource::_stopNow = false;
bool   OpenH264FrameSource::_networkInputThreadStopped[] = { false };
bool   OpenH264FrameSource::_decoderThreadStopped[] = { false };
bool   OpenH264FrameSource::_copyThreadStopped = false;
OpenH264FrameSource::STREAM_THREAD_INFO OpenH264FrameSource::_threadInfo[];

// Used to get buffers in sync on the screen
OpenH264FrameSource::TIME_BUFFER OpenH264FrameSource::_timeBuffer[];
string OpenH264FrameSource::_metaDataString;
bool OpenH264FrameSource::_decodingCanStart[] = { false };
mutex OpenH264FrameSource::_timeBufferCheckMutex;
mutex OpenH264FrameSource::_copyThreadLock;
condition_variable OpenH264FrameSource::_wakeUpCopyThread;

// for tracking analytic object
std::shared_ptr<SituationTracking> OpenH264FrameSource::_situationTracking;

unsigned OpenH264FrameSource::rtspClientCount = 0;              // Counts how many streams (i.e., "RTSPClient"s) are currently in use.
OpenH264FrameSource::ourRTSPClient * OpenH264FrameSource::_rtspClients[] = { NULL };

//////////////////////////////////////////////////////////////////////////////////
OpenH264FrameSource::OpenH264FrameSource(const string& cameraIP,
	const StreamSetupCallback& setup,
	const GetBufferCallback& getBuffer,
	const ReleaseBufferCallback& releaseBuffer,
    bool highRes, 
    bool connectAsNonOpteraType, 
    bool uniStream, 
    bool primaryStream, 
    const vector<string>& discoveryURLs, 
    bool multicast,
    bool useSitAwareness,
	const string& ptzIp,
	bool separatedCameras,
	const float opteraH,
	const float ptzH,
	const float distToOptera,
	const float minMoveDist,
	const float ptzTiltX,
	const float ptzTiltZ,
	const float ptzConeError,
	const float opteraTiltX,
	const float opteraTiltZ,
	const float opteraConeError,
    bool thisIsSitAwareCam,
    std::shared_ptr<SituationTracking> tracking,
    int encoderChannelSelected)
	: _cameraIP(cameraIP),
	_setupCallback(setup),
	_releaseBufferCallback(releaseBuffer),
	_highRes(highRes),
    _getBufferCallback(getBuffer),
    _useUnistream(uniStream),
    _useSitAwareness(useSitAwareness),
    _ptzIp(ptzIp),
	_separatedCameras(separatedCameras),
	_opteraHeight(opteraH),
	_ptzHeight(ptzH),
	_distanceToOptera(distToOptera),
	_minMoveDistance(minMoveDist),
	_ptzTiltX(ptzTiltX),
	_ptzTiltZ(ptzTiltZ),
	_ptzConeError(ptzConeError),
	_opteraTiltX(opteraTiltX),
	_opteraTiltZ(opteraTiltZ),
	_opteraConeError(opteraConeError),
    _thisIsSitAwareCam(thisIsSitAwareCam)
{
    rtspClientCount = 0;

    // This is largely copied from RtpFrameSource.cpp - this might make more sense in the base class someday
    //  Do not use the _stream like in RtpFrameSource though
    try {
        size_t count = 0;

        if (connectAsNonOpteraType == true) {
            string videoURLSubStr = "/stream1";
            if ((encoderChannelSelected > 0) && (encoderChannelSelected < 17)) {
                videoURLSubStr = "/channel";
                videoURLSubStr += to_string(encoderChannelSelected);
                videoURLSubStr += "/stream1";
            }
            if (_discoveryURLs.size() != 0) {
                bool found = false;
                size_t pos;
                for (size_t i = 0; i < _discoveryURLs.size(); ++i) {
                    pos = _discoveryURLs[i].find(_cameraIP);
                    if (pos != string::npos) {
                        videoURLSubStr = _discoveryURLs[i].substr(pos + _cameraIP.size());
                        pos = videoURLSubStr.rfind("_v");
                        videoURLSubStr = videoURLSubStr.substr(0, pos); //trim video only ending so vid and audio pass...
                        if (primaryStream == false) {
                            pos = videoURLSubStr.find("stream2");
                            if (pos != string::npos) {
                                found = true;
                            }
                        }
                        else {
                            pos = videoURLSubStr.find("stream1");
                            if (pos != string::npos) {
                                found = true;
                            }
                        }
                        if (found == true) {
                            break; // get out of for loop we have our substr
                        }
                    }
                }
                if (found == false) {
                    videoURLSubStr = "/stream1"; //default
                }
            }
            else if (primaryStream == false) {
                videoURLSubStr = "/stream2";
                if ((encoderChannelSelected > 0) && (encoderChannelSelected < 17)) {
                    videoURLSubStr = "/channel";
                    videoURLSubStr += to_string(encoderChannelSelected);
                    videoURLSubStr += "/stream2";
                }
            }
            string videoLayoutMetadata = "v1.0, 1, 4, 0, 0, 1, 1, 0, 0, 1, 1";
            _mediaDescriptorsInfo.push_back({ videoURLSubStr, videoLayoutMetadata });
            count = 1;
        } 
        else {
            // Optera Case
            std::string url;
            if (highRes == true) {
                if (uniStream == false) {
                    url = (string("rtsp://") + _cameraIP + "/immersive");
                }
                else {
                    url = (string("rtsp://") + _cameraIP + "/stream1");
                }
            }
            else {
                url = (string("rtsp://") + _cameraIP + "/mosaic");
            }
            GetStreamLayoutMetadatas(url, _mediaDescriptorsInfo);
            size_t count = _mediaDescriptorsInfo.size();
            if (_useUnistream == true) {
                _mediaDescriptorsInfo[0].rtspSubURL = "/stream1";
            }
            if (count == 0) {
                throw runtime_error("No video count in DESCRIBE call");
            }
            else if ((count > kMaxDragonFlyVStreams) || ((count < 2) && highRes && (uniStream == false))) {
                throw runtime_error("Unexpected stream count from DESCRIBE call");
            }
            if (_useSitAwareness == true) {
                _situationTracking = tracking;
                // ptz ip address
				_situationTracking->setPtzIpAddress(_ptzIp);
				_situationTracking->_cameraInfos.setInstallHeight(_opteraHeight, _ptzHeight);
				_situationTracking->_cameraInfos.setConeError((float)DEG2RAD(_opteraConeError), (float)DEG2RAD(_ptzConeError));
				_situationTracking->_cameraInfos.setDistanceToOptera(_distanceToOptera);
				_situationTracking->_cameraInfos.setSeparatedCameras(_separatedCameras);
				if (_minMoveDistance != 0) {
					_situationTracking->_cameraInfos.setMinMoveDistance(NULL, _minMoveDistance);  // using just the "ground" minMoveDist
				}
				_situationTracking->_cameraInfos.setPtzTilt((float)DEG2RAD(_ptzTiltX), (float)DEG2RAD(_ptzTiltZ));
				_situationTracking->_cameraInfos.setOpteraTilt((float)DEG2RAD(_opteraTiltX), (float)DEG2RAD(_opteraTiltZ));
                // set calibration params
				float ptzGlobalPosX = 0, ptzGlobalPosZ = 0, ptzRefPan = 0;
				_situationTracking->_options.getCalibrationParams(&ptzGlobalPosX, &ptzGlobalPosZ, &ptzRefPan, NULL, NULL, NULL, NULL, NULL, NULL);
				_situationTracking->_cameraInfos._cameraSpectra.setReferencePosX(ptzGlobalPosX);
				_situationTracking->_cameraInfos._cameraSpectra.setReferencePosZ(ptzGlobalPosZ);
				_situationTracking->_cameraInfos._cameraSpectra.setReferencePan(ptzRefPan);
            }
        }

        for (size_t i = 0; i < count; i++) {
            // we have a 180 dragonfly/optera camera
            if (connectAsNonOpteraType == false) {
                if ((uniStream == true) && (highRes == true)) {
                    _mediaDescriptorsInfo[i].rtspSubURL = "/stream1";
                }
                if (multicast == true) {
                    if (count == 1) {
                        _mediaDescriptorsInfo[i].rtspSubURL.append("m");
                    }
                    else {
                        std::size_t pos = _mediaDescriptorsInfo[i].rtspSubURL.rfind("_v");
                        _mediaDescriptorsInfo[i].rtspSubURL.replace(pos, 2, "m_v");
                    }
                }
            }
        }
	}
	catch (...) {
		cleanup();
		throw;
	}

}

//////////////////////////////////////////////////////////////////////////////////
OpenH264FrameSource::~OpenH264FrameSource() {
	cleanup();
}

//////////////////////////////////////////////////////////////////////////////////
void OpenH264FrameSource::cleanup() {
}

bool OpenH264FrameSource::hasUserMetadataText() {
    if (_metaDataString.length() > 0) {
        return true;
    }
    return false;
}

std::string OpenH264FrameSource::userMetadataText() {
    return _metaDataString;
}

//////////////////////////////////////////////////////////////////////////////////
void OpenH264FrameSource::start()
{
	_stopNow = false;

	// start thread
	HANDLE threadHandle = NULL;
	DWORD  threadId = 0;

    // Initialize all of the time buffers and
    //   other associated data
    for (int i = 0; i < (kMaxDragonFlyVStreams + 1); i++) {
        _decodingCanStart[i] = false;
        _timeBuffer[i].currentTimeBufferBeingUsed = 0;
        _timeBuffer[i].currentTimeBufferIsIFrame = false;
        _timeBuffer[i].currentTimeBufferTimeStored = false;
        // _decoderThreadStarted[i].lock();
        //_decoderThreadStarted.lock();
		for (int j = 0; j < kNumberTimeFrameBuffers; j++) {
            _timeBuffer[i].usedByNetworkInput[j] = false;
            _timeBuffer[i].usedByDecode[j] = false;
			_timeBuffer[i].usedByCopyThread[j] = false;
            _timeBuffer[i].requiresDecoding[j] = false;
            _timeBuffer[i].isDecoded[j] = false;
			_timeBuffer[i].time[j] = 0.0;
            _timeBuffer[i].indexToNetworkInputBuffer[j] = 0;
            _timeBuffer[i].decoderInputBuffer[j] = NULL;
            _timeBuffer[i].isIFrame[j] = false;
            for (int k = 0; k < 3; k++) {
                _timeBuffer[i].pictureInfo[j][k].data = NULL;
                _timeBuffer[i].pictureInfo[j][k].height = 0;
                _timeBuffer[i].pictureInfo[j][k].stride = 0;
                _timeBuffer[i].pictureInfo[j][k].width = 0;
            }
		}
	}

    // Start a message queue here so we can keep the threads
    //   started in order -decoder has be be done before network
    //
    MSG msg;
    PeekMessage(&msg, NULL, WM_USER, WM_USER, PM_NOREMOVE);

	//
	//  Note - at one time I tried to use one thread to decode the data, but his caused
	//    the PC to drop packets.  The RTP needs to be read more often - so using as many threads 
	//    as cubefaces
	//
	for (int i = 0; i < _mediaDescriptorsInfo.size(); i++) {
		_threadInfo[i].streamSetCallbackPtr = _setupCallback;
		_threadInfo[i].getBufferCallbackPtr = _getBufferCallback;
		_threadInfo[i].releaseBufferCallbackPtr = _releaseBufferCallback;
		_threadInfo[i].numberCubeFaces = _mediaDescriptorsInfo.size();
		_threadInfo[i].cubeFaceIndex = i;
        if (_thisIsSitAwareCam == true) {
            _threadInfo[i].cubeFaceIndex = 6;
        }
		_threadInfo[i].layoutInfo = _mediaDescriptorsInfo[i].layoutMetadata;
		_threadInfo[i].rtspURL = (string("rtsp://") + _cameraIP + _mediaDescriptorsInfo[i].rtspSubURL);
        _threadInfo[i].startingThreadID = GetCurrentThreadId();
        _threadInfo[i].useSitAwareness = _useSitAwareness;
        _networkInputThreadStopped[i] = false;
        _decoderThreadStopped[i] = false;
        if (_thisIsSitAwareCam == true) {
            _threadInfo[6].layoutInfo = _mediaDescriptorsInfo[i].layoutMetadata;
            _threadInfo[6].streamSetCallbackPtr = _setupCallback;
            _threadInfo[6].getBufferCallbackPtr = _getBufferCallback;
            _threadInfo[6].releaseBufferCallbackPtr = _releaseBufferCallback;
            _threadInfo[6].numberCubeFaces = _mediaDescriptorsInfo.size();
            _threadInfo[6].rtspURL = (string("rtsp://") + _cameraIP + _mediaDescriptorsInfo[i].rtspSubURL);
            _threadInfo[6].startingThreadID = GetCurrentThreadId();
            _threadInfo[6].useSitAwareness = _useSitAwareness;
            _networkInputThreadStopped[6] = false;
            _decoderThreadStopped[6] = false;
        }
        threadHandle = CreateThread(NULL, 0, decoderThread, (LPVOID)&(_threadInfo[i]), 0, &threadId);
        _timeBuffer[i].deocderThreadID = threadId;
        if (_thisIsSitAwareCam == true) {
            _timeBuffer[6].deocderThreadID = threadId;
        }
        // Need all of the decoder threads to make a message queue before
        //   starting any network threads
        GetMessage(&msg, NULL, 0, 0);
        threadHandle = CreateThread(NULL, 0, networkInputThread, (LPVOID)&(_threadInfo[i]), 0, &threadId);
    }

	CreateThread(NULL, 0, copyThread, (LPVOID)&(_threadInfo[0]), 0, &threadId);

	return;
}

//////////////////////////////////////////////////////////////////////////////////
void OpenH264FrameSource::stop()
{
	// This is a very simple and sloppy way to stop the start loop
	_stopNow = true;

    // Wake up copy thread to make sure it gets the stop
    std::unique_lock<std::mutex> locker(_copyThreadLock);
    _wakeUpCopyThread.notify_one();

    // Wake up decoder thread to make sure it gets the stop
    for (int i = 0; i < _mediaDescriptorsInfo.size(); i++) {
        PostThreadMessage(_timeBuffer[i].deocderThreadID, 0, kDecoderMessageQuit, 0);
    }

    for (int i = 0; i < _mediaDescriptorsInfo.size(); i++) {
        if (_rtspClients[i] != NULL) {
            shutdownStream(_rtspClients[i]);
        }
    }

	// Do not return until all the threads have checked in as 'stopped'
	FOREVER {
		for (int i = 0; i < _mediaDescriptorsInfo.size(); i++) {
            if ((_networkInputThreadStopped[i] == false) || (_decoderThreadStopped[i] == false) || _copyThreadStopped == false) {
				// in milliseconds
				Sleep(100);
				continue;
			}
		}
		// All threads should be stopped if here
		break;
	}
	// Even with the last command setting the threadStopped variable
	//   the class can still disappear.  So sleep and hope
	// Needs to be cleaned up
	Sleep(100);
}

DWORD WINAPI OpenH264FrameSource::decoderThread(LPVOID lpParam)
{
    STREAM_THREAD_INFO * pThreadInfo = (STREAM_THREAD_INFO *)lpParam;
    int cubeFaceIndex = pThreadInfo->cubeFaceIndex;

    //
    //   DECODER
    // 
    // Need to get the decoder ready to go at the beginning of the thread
    //
    //  Will continue to use the same decoder per thread since there is state
    //   information in the decoding process
    //
    int iLevelSetting = (int)WELS_LOG_WARNING;

    SDecodingParam * sDecParam = &_timeBuffer[cubeFaceIndex].sDecParam;
    memset(sDecParam, 0, sizeof(SDecodingParam));
    sDecParam->eOutputColorFormat = videoFormatI420;
    // sDecParam->eOutputColorFormat = videoFormatNV12;
    sDecParam->uiTargetDqLayer = (uint8_t)-1;
    sDecParam->eEcActiveIdc = ERROR_CON_SLICE_COPY;
    sDecParam->sVideoProperty.eVideoBsType = VIDEO_BITSTREAM_DEFAULT;

    sDecParam->sVideoProperty.size = sizeof(sDecParam->sVideoProperty);

    ISVCDecoder* pDecoder = NULL;
    if (WelsCreateDecoder(&pDecoder) || (NULL == pDecoder)) {
        throw runtime_error("Create Decoder failed.");
        return 1;
    }
    if (iLevelSetting >= 0) {
        pDecoder->SetOption(DECODER_OPTION_TRACE_LEVEL, &iLevelSetting);
    }

    if (pDecoder->Initialize(sDecParam)) {
        throw runtime_error("Decoder initialization failed.");
        return 1;
    }

    int32_t iErrorConMethod = (int32_t)ERROR_CON_SLICE_MV_COPY_CROSS_IDR_FREEZE_RES_CHANGE;
    pDecoder->SetOption(DECODER_OPTION_ERROR_CON_IDC, &iErrorConMethod);

    int32_t iColorFormat = videoFormatInternal;
    if (pDecoder->SetOption(DECODER_OPTION_DATAFORMAT, &iColorFormat)) {
        throw runtime_error("Decoder set option.");
        return 1;
    }

    _timeBuffer[cubeFaceIndex].pDecoder = pDecoder;

    // Create a message Queue
    MSG msg;
    PeekMessage(&msg, NULL, WM_USER, WM_USER, PM_NOREMOVE);

    // No messages should be sent until after the message queue has been created
    //   Use a message queue for this (mutex does not work across threads)
    PostThreadMessage(pThreadInfo->startingThreadID, 0, kDecoderMessageQuit, 0);

    BOOL bRet;
    //    while (true) {
    while ((bRet = GetMessage(&msg, NULL, 0, 0)) != 0)
    {
        if (bRet == -1)
        {
            // handle the error and possibly exit
            break;
        }
        if (_stopNow == true) {
            break;
        }

        // To figure out which info to actually decode use:
        //   1. cubeFaceIndex
        //   2. look at requiresDecoding
        // 
        //  Will find the latest deocding for now - this means 
        //   that if we get too busy, some input buffers may never 
        //   be decoded
        //
        _timeBufferCheckMutex.lock();

        double maxTime = DBL_MAX;
        int latestIndex = 0;
        volatile int numberOfFramesNeedingDecode = 0;
        for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
            if ( (_timeBuffer[cubeFaceIndex].requiresDecoding[i] == true) && 
                (_timeBuffer[cubeFaceIndex].usedByCopyThread[i] == false) &&
                (_timeBuffer[cubeFaceIndex].usedByNetworkInput[i] == false) )
            {
                // find smallest time
                if (_timeBuffer[cubeFaceIndex].time[i] < maxTime)
                {
                    maxTime = _timeBuffer[cubeFaceIndex].time[i];
                    latestIndex = i;
                    numberOfFramesNeedingDecode++;
                }
            }
        }
        _timeBuffer[cubeFaceIndex].usedByDecode[latestIndex] = true;
        _timeBufferCheckMutex.unlock();
        if (numberOfFramesNeedingDecode > 1) {
            DBOUT("\n\nOut of Processor?  Cubeface: " << cubeFaceIndex << " Decode " << numberOfFramesNeedingDecode << " frame\n\n\n");
        }

        bool rv = DecodeNextImage(cubeFaceIndex, &_timeBuffer[cubeFaceIndex], _timeBuffer[cubeFaceIndex].decoderInputBuffer[latestIndex], latestIndex);
        _timeBufferCheckMutex.lock();
        _timeBuffer[cubeFaceIndex].usedByDecode[latestIndex] = false;
        _timeBuffer[cubeFaceIndex].requiresDecoding[latestIndex] = false;
        if (rv == true) {
        // if (0 == 0) {
            _timeBuffer[cubeFaceIndex].isDecoded[latestIndex] = true;
            vector<ObjectRect>& analyticObjs = _timeBuffer[cubeFaceIndex].analyticObjects[latestIndex];
            if (!analyticObjs.empty()) {
                // drawing 
                DrawAnalyticObject(_timeBuffer[cubeFaceIndex].pictureInfo[latestIndex][0], analyticObjs);
            }
            if (pThreadInfo->useSitAwareness == true) {
                _situationTracking->movePtz(analyticObjs);
            }
        } 
        else {
            _timeBuffer[cubeFaceIndex].isDecoded[latestIndex] = false;
        }
        _timeBufferCheckMutex.unlock();

        std::unique_lock<std::mutex> locker(_copyThreadLock);
        if (rv == true) {
            _wakeUpCopyThread.notify_one();
        }
    }
    // Clean up Decoder
    if (pDecoder) {
        pDecoder->Uninitialize();
        WelsDestroyDecoder(pDecoder);
    }
    if (sDecParam->pFileNameRestructed != NULL) {
        delete[]sDecParam->pFileNameRestructed;
        sDecParam->pFileNameRestructed = NULL;
    }

    // Free the memory that might have been allocated
    for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
        if (_timeBuffer[cubeFaceIndex].pictureInfo[i][0].data != NULL) {
            HeapFree(GetProcessHeap(), HEAP_NO_SERIALIZE, _timeBuffer[cubeFaceIndex].pictureInfo[i][0].data);
            _timeBuffer[cubeFaceIndex].pictureInfo[i][0].data = NULL;
        }
        if (_timeBuffer[cubeFaceIndex].pictureInfo[i][1].data != NULL) {
            HeapFree(GetProcessHeap(), HEAP_NO_SERIALIZE, _timeBuffer[cubeFaceIndex].pictureInfo[i][1].data);
            _timeBuffer[cubeFaceIndex].pictureInfo[i][1].data = NULL;
        }
        if (_timeBuffer[cubeFaceIndex].pictureInfo[i][2].data != NULL) {
            HeapFree(GetProcessHeap(), HEAP_NO_SERIALIZE, _timeBuffer[cubeFaceIndex].pictureInfo[i][2].data);
            _timeBuffer[cubeFaceIndex].pictureInfo[i][2].data = NULL;
        }
    }

    _decoderThreadStopped[cubeFaceIndex] = true;
    return 0;
}

char eventLoopWatchVariable = 0;
DWORD WINAPI OpenH264FrameSource::networkInputThread(LPVOID lpParam)
{
    STREAM_THREAD_INFO * pThreadInfo = (STREAM_THREAD_INFO *)lpParam;
    int cubeFaceIndex = pThreadInfo->cubeFaceIndex;

    // Allocate all the input buffers here
    for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
        _timeBuffer[cubeFaceIndex].decoderInputBuffer[i] = (unsigned char *)HeapAlloc(GetProcessHeap(), HEAP_NO_SERIALIZE, kMaxSizeOfDecoderInput);
        if (_timeBuffer[cubeFaceIndex].decoderInputBuffer[i] == NULL) {
            throw runtime_error("Out of Memory: Could not allocate netowrk input buffer");
        }
    }

    // STREAM
    //
    // Mostly copied by testRTSPClient.cpp of live555 code
    // Begin by setting up our usage environment:
    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    UsageEnvironment* env = BasicUsageEnvironment::createNew(*scheduler);
    string programName = "Stream" + pThreadInfo->cubeFaceIndex;
    _rtspClients[pThreadInfo->cubeFaceIndex] = openURL(*env, programName.c_str(), pThreadInfo->rtspURL.c_str(), pThreadInfo->cubeFaceIndex );

    // All subsequent activity takes place within the event loop:
    eventLoopWatchVariable = 0;
    env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
    // This function call does not return, unless, at some point in time, "eventLoopWatchVariable" gets set to something non-zero.
    // This happens when 'shutdownStream' is called by 'stop' - which happens when the app closes
    //   or you start a different stream in the app

    // Free the buffers from above
    for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
        BOOL rv = HeapFree(GetProcessHeap(), 0, _timeBuffer[cubeFaceIndex].decoderInputBuffer[i]);
        if (rv == FALSE) {
            throw runtime_error("Out of Memory: Could not allocate netowrk input buffer");
        }
    }

    _networkInputThreadStopped[pThreadInfo->cubeFaceIndex] = true;
    return 0;
}

// This thead will wake up when it is time to check buffers.  
//   If the times all match - it will copy the data for showing on display
DWORD WINAPI OpenH264FrameSource::copyThread(LPVOID lpParam) {
    STREAM_THREAD_INFO * pThreadInfo = (STREAM_THREAD_INFO *)lpParam;
    size_t numberCubeFaces = pThreadInfo->numberCubeFaces;
    double lastMatchTime = 0.0;
    int cubeFaceMatchedIndex[6] = { 0 };
    FOREVER
    {
        if (_stopNow == true) {
            break;
        }
        {
            unique_lock<mutex> locker(_copyThreadLock);
            _wakeUpCopyThread.wait(locker);
        }
        if (_stopNow == true) {
            break;
        }
        // Woke up - check for data  in buffers with matching times that
        //   are also decoded
        bool haveMatch = false;

        for (int i = 0; i < numberCubeFaces; i++) {
            cubeFaceMatchedIndex[i] = -1;
        }
        // Have kNumberFrameBuffers by numberCubeFaces 2d array to compare
        //   Any match in all of the numberCubeFaces columns is good and can be used
        //     Note - there can be more than one match
        //            only care about times greater than the last match
        //
        const double kJitterAllowed = 0.004;
        _timeBufferCheckMutex.lock();
        // Handle up to six cube faces
        double largestTime = 0.0;
        if (numberCubeFaces == 1) {
            for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
                // Find the largest number that is decoded (at least one should be since the decoder
                //    starts this check)
                if (_timeBuffer[0].isDecoded[i] == true) {
                    if (largestTime < _timeBuffer[0].time[i]) {
                        cubeFaceMatchedIndex[0] = i;
                        largestTime = _timeBuffer[0].time[i];
                    }
                }
            }
        }
        for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
            if (numberCubeFaces == 1) {
                // Already done in this case, so leave
                break;
            }
            for (int j = 0; j < kNumberTimeFrameBuffers; j++) {
                if ((_timeBuffer[0].time[i] == _timeBuffer[1].time[j]) &&
                    (_timeBuffer[0].time[i] > lastMatchTime) &&
                    (_timeBuffer[0].usedByDecode[i] == false) &&
                    (_timeBuffer[1].usedByDecode[j] == false) &&
                    (_timeBuffer[0].isDecoded[i] == true) &&
                    (_timeBuffer[1].isDecoded[j] == true) ) {
                    if (numberCubeFaces == 2) {
                        // DONE!
                        cubeFaceMatchedIndex[0] = i;
                        cubeFaceMatchedIndex[1] = j;
                        break;
                    }
                    for (int k = 0; k < kNumberTimeFrameBuffers; k++) {
                        if ((abs(_timeBuffer[0].time[i] - _timeBuffer[2].time[k]) < kJitterAllowed) && (_timeBuffer[2].usedByDecode[k] == false) && (_timeBuffer[2].isDecoded[k] == true)) {
                            if (numberCubeFaces == 3) {
                                // DONE!
                                cubeFaceMatchedIndex[0] = i;
                                cubeFaceMatchedIndex[1] = j;
                                cubeFaceMatchedIndex[2] = k;
                                break;
                            }
                            for (int l = 0; l < kNumberTimeFrameBuffers; l++) {
                                if ((abs(_timeBuffer[0].time[i] - _timeBuffer[3].time[l]) < kJitterAllowed) && (_timeBuffer[3].usedByDecode[l] == false) && (_timeBuffer[3].isDecoded[l] == true)) {
                                    if (numberCubeFaces == 4) {
                                        // DONE!
                                        cubeFaceMatchedIndex[0] = i;
                                        cubeFaceMatchedIndex[1] = j;
                                        cubeFaceMatchedIndex[2] = k;
                                        cubeFaceMatchedIndex[3] = l;
                                        break;
                                    }
                                    for (int m = 0; m < kNumberTimeFrameBuffers; m++) {
                                        if ((abs(_timeBuffer[0].time[i] - _timeBuffer[4].time[m]) < kJitterAllowed) && (_timeBuffer[4].usedByDecode[m] == false) && (_timeBuffer[4].isDecoded[m] == true)) {
                                            if (numberCubeFaces == 5) {
                                                // DONE!
                                                cubeFaceMatchedIndex[0] = i;
                                                cubeFaceMatchedIndex[1] = j;
                                                cubeFaceMatchedIndex[2] = k;
                                                cubeFaceMatchedIndex[3] = l;
                                                cubeFaceMatchedIndex[4] = m;
                                                break;
                                            }
                                        }
                                        for (int n = 0; n < kNumberTimeFrameBuffers; n++) {
                                            if ((abs(_timeBuffer[0].time[i] - _timeBuffer[5].time[n]) < kJitterAllowed) && (_timeBuffer[5].usedByDecode[n] == false) && (_timeBuffer[5].isDecoded[n] == true)) {
                                                if (numberCubeFaces == 6) {
                                                    // DONE!
                                                    cubeFaceMatchedIndex[0] = i;
                                                    cubeFaceMatchedIndex[1] = j;
                                                    cubeFaceMatchedIndex[2] = k;
                                                    cubeFaceMatchedIndex[3] = l;
                                                    cubeFaceMatchedIndex[4] = m;
                                                    cubeFaceMatchedIndex[5] = n;
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (cubeFaceMatchedIndex[0] != -1)
        {
            for (int i = 0; i < numberCubeFaces; i++) {
                _timeBuffer[i].usedByCopyThread[cubeFaceMatchedIndex[i]] = true;
            }
        }
        _timeBufferCheckMutex.unlock();

        if (cubeFaceMatchedIndex[0] != -1) {
            DWORD startTick = GetTickCount();
            for (int i = 0; i < numberCubeFaces; i++) {
                unsigned char *bitmapBuffer[3];

                pThreadInfo->getBufferCallbackPtr(i, bitmapBuffer[0], bitmapBuffer[1], bitmapBuffer[2]);

                for (int j = 0; j < 3; j++) {
                    PLANE_INFO * planeInfo = &_timeBuffer[i].pictureInfo[cubeFaceMatchedIndex[i]][j];
                    if (planeInfo->stride == planeInfo->width) {
                        memcpy(bitmapBuffer[j], planeInfo->data, planeInfo->width * planeInfo->height);
                    }
                    else {
                        for (int k = 0; k < planeInfo->height; k++) {
                            // Go row by row
                            memcpy(bitmapBuffer[j] + (k * planeInfo->width), planeInfo->data + (k * planeInfo->stride), planeInfo->width);
                        }
                    }
                }

                bool show = (i == numberCubeFaces - 1) ? true : false;
                pThreadInfo->releaseBufferCallbackPtr(i, bitmapBuffer[0], bitmapBuffer[1], bitmapBuffer[2], show, 0.0, 0.0);
            }
            DWORD stopTick = GetTickCount();
            DBOUT("Ready to show Data.  Time:  " << _timeBuffer[0].time[cubeFaceMatchedIndex[0]] << "Time to copy:  " << stopTick - startTick << "\n");

            lastMatchTime = _timeBuffer[0].time[cubeFaceMatchedIndex[0]];

            _timeBufferCheckMutex.lock();
            for (int i = 0; i < numberCubeFaces; i++) {
                _timeBuffer[i].usedByCopyThread[cubeFaceMatchedIndex[i]] = false;
            }
            _timeBufferCheckMutex.unlock();
        }

    } // end of FOREVER
    _copyThreadStopped = true;
    return 0;
}

#ifdef _WIN32
// Need htohl for byte-swapping.  Windows has this, but they make you link
// against winsock2.  In order to minimize dependencies, I'd rather just
// implement this here.
uint32_t ntohl(uint32_t in) {
    // assuming all WIN32 is little-endian
    return
        ((in << 24) & 0xFF000000) |
        ((in << 8) & 0x00FF0000) |
        ((in >> 8) & 0x0000FF00) |
        ((in >> 24) & 0x000000FF);
}
#else
#include <arpa/inet.h>
#endif

double OpenH264FrameSource::FindPelcoMetaData(unsigned char * readBuffer, int bufferSize, string & textMetadata, vector<ObjectRect>& analyticObjects)
{
    // For comparison
    const int layoutBufferSize = 500;
    const int osdBufferSize = 500;
    int layoutBufferUsed = 0;
    int osdBufferUsed = 0;
    char pLayoutBuffer[layoutBufferSize];
    char pOsdBuffer[osdBufferSize];
    pelco_imextk_extract_metadata_from_H264(readBuffer, bufferSize, pLayoutBuffer, layoutBufferSize, &layoutBufferUsed, pOsdBuffer, osdBufferSize, &osdBufferUsed);

    textMetadata.clear();
    analyticObjects.clear();
    int index = 0;
    for (int index = 0; index < bufferSize - 4;) {

        if ((readBuffer[index + 0] == 0) &&
            (readBuffer[index + 1] == 0) &&
            (readBuffer[index + 2] == 0) &&
            (readBuffer[index + 3] == 1))
        {
            // Start of NAL unit
            volatile unsigned char type1 = readBuffer[index + 4];
            volatile unsigned char type2 = readBuffer[index + 5];
            index += 6;
        }
        else {
            //  index++;
        }

        // Find a 'PLCO' 
        if ((readBuffer[index + 0] == 'P') &&
            (readBuffer[index + 1] == 'L') &&
            (readBuffer[index + 2] == 'C') &&
            (readBuffer[index + 3] == 'O')) {

            int startOfPinPLCOIndex = index;

            // The next four bytes will be the size of the buffer
            //  But I think this size in reference to the non RLE size - not the actual buffer 
            //  size after you deRLE it.  This size is also offset by 0x200 to prevent emulation
            //   of start codes (ie. will not be three zeros in a row)
            uint32_t sizeOfUserDataBeforeUnRLE = ntohl(*reinterpret_cast<const uint32_t*>(&readBuffer[startOfPinPLCOIndex + 4]));
            sizeOfUserDataBeforeUnRLE -= 0x200;


            // The rest of the data is "RLE" encoded kind of
            //   Let's just make a new buffer with that stuff all gone
            //   Since there can be motion data which can be really big, this 
            //   buffer could explode
            //
            int sizeOfUnRLEBuffer = bufferSize * 10;
            unsigned char * pUnRLEBuffer = new unsigned char[sizeOfUnRLEBuffer];
            // Assert happens if the result is FALSE
            assert(pUnRLEBuffer != NULL);

            int indexToMakeUnRLEBuffer = 0;
            // buffer-4 because the last part of the buffer is always 80808000
            for (int i = startOfPinPLCOIndex + 8; i < bufferSize - 4; ) {
                if (readBuffer[i] == 0) {
                    pUnRLEBuffer[indexToMakeUnRLEBuffer++] = 0;
                    if (indexToMakeUnRLEBuffer == sizeOfUnRLEBuffer) {
                        break;
                    }
                    if ((readBuffer[i + 1] != 0) && (readBuffer[i + 1] != 1)) {
                        for (int j = 0; j < (readBuffer[i + 1] - 1); j++) {
                            pUnRLEBuffer[indexToMakeUnRLEBuffer++] = 0;
                            if (indexToMakeUnRLEBuffer == sizeOfUnRLEBuffer) {
                                break;
                            }
                        }
                    }
                    i += 2;
                }
                else {
                    pUnRLEBuffer[indexToMakeUnRLEBuffer++] = readBuffer[i++];
                    if (indexToMakeUnRLEBuffer == sizeOfUnRLEBuffer) {
                        break;
                    }
                }
            }
            // Now that we have a unRLE buffer, find the next things to find
            //  Next four bytes are data type, with 4 bytes of length after it
            //  Then length is the size of the data WITHOUT the type and length
            // all known packet h264 Pelco user-data type IDs
            // (Only TextMetadata and VideoLayoutMetadata are used by the ImExTk)
            static const uint32_t kUserDataTypeTimestamp = 0x00000280;
            static const uint32_t kUserDataTypeMotionData = 0x000002F8;
            static const uint32_t kUserDataTypeSecurityToken = 0x0000028A;
            static const uint32_t kUserDataTypeDrawingData = 0x000005B4;
            static const uint32_t kUserDataTextMetadata = 0x000005D2;
            static const uint32_t kUserDataVideoLayoutMetadata = 0x000005D3;

            // The next four bytes will be the size of the buffer
            //  But I think this size in reference to the non RLE size - not the actual buffer 
            //  size after you deRLE it

            uint32_t seconds = 0;
            uint32_t microSeconds = 0;
            uint32_t size = 0;
            for (int i = 0; i < indexToMakeUnRLEBuffer; ) {
                uint32_t userDataType = ntohl(*reinterpret_cast<const uint32_t*>(&pUnRLEBuffer[i]));
                i += 4;
                switch (userDataType) {
                    case kUserDataTypeTimestamp:
                        // assert happens if the expression is FALSE
                        size = ntohl(*reinterpret_cast<const uint32_t*>(&pUnRLEBuffer[i]));
                        assert(size == 12);
                        i += 4;
                        // First four bytes are seconds
                        // Next four bytes are microsends
                        seconds = ntohl(*reinterpret_cast<const uint32_t*>(&pUnRLEBuffer[i]));
                        i += 4;
                        microSeconds = ntohl(*reinterpret_cast<const uint32_t*>(&pUnRLEBuffer[i]));
                        assert(microSeconds < 1000000);
                        i += 4;
                        // Don't care about timezone and dst so skip (2 bytes each)
                        i += 4;
                        break;
                    case kUserDataTextMetadata:
                        size = ntohl(*reinterpret_cast<const uint32_t*>(&pUnRLEBuffer[i]));
                        i += 4;
                        textMetadata.clear();
                        for (int j = 0; j < (int) size; j++) {
                            // Don't know why - but there are two zeros at the beginning
                            if (pUnRLEBuffer[i + j] != 0) {
                                textMetadata += pUnRLEBuffer[i + j];
                            }
                        }
                        i += size;
                        break;
                    case kUserDataTypeDrawingData:
                        size = ntohl(*reinterpret_cast<const uint32_t*>(&pUnRLEBuffer[i]));
                        parseObjects(pUnRLEBuffer + i + 4, (int) size, analyticObjects);
                        i += (size + 4);
                        break;
                    default:
                        size = ntohl(*reinterpret_cast<const uint32_t*>(&pUnRLEBuffer[i]));
                        i += (size + 4);
                        break;
                }
            }
            delete [] pUnRLEBuffer;
/*
            // Okay - now find the time which
            // should always be 12 bytes away from the 'O'

            unsigned int seconds = 0;
            unsigned int microSeconds = 0;
            int indexFromPLCO = 15;
            // Need to UN-RLE the zeros in the data
            int bytesRead = 0;
            for (; bytesRead < 4;) {
                unsigned int testByte = readBuffer[index + indexFromPLCO];
                if (testByte == 0) {
                    int numZeros = readBuffer[index + indexFromPLCO + 1];
                    bytesRead += numZeros;
                    indexFromPLCO += 2;
                }
                else {
                    seconds += (testByte << (8 * (3 - bytesRead)));
                    bytesRead++;
                    indexFromPLCO++;
                }
            }
            // there is a case where the last number(s) of seconds is zero as
            // well as the first numbers of microseconds.  Handle this case . . .
            bytesRead -= 4;
            for (; bytesRead < 4;) {
                unsigned int testByte = readBuffer[index + indexFromPLCO];
                if (testByte == 0) {
                    int numZeros = readBuffer[index + indexFromPLCO + 1];
                    bytesRead += numZeros;
                    indexFromPLCO += 2;
                }
                else {
                    microSeconds += (testByte << (8 * (3 - bytesRead)));
                    bytesRead++;
                    indexFromPLCO++;
                }
            }
            assert(microSeconds < 1000000);
*/
            double time = seconds;
            time += (microSeconds / 1000000.0);
            return time;
        } // End of IF PLCO
        else {
            index++;
        }
    } // End of FOR file loop 
    return 0.0;
}

///////////////////////////////////////////////////////////////////////////////
//
//
//
//  LIVE 555
//
//
//
///////////////////////////////////////////////////////////////////////////////

// Must be nonmembers 
// A function that outputs a string that identifies each stream (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env, const RTSPClient& rtspClient) {
    return env << "[URL:\"" << rtspClient.url() << "\"]: ";
}

// A function that outputs a string that identifies each subsession (for debugging output).  Modify this if you wish:
UsageEnvironment& operator<<(UsageEnvironment& env, const MediaSubsession& subsession) {
    return env << subsession.mediumName() << "/" << subsession.codecName();
}

OpenH264FrameSource::ourRTSPClient * OpenH264FrameSource::openURL(UsageEnvironment& env, char const* progName, char const* rtspURL, int cubeFaceIndex) {
    // Begin by creating a "RTSPClient" object.  Note that there is a separate "RTSPClient" object for each stream that we wish
    // to receive (even if more than stream uses the same "rtsp://" URL).
    ourRTSPClient* rtspClient = ourRTSPClient::createNew(env, rtspURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
    if (rtspClient == NULL) {
        env << "Failed to create a RTSP client for URL \"" << rtspURL << "\": " << env.getResultMsg() << "\n";
        return NULL;
    }
    rtspClient->cubeFaceIndex = cubeFaceIndex;

    ++rtspClientCount;

    // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the stream.
    // Note that this command - like all RTSP commands - is sent asynchronously; we do not block, waiting for a response.
    // Instead, the following function call returns immediately, and we handle the RTSP response later, from within the event loop:
    rtspClient->sendDescribeCommand(continueAfterDESCRIBE);
    return rtspClient;
}

// Implementation of the RTSP 'response handlers':

void OpenH264FrameSource::continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString) {
    do {
        UsageEnvironment& env = rtspClient->envir(); // alias
        StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

        if (resultCode != 0) {
            env << *rtspClient << "Failed to get a SDP description: " << resultString << "\n";
            delete[] resultString;
            break;
        }

        char* const sdpDescription = resultString;
        env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

        // Create a media session object from this SDP description:
        scs.session = MediaSession::createNew(env, sdpDescription);
        delete[] sdpDescription; // because we don't need it anymore
        if (scs.session == NULL) {
            env << *rtspClient << "Failed to create a MediaSession object from the SDP description: " << env.getResultMsg() << "\n";
            break;
        }
        else if (!scs.session->hasSubsessions()) {
            env << *rtspClient << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
            break;
        }

        // Then, create and set up our data source objects for the session.  We do this by iterating over the session's 'subsessions',
        // calling "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command, on each one.
        // (Each 'subsession' will have its own data source.)
        scs.iter = new MediaSubsessionIterator(*scs.session);
        setupNextSubsession(rtspClient);
        return;
    } while (0);

    // An unrecoverable error occurred with this stream.
    shutdownStream(rtspClient);
}

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP-over-TCP, change the following to True:
#define REQUEST_STREAMING_OVER_TCP False

void OpenH264FrameSource::setupNextSubsession(RTSPClient* rtspClient) {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

    scs.subsession = scs.iter->next();
    if (scs.subsession != NULL) {
        if (!scs.subsession->initiate()) {
            env << *rtspClient << "Failed to initiate the \"" << *scs.subsession << "\" subsession: " << env.getResultMsg() << "\n";
            setupNextSubsession(rtspClient); // give up on this subsession; go to the next one
        }
        else {
            env << *rtspClient << "Initiated the \"" << *scs.subsession << "\" subsession (";
            if (scs.subsession->rtcpIsMuxed()) {
                env << "client port " << scs.subsession->clientPortNum();
            }
            else {
                env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum() + 1;
            }
            env << ")\n";

            // Continue setting up this subsession, by sending a RTSP "SETUP" command:
            rtspClient->sendSetupCommand(*scs.subsession, continueAfterSETUP, False, REQUEST_STREAMING_OVER_TCP);
        }
        return;
    }

    // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY" command to start the streaming:
    if (scs.session->absStartTime() != NULL) {
        // Special case: The stream is indexed by 'absolute' time, so send an appropriate "PLAY" command:
        rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, scs.session->absStartTime(), scs.session->absEndTime());
    }
    else {
        scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
        rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY);
    }
}

void OpenH264FrameSource::continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString) {
    do {
        UsageEnvironment& env = rtspClient->envir(); // alias
        StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias
        int cubeFaceIndex = ((ourRTSPClient*)rtspClient)->cubeFaceIndex;

        if (resultCode != 0) {
            env << *rtspClient << "Failed to set up the \"" << *scs.subsession << "\" subsession: " << resultString << "\n";
            break;
        }

        env << *rtspClient << "Set up the \"" << *scs.subsession << "\" subsession (";
        if (scs.subsession->rtcpIsMuxed()) {
            env << "client port " << scs.subsession->clientPortNum();
        }
        else {
            env << "client ports " << scs.subsession->clientPortNum() << "-" << scs.subsession->clientPortNum() + 1;
        }
        env << ")\n";

        // Having successfully setup the subsession, create a data sink for it, and call "startPlaying()" on it.
        // (This will prepare the data sink to receive data; the actual flow of data from the client won't start happening until later,
        // after we've sent a RTSP "PLAY" command.)

        scs.subsession->sink = DummySink::createNew(env, *scs.subsession, cubeFaceIndex, rtspClient->url());
        // perhaps use your own custom "MediaSink" subclass instead
        if (scs.subsession->sink == NULL) {
            env << *rtspClient << "Failed to create a data sink for the \"" << *scs.subsession
                << "\" subsession: " << env.getResultMsg() << "\n";
            break;
        }

        env << *rtspClient << "Created a data sink for the \"" << *scs.subsession << "\" subsession\n";
        scs.subsession->miscPtr = rtspClient; // a hack to let subsession handler functions get the "RTSPClient" from the subsession 
        scs.subsession->sink->startPlaying(*(scs.subsession->readSource()),
            subsessionAfterPlaying, scs.subsession);
        // Also set a handler to be called if a RTCP "BYE" arrives for this subsession:
        if (scs.subsession->rtcpInstance() != NULL) {
            scs.subsession->rtcpInstance()->setByeHandler(subsessionByeHandler, scs.subsession);
        }
    } while (0);
    // delete[] resultString;

    // Set up the next subsession, if any:
    setupNextSubsession(rtspClient);
}

void OpenH264FrameSource::continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString) {
    Boolean success = False;

    do {
        UsageEnvironment& env = rtspClient->envir(); // alias
        StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

        if (resultCode != 0) {
            env << *rtspClient << "Failed to start playing session: " << resultString << "\n";
            break;
        }

        // Set a timer to be handled at the end of the stream's expected duration (if the stream does not already signal its end
        // using a RTCP "BYE").  This is optional.  If, instead, you want to keep the stream active - e.g., so you can later
        // 'seek' back within it and do another RTSP "PLAY" - then you can omit this code.
        // (Alternatively, if you don't want to receive the entire stream, you could set this timer for some shorter value.)
        if (scs.duration > 0) {
            unsigned const delaySlop = 2; // number of seconds extra to delay, after the stream's expected duration.  (This is optional.)
            scs.duration += delaySlop;
            unsigned uSecsToDelay = (unsigned)(scs.duration * 1000000);
            scs.streamTimerTask = env.taskScheduler().scheduleDelayedTask(uSecsToDelay, (TaskFunc*)streamTimerHandler, rtspClient);
        }

        env << *rtspClient << "Started playing session";
        if (scs.duration > 0) {
            env << " (for up to " << scs.duration << " seconds)";
        }
        env << "...\n";

        success = True;
    } while (0);
    delete[] resultString;

    if (!success) {
        // An unrecoverable error occurred with this stream.
        shutdownStream(rtspClient);
    }
}


// Implementation of the other event handlers:

void OpenH264FrameSource::subsessionAfterPlaying(void* clientData) {
    MediaSubsession* subsession = (MediaSubsession*)clientData;
    RTSPClient* rtspClient = (RTSPClient*)(subsession->miscPtr);

    // Begin by closing this subsession's stream:
    Medium::close(subsession->sink);
    subsession->sink = NULL;

    // Next, check whether *all* subsessions' streams have now been closed:
    MediaSession& session = subsession->parentSession();
    MediaSubsessionIterator iter(session);
    while ((subsession = iter.next()) != NULL) {
        if (subsession->sink != NULL) return; // this subsession is still active
    }

    // All subsessions' streams have now been closed, so shutdown the client:
    shutdownStream(rtspClient);
}

void OpenH264FrameSource::subsessionByeHandler(void* clientData) {
    MediaSubsession* subsession = (MediaSubsession*)clientData;
    RTSPClient* rtspClient = (RTSPClient*)subsession->miscPtr;
    UsageEnvironment& env = rtspClient->envir(); // alias

    env << *rtspClient << "Received RTCP \"BYE\" on \"" << *subsession << "\" subsession\n";

    // Now act as if the subsession had closed:
    subsessionAfterPlaying(subsession);
}

void OpenH264FrameSource::streamTimerHandler(void* clientData) {
    ourRTSPClient* rtspClient = (ourRTSPClient*)clientData;
    StreamClientState& scs = rtspClient->scs; // alias

    scs.streamTimerTask = NULL;

    // Shut down the stream:
    shutdownStream(rtspClient);
}

void OpenH264FrameSource::shutdownStream(RTSPClient* rtspClient, int exitCode) {
    UsageEnvironment& env = rtspClient->envir(); // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs; // alias

    // First, check whether any subsessions have still to be closed:
    if (scs.session != NULL) {
        Boolean someSubsessionsWereActive = False;
        MediaSubsessionIterator iter(*scs.session);
        MediaSubsession* subsession;

        while ((subsession = iter.next()) != NULL) {
            if (subsession->sink != NULL) {
                Medium::close(subsession->sink);
                subsession->sink = NULL;

                if (subsession->rtcpInstance() != NULL) {
                    subsession->rtcpInstance()->setByeHandler(NULL, NULL); // in case the server sends a RTCP "BYE" while handling "TEARDOWN"
                }

                someSubsessionsWereActive = True;
            }
        }

        if (someSubsessionsWereActive) {
            // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the stream.
            // Don't bother handling the response to the "TEARDOWN".
            rtspClient->sendTeardownCommand(*scs.session, NULL);
        }
    }

    env << *rtspClient << "Closing the stream.\n";
    Sleep(200);
    Medium::close(rtspClient);
    // Note that this will also cause this stream's "StreamClientState" structure to get reclaimed.

    // if (--rtspClientCount == 0) {
    if (1) {
        // The final stream has ended, so exit the application now.
        // (Of course, if you're embedding this code into your own application, you might want to comment this out,
        // and replace it with "eventLoopWatchVariable = 1;", so that we leave the LIVE555 event loop, and continue running "main()".)
        Sleep(50);
        eventLoopWatchVariable = 1;

        // exit(exitCode);
    }
}


// Implementation of "ourRTSPClient":

OpenH264FrameSource::ourRTSPClient* OpenH264FrameSource::ourRTSPClient::createNew(UsageEnvironment& env, char const* rtspURL,
    int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum) {
    return new ourRTSPClient(env, rtspURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum);
}

OpenH264FrameSource::ourRTSPClient::ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
    int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum)
    : RTSPClient(env, rtspURL, verbosityLevel, applicationName, tunnelOverHTTPPortNum, -1) {
}

OpenH264FrameSource::ourRTSPClient::~ourRTSPClient() {
}


// Implementation of "StreamClientState":

OpenH264FrameSource::StreamClientState::StreamClientState()
    : iter(NULL), session(NULL), subsession(NULL), streamTimerTask(NULL), duration(0.0) {
}

OpenH264FrameSource::StreamClientState::~StreamClientState() {
    delete iter;
    if (session != NULL) {
        // We also need to delete "session", and unschedule "streamTimerTask" (if set)
        UsageEnvironment& env = session->envir(); // alias

        env.taskScheduler().unscheduleDelayedTask(streamTimerTask);
        Medium::close(session);
    }
}


// Implementation of "DummySink":

// Even though we're not going to be doing anything with the incoming data, we still need to receive it.
// Define the size of the buffer that we'll use:
//  This needs to be big enough to handle an I-Frame data size
#define DUMMY_SINK_RECEIVE_BUFFER_SIZE (3 * 1024 * 1024)

OpenH264FrameSource::DummySink* OpenH264FrameSource::DummySink::createNew(UsageEnvironment& env, MediaSubsession& subsession, int cubeFaceIndex, char const* streamId) {
    return new DummySink(env, subsession, cubeFaceIndex, streamId);
}

OpenH264FrameSource::DummySink::DummySink(UsageEnvironment& env, MediaSubsession& subsession, int cubeFaceIndex, char const* streamId)
    : RTPSink(env, subsession.rtpSource()->RTPgs(), subsession.rtpSource()->rtpPayloadFormat(), subsession.rtpSource()->timestampFrequency(), streamId, subsession.numChannels()), // MediaSink(env),
      fCubeFaceIndex(cubeFaceIndex),
      fSubsession(subsession) {
    fStreamId = strDup(streamId);
    fReceiveBuffer = new u_int8_t[DUMMY_SINK_RECEIVE_BUFFER_SIZE];
    unsigned char payloadfjld = subsession.rtpSource()->rtpPayloadFormat();
}

OpenH264FrameSource::DummySink::~DummySink() {
    delete[] fReceiveBuffer;
    delete[] fStreamId;
}

// is this thread safe???
// I think so - but to be sure 
//   Well - there is a case where live 555 will call this right away 
//    in the 'continue playing function' - so the mutex will make it lock
void OpenH264FrameSource::DummySink::afterGettingFrame(void* clientData, unsigned frameSize, unsigned numTruncatedBytes,
                                                       struct timeval presentationTime, unsigned durationInMicroseconds) {
    if (clientData == NULL) {
        DBOUT("Received Sink of NULL \n");
        return;
    }
    DummySink* sink = (DummySink*)clientData;

    // This will point to the first byte after the RTP stuff
    //  This will be the NAL unit header or first byte of payload
    // From experimentation, an Optera camera will send the following:
    //  00 00 00 01
    //  41 - P frame (NOTE: 61, 21, can also be P (well non-I) frames in general)
    //  67 - start of IFrame - well really data about the frame size (SPS)
    //  68 - Next part of I Frame - keep this in the same file if you are splitting up frames (PPS)
    //  06 - PCLO data - this needs to be removed
    //  65 - the actual I frame data - keep in the file 
    //
    //  So 00 00 00 01 06 ......  00 00 00 01 41  <- P data 
    //     00 00 00 01 67 ... 00 00 00 01 68 .. 00 00 00 01 06 .. 00 00 00 01 65 <- I frame
    //
    //  Need to check for two error conditions
    //     1.  Missing Packets
    //     2.  Packets out of order
    // Actually - Live555 will do this for you, but need to check for the problems
    //
    unsigned char firstByte = sink->fReceiveBuffer[0];
    RTPSource * rtpSource = (RTPSource *)sink->fSource;
    u_int16_t seqNumber = rtpSource->curPacketRTPSeqNum();
    seqNumber = sink->currentSeqNo();
    int cubeFaceIndex = sink->fCubeFaceIndex;
    int width = 0;
    int height = 0;

    if (_decodingCanStart[cubeFaceIndex] == false) {
        if (firstByte == 0x67) {
            _decodingCanStart[cubeFaceIndex] = true;
        }
    }
    /*  Was allocating here - use the decoder cause there's lots of dependencies - 
           have the decoder figure this out for me instead
    if (_decodingCanStart[cubeFaceIndex] == false) {
        if (firstByte == 0x67) {
            // Need to allocate the buffers for decoding here
            // Will depend on 'level_id'
            //  Hey - it's even byte aligned
            int levelId = (sink->fReceiveBuffer[3]);
            if (levelId == 42) {
                width = (sink->fReceiveBuffer[6] & 0x01) << 10;
                width += (sink->fReceiveBuffer[7] & 0xff) << 2;
                width += (sink->fReceiveBuffer[8] & 0xC0) >> 6;
                width *= 16;

                height = (sink->fReceiveBuffer[8] & 0x3f) << 9;
                height += (sink->fReceiveBuffer[9] & 0xff) << 1;
                height += (sink->fReceiveBuffer[10] & 0x80) >> 7;
                height *= 16;
            }
            else if (levelId == 50) {
                width = (sink->fReceiveBuffer[6] & 0x01) << 14;
                width += (sink->fReceiveBuffer[7] & 0xff) << 6;
                width += (sink->fReceiveBuffer[8] & 0xfc) >> 2;
                width *= 16;

                height = (sink->fReceiveBuffer[8] & 0x03) << 11;
                height += (sink->fReceiveBuffer[9] & 0xff) << 3;
                height += (sink->fReceiveBuffer[10] & 0xe0) >> 5;
                height *= 16;
            }
            else {
                throw runtime_error("Need to handle level_id case " + levelId);
            }


            for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
                // Y
                _timeBuffer[cubeFaceIndex].pictureInfo[i][0].width = width;
                int stride = _timeBuffer[cubeFaceIndex].pictureInfo[i][0].stride = ((width % 16) == 0) ? width : ((width / 16) + 1 * 16);
                _timeBuffer[cubeFaceIndex].pictureInfo[i][0].height = height;
                // _timeBuffer[cubeFaceIndex].pictureInfo[i][0].data = (unsigned char *)HeapAlloc(GetProcessHeap(), HEAP_NO_SERIALIZE, stride * height);
                _timeBuffer[cubeFaceIndex].pictureInfo[i][0].data = (unsigned char *)malloc(stride * height);
                if (_timeBuffer[cubeFaceIndex].pictureInfo[i][0].data == NULL) {
                    throw runtime_error("OUT OF MEMORY - Cannot allocate output frame buffer.");
                }
                // U
                _timeBuffer[cubeFaceIndex].pictureInfo[i][1].width = width / 2;
                stride = _timeBuffer[cubeFaceIndex].pictureInfo[i][1].stride = ((width % 16) == 0) ? width / 2 : ((width / 16) + 1 * 16) / 2;
                _timeBuffer[cubeFaceIndex].pictureInfo[i][1].height = height / 2;
                // _timeBuffer[cubeFaceIndex].pictureInfo[i][1].data = (unsigned char *)HeapAlloc(GetProcessHeap(), HEAP_NO_SERIALIZE, stride * height / 2);
                _timeBuffer[cubeFaceIndex].pictureInfo[i][1].data = (unsigned char *)malloc(stride * height / 2);
                if (_timeBuffer[cubeFaceIndex].pictureInfo[i][1].data == NULL) {
                    throw runtime_error("OUT OF MEMORY - Cannot allocate output frame buffer.");
                }
                // V
                _timeBuffer[cubeFaceIndex].pictureInfo[i][2].width = width / 2;
                stride = _timeBuffer[cubeFaceIndex].pictureInfo[i][2].stride = ((width % 16) == 0) ? width / 2 : ((width / 16) + 1 * 16) / 2;
                _timeBuffer[cubeFaceIndex].pictureInfo[i][2].height = height / 2;
                // _timeBuffer[cubeFaceIndex].pictureInfo[i][2].data = (unsigned char *)HeapAlloc(GetProcessHeap(), HEAP_NO_SERIALIZE, stride * height / 2);
                _timeBuffer[cubeFaceIndex].pictureInfo[i][2].data = (unsigned char *)malloc(stride * height / 2);
                if (_timeBuffer[cubeFaceIndex].pictureInfo[i][2].data == NULL) {
                    throw runtime_error("OUT OF MEMORY - Cannot allocate output frame buffer.");
                }
            }
            _threadInfo[cubeFaceIndex].streamSetCallbackPtr(cubeFaceIndex, width, height, _threadInfo[cubeFaceIndex].layoutInfo);

            _decodingCanStart[cubeFaceIndex] = true;
        }
    }
    */
    // To save typeing
    TIME_BUFFER * pTimeBuffer = &_timeBuffer[cubeFaceIndex];
    int timeBufferIndex = pTimeBuffer->currentTimeBufferBeingUsed;
    int bufferIndex = pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex];
    unsigned char * pBuffer = pTimeBuffer->decoderInputBuffer[timeBufferIndex];

    if (_decodingCanStart[cubeFaceIndex] == true)
    {
        if (firstByte == 6) {

#ifdef IFRAME_ONLY
            // only write i-frame right now
            if ((pTimeBuffer->currentTimeBufferIsIFrame == true) && (pTimeBuffer->currentTimeBufferIsIFrame == true) && (pTimeBuffer->currentTimeBufferTimeStored == false))
            {
                pTimeBuffer->currentTimeBufferTimeStored = true;
                double time = FindPelcoMetaData(sink->fReceiveBuffer, frameSize);
                _timeBuffer[sink->fCubeFaceIndex].time[timeBufferIndex] = time;
                DBOUT("Wrote 0x6 value. Cubeface:  " << cubeFaceIndex << "\n");
            }
#else
            // UGH
            //   For i-frames - this will happen after the 0x67
            //   For other frames, it happens BEFORE the 0x41
            // So need to check the type of frame I think I am . . .
            double time = FindPelcoMetaData(sink->fReceiveBuffer, frameSize, _metaDataString,
                _timeBuffer[sink->fCubeFaceIndex].analyticObjects[timeBufferIndex]);
            // This can have problems if we drop the wrong frame . . ..
            if (pTimeBuffer->currentTimeBufferTimeStored == false) {
                _timeBuffer[sink->fCubeFaceIndex].time[timeBufferIndex] = time;
                pTimeBuffer->currentTimeBufferTimeStored = true;
            }
            else {
                // Assuming that this must be a NON-iframe - risky business here . . ..
                FindAndLockNextTimeBuffer(sink->fCubeFaceIndex);
                timeBufferIndex = pTimeBuffer->currentTimeBufferBeingUsed;
                _timeBuffer[sink->fCubeFaceIndex].time[timeBufferIndex] = time;
                pTimeBuffer->currentTimeBufferTimeStored = true;
            }
            //DBOUT("Wrote 0x6 value. Cubeface:  " << cubeFaceIndex << "\n");
#endif
        }
        // Non - I Frame data - Start AND end of it - means if fit into one packet
        else if (firstByte == 0x41) {
#ifndef IFRAME_ONLY
            pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] = FillBuffer(&pBuffer[bufferIndex], sink->fReceiveBuffer, frameSize, bufferIndex, firstByte);

            PostThreadMessage(pTimeBuffer->deocderThreadID, 0, kDecoderMessageNotIFrame, 0);
#endif
        }
        // Non -I frame data - start end or continuation of P frame
        else if (firstByte == 0x5C) {
            // This will not happen - live555 will accumulate the buffer for you
            pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] = FillBuffer(&pBuffer[bufferIndex], sink->fReceiveBuffer, frameSize, bufferIndex, 0);
        }
        //  SPS (sequence parameter set)
        else if (firstByte == 0x67) {
            // When you get this, you want to increment the current timebuffer pointer
            FindAndLockNextTimeBuffer(sink->fCubeFaceIndex);
            timeBufferIndex = pTimeBuffer->currentTimeBufferBeingUsed;
            bufferIndex = 0;
            pTimeBuffer->currentTimeBufferIsIFrame = true;
            pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] = FillBuffer(&pBuffer[bufferIndex], sink->fReceiveBuffer, frameSize, bufferIndex, firstByte);
            pTimeBuffer->isIFrame[timeBufferIndex] = true;
            //DBOUT("Wrote 0x67 value. Cubeface:  " << cubeFaceIndex << "\n");
        }
        //  PPS (Picture Parameter Set)
        else if (firstByte == 0x68) {
            pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] = FillBuffer(&pBuffer[bufferIndex], sink->fReceiveBuffer, frameSize, bufferIndex, firstByte);
            //DBOUT("Wrote 0x68 value. Cubeface:  " << cubeFaceIndex << "\n");
        }
        // I Frame data - Start AND end of it (guess - never have seen one fit in one packet) - though Live555 will make it fit in a packet
        else if (firstByte == 0x65) {
            pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] = FillBuffer(&pBuffer[bufferIndex], sink->fReceiveBuffer, frameSize, bufferIndex, firstByte);
            // Put this data now through the decoder
            PostThreadMessage(pTimeBuffer->deocderThreadID, 0, kDecoderMessageIFrame, 0);

            //DBOUT("Wrote 0x65 value. Cubeface:  " << cubeFaceIndex << "\n");
        }
        // Start, End, or continuation of bit stream for I 
        else if (firstByte == 0x7C) {
            // This will never happen because Live555 will accumulate this for you
            pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] = FillBuffer(&pBuffer[bufferIndex], sink->fReceiveBuffer, frameSize, bufferIndex, 0);
        }
        else {
            // throw runtime_error("Have RTP Byte that does not make sense:  " + firstByte);
            DBOUT("Received RTP byte not parsed:  " << firstByte << "\n");
        }
    }
    // Then continue, to request the next frame of data:
    sink->continuePlaying();
}


Boolean OpenH264FrameSource::DummySink::continuePlaying() {
    if (fSource == NULL) return False; // sanity check (should not happen)

    // Request the next frame of data from our input source.  "afterGettingFrame()" will get called later, when it arrives:
    fSource->getNextFrame(fReceiveBuffer, DUMMY_SINK_RECEIVE_BUFFER_SIZE,
        afterGettingFrame, this,
        onSourceClosure, this);
    return True;
}

///////////////////////////////////////////////////////////////////////////////
//
//  STOP of LIVE555 code 
//
//  START of decoding helper functions
//
//
///////////////////////////////////////////////////////////////////////////////
//pTimeBuffer->indexToInputBuffer[timeBufferIndex] = FillBuffer(&pBuffer[bufferIndex], sink->fReceiveBuffer, frameSize);
int OpenH264FrameSource::FillBuffer(const unsigned char * destintation, const unsigned char * source, int size, int index, unsigned char marker)
{
    if (marker != 0) {
        if ((index + 4 + size) < kMaxSizeOfDecoderInput) {
            unsigned char specialBytes[] = { 0x00, 0x00, 0x00, 0x01 };
            memcpy((void *)destintation, specialBytes, 4);
            index += 4;
            memcpy((void *)(destintation + 4), source, size);
            return index + size;
        }
        else {
            throw runtime_error("Network input buffer not large enough");
        }
    }
    else {
        if ((index + size - 1) < kMaxSizeOfDecoderInput) {
            // Do not want to write the first byte in this case
            memcpy((void *)destintation, source + 1, size - 1);
            return index + size - 1;
        }
        else {
            throw runtime_error("Network input buffer not large enough");
        }
    }
}

void OpenH264FrameSource::FindAndLockNextTimeBuffer(int cubeFaceIndex) {
    // TODO - WFB _ change this to a per cubeface mutex
    _timeBufferCheckMutex.lock();
    {
        // Make sure to mark the buffer as not being used any longer
        _timeBuffer[cubeFaceIndex].usedByNetworkInput[_timeBuffer[cubeFaceIndex].currentTimeBufferBeingUsed] = false;
        int usingThisFrameIndex = 0;
        // Start with the lowest index that is not being used for copy or decode(should only be one at most)
        for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
            if ( (_timeBuffer[cubeFaceIndex].usedByCopyThread[i] == false)  &&
                (_timeBuffer[cubeFaceIndex].usedByDecode[i] == false) )
            {
                usingThisFrameIndex = i;
                break;
            }
        }

        // Find lowest time buffer not locked (or zero time)
        for (int i = usingThisFrameIndex; i < kNumberTimeFrameBuffers; i++) {
            if ((_timeBuffer[cubeFaceIndex].time[usingThisFrameIndex] > _timeBuffer[cubeFaceIndex].time[i]) &&
                (_timeBuffer[cubeFaceIndex].usedByCopyThread[i] == false) &&
                (_timeBuffer[cubeFaceIndex].usedByDecode[i] == false) ) {
                usingThisFrameIndex = i;
            }
        }
        _timeBuffer[cubeFaceIndex].currentTimeBufferBeingUsed = usingThisFrameIndex;
        _timeBuffer[cubeFaceIndex].currentTimeBufferIsIFrame = false;
        _timeBuffer[cubeFaceIndex].currentTimeBufferTimeStored = false;
        _timeBuffer[cubeFaceIndex].indexToNetworkInputBuffer[usingThisFrameIndex] = 0;
        _timeBuffer[cubeFaceIndex].usedByNetworkInput[usingThisFrameIndex] = true;
        _timeBuffer[cubeFaceIndex].requiresDecoding[usingThisFrameIndex] = true;
        _timeBuffer[cubeFaceIndex].isDecoded[usingThisFrameIndex] = false;
        _timeBuffer[cubeFaceIndex].isIFrame[usingThisFrameIndex] = false;
    }
    _timeBufferCheckMutex.unlock();
}

// true - good
// false - bad
bool OpenH264FrameSource::DecodeNextImage(int cubeFaceIndex, TIME_BUFFER * pTimeBuffer, unsigned char * pBuffer, int timeBufferIndex)
{
    /*
    if (pTimeBuffer->isIFrame[timeBufferIndex] == false) {
        DBOUT("Not I-frame    size:  " << pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] << "\n");
        return false;
    }
    else {
        DBOUT("IS IFRAME     size   " << pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex] << "\n");
    }
    */

    //  These will be in milliseconds
    DWORD tickStart, tickEnd, tickDuration;

    tickStart = GetTickCount();
    
    unsigned char * pData[3] = { NULL };
    //pData[0] = pTimeBuffer->pictureInfo[timeBufferIndex][0].data;
    //pData[1] = pTimeBuffer->pictureInfo[timeBufferIndex][1].data;
    //pData[2] = pTimeBuffer->pictureInfo[timeBufferIndex][2].data;

    // This thing uses its own buffers???
    //  no idea when they are freed
    SBufferInfo sDstBufInfo;
    memset(&sDstBufInfo, 0, sizeof(SBufferInfo));
    unsigned long long uiTimeStamp = 0;
    sDstBufInfo.uiInBsTimeStamp = (unsigned long long) pTimeBuffer->time[timeBufferIndex];

    pTimeBuffer->pDecoder->DecodeFrameNoDelay(pBuffer, pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex], pData, &sDstBufInfo);
    //pTimeBuffer->pDecoder->DecodeFrame2(pBuffer, pTimeBuffer->indexToNetworkInputBuffer[timeBufferIndex], pData, &sDstBufInfo);
    /*
    memset(&sDstBufInfo, 0, sizeof(SBufferInfo));
    pTimeBuffer->pDecoder->DecodeFrame2(NULL, 0, pData, &sDstBufInfo);
    */
    if (sDstBufInfo.iBufferStatus == 1) {
        //  These will be in milliseconds
        DWORD tickStart, tickEnd, tickDuration;
        tickStart = GetTickCount();
        // This is good I think
        //    May need to allocate the buffers if they never have been before . . .
        if (pTimeBuffer->pictureInfo[timeBufferIndex][0].data == NULL) {
            AllocateDecodingImageBuffers(cubeFaceIndex, sDstBufInfo.UsrData.sSystemBuffer.iWidth, sDstBufInfo.UsrData.sSystemBuffer.iHeight);
        }
        for (int i = 0; i < sDstBufInfo.UsrData.sSystemBuffer.iHeight; i++) {
            memcpy(pTimeBuffer->pictureInfo[timeBufferIndex][0].data + (i * sDstBufInfo.UsrData.sSystemBuffer.iWidth),
                pData[0] + i * sDstBufInfo.UsrData.sSystemBuffer.iStride[0],
                sDstBufInfo.UsrData.sSystemBuffer.iWidth);
        }
        for (int i = 0; i < sDstBufInfo.UsrData.sSystemBuffer.iHeight / 2; i++) {
            memcpy(pTimeBuffer->pictureInfo[timeBufferIndex][1].data + (i * sDstBufInfo.UsrData.sSystemBuffer.iWidth / 2),
                pData[1] + i * sDstBufInfo.UsrData.sSystemBuffer.iStride[1],
                sDstBufInfo.UsrData.sSystemBuffer.iWidth / 2);
            memcpy(pTimeBuffer->pictureInfo[timeBufferIndex][2].data + (i * sDstBufInfo.UsrData.sSystemBuffer.iWidth / 2),
                pData[2] + i * sDstBufInfo.UsrData.sSystemBuffer.iStride[1],
                sDstBufInfo.UsrData.sSystemBuffer.iWidth / 2);
        }
        tickEnd = GetTickCount();
        pTimeBuffer->timeToCopy[timeBufferIndex] = tickDuration = tickEnd - tickStart;
    }
    else {
        DBOUT("Bad Decode \n");
    }
    tickEnd = GetTickCount();
    pTimeBuffer->timeToDecode[timeBufferIndex] = tickDuration = tickEnd - tickStart;
    DBOUT("Time to Decode " << tickDuration << "ms \n");
    return (sDstBufInfo.iBufferStatus == 1) ? true : false;
}
void OpenH264FrameSource::AllocateDecodingImageBuffers(int cubeFaceIndex, int width, int height)
{
    for (int i = 0; i < kNumberTimeFrameBuffers; i++) {
        // Y
        _timeBuffer[cubeFaceIndex].pictureInfo[i][0].width = width;
        int stride = _timeBuffer[cubeFaceIndex].pictureInfo[i][0].stride = ((width % 16) == 0) ? width : ((width / 16) + 1 * 16);
        _timeBuffer[cubeFaceIndex].pictureInfo[i][0].height = height;
        _timeBuffer[cubeFaceIndex].pictureInfo[i][0].data = (unsigned char *)HeapAlloc(GetProcessHeap(), HEAP_NO_SERIALIZE, stride * height);
        //_timeBuffer[cubeFaceIndex].pictureInfo[i][0].data = (unsigned char *)malloc(stride * height);
        if (_timeBuffer[cubeFaceIndex].pictureInfo[i][0].data == NULL) {
            throw runtime_error("OUT OF MEMORY - Cannot allocate output frame buffer.");
        }
        // U
        _timeBuffer[cubeFaceIndex].pictureInfo[i][1].width = width / 2;
        stride = _timeBuffer[cubeFaceIndex].pictureInfo[i][1].stride = ((width % 16) == 0) ? width / 2 : ((width / 16) + 1 * 16) / 2;
        _timeBuffer[cubeFaceIndex].pictureInfo[i][1].height = height / 2;
        _timeBuffer[cubeFaceIndex].pictureInfo[i][1].data = (unsigned char *)HeapAlloc(GetProcessHeap(), HEAP_NO_SERIALIZE, stride * height / 2);
        //_timeBuffer[cubeFaceIndex].pictureInfo[i][1].data = (unsigned char *)malloc(stride * height / 2);
        if (_timeBuffer[cubeFaceIndex].pictureInfo[i][1].data == NULL) {
            throw runtime_error("OUT OF MEMORY - Cannot allocate output frame buffer.");
        }
        // V
        _timeBuffer[cubeFaceIndex].pictureInfo[i][2].width = width / 2;
        stride = _timeBuffer[cubeFaceIndex].pictureInfo[i][2].stride = ((width % 16) == 0) ? width / 2 : ((width / 16) + 1 * 16) / 2;
        _timeBuffer[cubeFaceIndex].pictureInfo[i][2].height = height / 2;
        _timeBuffer[cubeFaceIndex].pictureInfo[i][2].data = (unsigned char *)HeapAlloc(GetProcessHeap(), HEAP_NO_SERIALIZE, stride * height / 2);
        //_timeBuffer[cubeFaceIndex].pictureInfo[i][2].data = (unsigned char *)malloc(stride * height / 2);
        if (_timeBuffer[cubeFaceIndex].pictureInfo[i][2].data == NULL) {
            throw runtime_error("OUT OF MEMORY - Cannot allocate output frame buffer.");
        }
    }
    _threadInfo[cubeFaceIndex].streamSetCallbackPtr(cubeFaceIndex, width, height, _threadInfo[cubeFaceIndex].layoutInfo, (cubeFaceIndex == 6) ? true : false);
    if ((_threadInfo[cubeFaceIndex].numberCubeFaces == 1) && (_threadInfo[cubeFaceIndex].useSitAwareness)) {
        // mosaic to panoramic conversion
        _situationTracking->setLayoutInfo(_threadInfo[cubeFaceIndex].layoutInfo, width, height);
    }
}

void OpenH264FrameSource::DrawAnalyticObject(PLANE_INFO& pictureInfo, const vector<ObjectRect>& objects)
{
    for (vector<ObjectRect>::const_iterator it = objects.begin(); it != objects.end(); ++it) {
        drawBox(imageCoordinate(it->left, pictureInfo.width),
            imageCoordinate(it->top, pictureInfo.height),
            imageCoordinate(it->right, pictureInfo.width),
            imageCoordinate(it->bottom, pictureInfo.height), pictureInfo.data, 255,
            pictureInfo.width, pictureInfo.height, pictureInfo.stride);
    }
}
