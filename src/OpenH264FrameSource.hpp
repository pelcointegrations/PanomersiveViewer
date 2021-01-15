//============================================================================
// Copyright (c) 2015 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_OPENH264_FRAME_SOURCE_HPP__
#define __PELCO_IMEXTK_OPENH264_FRAME_SOURCE_HPP__

#include "BaseFrameSource.hpp"
#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <condition_variable>
#include "StreamLayoutMetadata.hpp"
#include "MosaicToPanTilt.hpp"

// Live555
#include "liveMedia.hh"
#include "BasicUsageEnvironment.hh"

// OpenH264 decoder
#include <codec_def.h>
#include <codec_app_def.h>
#include <codec_api.h>

class OpenH264FrameSource : public BaseFrameSource {
public:
	OpenH264FrameSource(const OpenH264FrameSource&) = delete;
	OpenH264FrameSource& operator=(const OpenH264FrameSource&) = delete;


	// attaches to camera, and gets relevant stream info.
    OpenH264FrameSource(const std::string& cameraIP,
        const StreamSetupCallback& setup,
        const GetBufferCallback& getBuffer,
        const ReleaseBufferCallback& releaseBuffer,
        bool highRes,
        bool connectAsNonOpteraType,
        bool uniStream,
        bool primaryStream,
        const std::vector<std::string>& discoveryURLs,
        bool multicast,
        bool useSitAwareness,
        const std::string& ptzIp,
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
        bool thisIsSitAware,
        std::shared_ptr<SituationTracking> tracking,
        int encoderChannelSelected);

	virtual ~OpenH264FrameSource();

	// start/stop the RTP stream.  Frames will be delivered to ImageUpdateCallback as they
	// are received.
	// Note if there are outstanding FrameBuffers when stop() is called,
	// some tearing may restult.  The best approach is to stop flush all outstanding
	// frames before calling stop().
	void start();
	void stop();

    bool hasUserMetadataText();
    std::string userMetadataText();

private:
	const std::string      _cameraIP;
	const bool             _highRes;
	std::vector<MediaDescriptorsInfo> _mediaDescriptorsInfo;
    const std::vector<std::string>    _discoveryURLs;
    bool                   _useUnistream;
    bool                   _useSitAwareness;
    const std::string      _ptzIp;
	bool                   _separatedCameras;
	const float            _opteraHeight;
	const float            _ptzHeight;
	const float            _distanceToOptera;
	const float            _minMoveDistance;
	const float            _ptzTiltX;
	const float            _ptzTiltZ;
	const float            _ptzConeError;
	const float            _opteraTiltX;
	const float            _opteraTiltZ;
	const float            _opteraConeError;
    bool                   _thisIsSitAwareCam;

    StreamSetupCallback   _setupCallback;
	GetBufferCallback     _getBufferCallback;
	ReleaseBufferCallback _releaseBufferCallback;

	typedef struct {
		GetBufferCallback                getBufferCallbackPtr;
		ReleaseBufferCallback            releaseBufferCallbackPtr;
		StreamSetupCallback              streamSetCallbackPtr;
		size_t                           numberCubeFaces;
		std::string                      rtspURL;
		std::string                      layoutInfo;
		int                              cubeFaceIndex;
        DWORD                            startingThreadID;
        bool                             useSitAwareness;
        bool                             thisIsSitAwareCam;
	} STREAM_THREAD_INFO;
	static const int kNumberTimeFrameBuffers = 5;
	static STREAM_THREAD_INFO _threadInfo[kMaxDragonFlyVStreams + 1];

	static bool _stopNow;
	static bool _networkInputThreadStopped[kMaxDragonFlyVStreams + 1];
    static bool _decoderThreadStopped[kMaxDragonFlyVStreams + 1];
    static bool _copyThreadStopped;
    
    void cleanup(void);

    typedef struct {
        unsigned char * data;
        int width;
        int height;
        int stride;
    } PLANE_INFO;
	typedef struct {
        bool usedByNetworkInput[kNumberTimeFrameBuffers];
		bool usedByDecode[kNumberTimeFrameBuffers];
		bool usedByCopyThread[kNumberTimeFrameBuffers];
		double time[kNumberTimeFrameBuffers];
        // 3 picture Info Buffers - Y U V
        // This is the output of the decoding process
        PLANE_INFO pictureInfo[kNumberTimeFrameBuffers][3];
        // This is the input to the decoder for this particular 
        //   instance
        unsigned char * decoderInputBuffer[kNumberTimeFrameBuffers];
        // This will be an index to the buffer decoderInputBuffer
        int indexToNetworkInputBuffer[kNumberTimeFrameBuffers];
        ISVCDecoder* pDecoder;
        SDecodingParam sDecParam;
        bool requiresDecoding[kNumberTimeFrameBuffers];
        bool isDecoded[kNumberTimeFrameBuffers];
        bool isIFrame[kNumberTimeFrameBuffers];
        DWORD timeToDecode[kNumberTimeFrameBuffers];
        DWORD timeToCopy[kNumberTimeFrameBuffers];
        int  currentTimeBufferBeingUsed;
        bool currentTimeBufferIsIFrame;
        bool currentTimeBufferTimeStored;
        DWORD deocderThreadID;
        std::vector<ObjectRect> analyticObjects[kNumberTimeFrameBuffers];
	} TIME_BUFFER;
    static std::mutex _timeBufferCheckMutex;
    //  For time buffers - there will be an array of cubeFaces X kNumberTimeFrameBuffers
	static TIME_BUFFER _timeBuffer[kMaxDragonFlyVStreams + 1];
    static std::string _metaDataString;
    // This is because the data to the buffers come from live 555 piecemeal
    // This is needed to start the decoder on the first I-Frame received
    static bool _decodingCanStart[kMaxDragonFlyVStreams + 1];
    static const int kMaxSizeOfDecoderInput = 3 * 1024 * 1024;

	// One thread regardless of cube faces
    static DWORD WINAPI copyThread(LPVOID lpParam);
	static std::mutex _copyThreadLock;
	static std::condition_variable _wakeUpCopyThread;

    // Will be one of these threads per cubeface - this is for Live555
    static DWORD WINAPI networkInputThread(LPVOID lpParam);

    // Will be one thread for cubeface
    static DWORD WINAPI decoderThread(LPVOID lpParam);
    typedef enum {
        kDecoderMessageQuit,
        kDecoderMessageIFrame,
        kDecoderMessageNotIFrame
    } DECODER_MESSAGES;
    

    // Helper Functions
    static void FindAndLockNextTimeBuffer(int cubeFaceIndex);
    static int FillBuffer(const unsigned char * dest, const unsigned char * source, int size, int index, unsigned char marker);
    static double FindPelcoMetaData(unsigned char * readBuffer, int fileSize, std::string & textMetaData,
        std::vector<ObjectRect>& analyticObjects);
    static void AllocateDecodingImageBuffers(int cubeFaceIndex, int w, int h);
    static bool DecodeNextImage(int cubeFaceIndex, TIME_BUFFER * pTimeBuffer, unsigned char * pBuffer, int timeBufferIndex);
    static void DrawAnalyticObject(PLANE_INFO& pictureInfo, const std::vector<ObjectRect>& objects);


    ///////////////////////////////////////////////////////////////////////
    //
    // LIVE555
    //
    ///////////////////////////////////////////////////////////////////////
    // RTSP 'response handlers':
    static void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString);
    static void continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString);
    static void continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString);

    // Other event handler functions:
    static void subsessionAfterPlaying(void* clientData); // called when a stream's subsession (e.g., audio or video substream) ends
    static void subsessionByeHandler(void* clientData); // called when a RTCP "BYE" is received for a subsession
    static void streamTimerHandler(void* clientData);
    // called at the end of a stream's expected duration (if the stream has not already signaled its end using a RTCP "BYE")

    // Used to iterate through each stream's 'subsessions', setting up each one:
    static void setupNextSubsession(RTSPClient* rtspClient);

    // Used to shut down and close a stream (including its "RTSPClient" object):
    static void shutdownStream(RTSPClient* rtspClient, int exitCode = 1);

    // Define a class to hold per-stream state that we maintain throughout each stream's lifetime:
    class StreamClientState {
    public:
        StreamClientState();
        virtual ~StreamClientState();

    public:
        MediaSubsessionIterator* iter;
        MediaSession* session;
        MediaSubsession* subsession;
        TaskToken streamTimerTask;
        double duration;
    };
    // If you're streaming just a single stream (i.e., just from a single URL, once), then you can define and use just a single
    // "StreamClientState" structure, as a global variable in your application.  However, because - in this demo application - we're
    // showing how to play multiple streams, concurrently, we can't do that.  Instead, we have to have a separate "StreamClientState"
    // structure for each "RTSPClient".  To do this, we subclass "RTSPClient", and add a "StreamClientState" field to the subclass:
    class ourRTSPClient : public RTSPClient {
    public:
        static ourRTSPClient* createNew(UsageEnvironment& env, char const* rtspURL,
            int verbosityLevel = 0,
            char const* applicationName = NULL,
            portNumBits tunnelOverHTTPPortNum = 0);

    protected:
        ourRTSPClient(UsageEnvironment& env, char const* rtspURL,
            int verbosityLevel, char const* applicationName, portNumBits tunnelOverHTTPPortNum);
        // called only by createNew();
        virtual ~ourRTSPClient();

    public:
        StreamClientState scs;
        int cubeFaceIndex;
    };
    static ourRTSPClient * _rtspClients[kMaxDragonFlyVStreams + 1];

    // The main streaming routine (for each "rtsp://" URL):
    static ourRTSPClient * openURL(UsageEnvironment& env, char const* progName, char const* rtspURL, int cubeFaceIndex);

    // Define a data sink (a subclass of "MediaSink") to receive the data for each subsession (i.e., each audio or video 'substream').
    // In practice, this might be a class (or a chain of classes) that decodes and then renders the incoming audio or video.
    // Or it might be a "FileSink", for outputting the received data into a file (as is done by the "openRTSP" application).
    // In this example code, however, we define a simple 'dummy' sink that receives incoming data, but does nothing with it.

    // class DummySink : public MediaSink {
    class DummySink: public RTPSink {
    public:
        static DummySink* createNew(UsageEnvironment& env,
            MediaSubsession& subsession, // identifies the kind of data that's being received
            int cubeFaceIndex,            // CubeFace Index
            char const* streamId = NULL); // identifies the stream itself (optional)
    private:
        DummySink(UsageEnvironment& env, MediaSubsession& subsession, int cubeFaceIndex, char const* streamId);
        // called only by "createNew()"
        virtual ~DummySink();

        static void afterGettingFrame(void* clientData, unsigned frameSize,
            unsigned numTruncatedBytes,
            struct timeval presentationTime,
            unsigned durationInMicroseconds);

    private:
        // redefined virtual functions:
        virtual Boolean continuePlaying();

    private:
        u_int8_t* fReceiveBuffer;
        MediaSubsession& fSubsession;
        char* fStreamId;
        int fCubeFaceIndex;
    };

    static const int RTSP_CLIENT_VERBOSITY_LEVEL = 1; // by default, print verbose output from each "RTSPClient"
    static unsigned rtspClientCount;              // Counts how many streams (i.e., "RTSPClient"s) are currently in use.
    static std::shared_ptr<SituationTracking> _situationTracking;
};

#endif  //#ifndef __PELCO_IMEXTK_OPENH264_FRAME_SOURCE_HPP__
