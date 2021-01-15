//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "RtpFrameSource.hpp"
#include "StreamLayoutMetadata.hpp"
#include <sstream>
#include <string.h>
#include "MosaicToPanTilt.hpp"
#include <cassert>

using namespace std;


//////////////////////////////////////////////////////////////////////////////////
RtpFrameSource::RtpFrameSource(const string& cameraIP,
        const StreamSetupCallback& setup, const GetBufferCallback& getBuffer,
        const ReleaseBufferCallback& releaseBuffer,
        bool highRes, bool connectAsNonOpteraType, bool uniStream, bool primaryStream, const vector<string>& discoveryURLs, bool multicast, bool thisIsSitAwareness, 
        int encoderChannelSelected)
    : _cameraIP(cameraIP),
      _vlcInstance(NULL),
      _setupCallback(setup),
      _getBufferCallback(getBuffer),
      _releaseBufferCallback(releaseBuffer),
      _streams(0),
      _frameCount(0),
      _averageBitRate(0.0),
      _rollingBitRate(0.0),
      _discoveryURLs(discoveryURLs),
      _stoppingAllStreamsNow(false)
{
    const char* const vlc_args[] = {"--aout=dummy","--vout=dummy","--clock-synchro=0"};
    _vlcInstance = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);
    if (_vlcInstance == NULL) {
        throw runtime_error("Error starting libvlc");
    }

    // TODO: URL discovery protocol -- for now, just assuming it's a 180-model,
    // with front and right stream names.
    try {
        std::vector<MediaDescriptorsInfo> mediaDescriptorsInfo;
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
            } else if (primaryStream == false) {
                videoURLSubStr = "/stream2";
                if ((encoderChannelSelected > 0) && (encoderChannelSelected < 17)) {
                    videoURLSubStr = "/channel";
                    videoURLSubStr += to_string(encoderChannelSelected);
                    videoURLSubStr += "/stream2";
                }
            }
            string videoLayoutMetadata = "v1.0, 1, 4, 0, 0, 1, 1, 0, 0, 1, 1";
            mediaDescriptorsInfo.push_back({ videoURLSubStr, videoLayoutMetadata });
            count = 1;
            _streams.resize(mediaDescriptorsInfo.size(), NULL);
        }
        else {
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
            GetStreamLayoutMetadatas(url, mediaDescriptorsInfo);
            count = mediaDescriptorsInfo.size();
            _streams.resize(mediaDescriptorsInfo.size(), NULL);
            if (count == 0) {
                throw runtime_error("No video count in DESCRIBE call");
            }
            else if ((count > kMaxDragonFlyVStreams) || ((count < 2) && highRes && (uniStream == false))) {
                throw runtime_error("Unexpected stream count from DESCRIBE call");
            }
        }

        for (size_t i = 0; i < count; i++) {
            // we have a 180 dragonfly/optera camera
            if (connectAsNonOpteraType == false) {
                if ((uniStream == true) && (highRes == true)) {
                    mediaDescriptorsInfo[i].rtspSubURL = "/stream1";
                }
                if (multicast == true) {
                    if (count == 1) {
                        mediaDescriptorsInfo[i].rtspSubURL.append("m");
                    }
                    else {
                        std::size_t pos = mediaDescriptorsInfo[i].rtspSubURL.rfind("_v");
                        mediaDescriptorsInfo[i].rtspSubURL.replace(pos, 2, "m_v");
                    }
                }
            }
            // This has to be unique right now since it is an index to the PanomersiveViewer
            if (thisIsSitAwareness == true) {
                _streams[i] = new Stream(this, 6, string("rtsp://") + _cameraIP + mediaDescriptorsInfo[i].rtspSubURL,
                    _vlcInstance, mediaDescriptorsInfo[i].layoutMetadata, true);
            }
            else {
                _streams[i] = new Stream(this, i, string("rtsp://") + _cameraIP + mediaDescriptorsInfo[i].rtspSubURL,
                    _vlcInstance, mediaDescriptorsInfo[i].layoutMetadata, false);
            }
        }
    }
    catch (...) {
        cleanup();
        throw;
    }
}

//////////////////////////////////////////////////////////////////////////////////
RtpFrameSource::~RtpFrameSource() {
    cleanup();
}

//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::cleanup() {
    for (size_t i = 0; i < _streams.size(); ++i) {
        delete _streams[i];
        _streams[i] = NULL;
    }
    _streams.clear();

    libvlc_release(_vlcInstance);
}

//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::start()
{
    for (size_t i = 0; i < _streams.size(); ++i) {
        _streams[i]->start();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::stop()
{
    _stoppingAllStreamsNow = true;
    for (size_t i = 0; i < _streams.size(); ++i) {
        _streams[i]->stopMe();
    }
}

//////////////////////////////////////////////////////////////////////////////////
RtpFrameSource::Stream::Stream(RtpFrameSource* pFrameSource,
        size_t streamIndex, const string& url, libvlc_instance_t* pVlcInstance, const std::string& layoutMetadata, bool thisIsSitAwareness)
    : _pFrameSource(pFrameSource),
      _streamIndex(streamIndex),
      _url(url),
      _pVlcInstance(pVlcInstance),
      _media(NULL),
      _mediaPlayer(NULL),
      _width(0),
      _height(0),
      _thisIsSitAwareness(thisIsSitAwareness)
{
    try {
        //
        // fetch stream layout info from the RTSP server (note, we're bypassing
        // libvlc at the moment.  A more correct solution would be to
        // either access/modify the rtsp plugin, or extract this info from the
        // h264 stream (using the imextk function,
        // pelco_imextk_extract_metadata_from_H264).
        // Both of those options would require writing a custom VLC plugin, so
        // that's currently left as an excersise for the VMS developer.
        //
        _layoutMetadata = layoutMetadata;
        if (_layoutMetadata.empty()) {
            //throw runtime_error("Couldn't get layout metadata");
        }

        _mediaPlayer = libvlc_media_player_new(_pVlcInstance);
        if (_mediaPlayer == NULL) {
            throw runtime_error("Error constructing vlc player");
        }

        //
        // Setup VLC callbacks: vlc calls 'setup' once it knows the stream
        // dimensions.  In that callback, we allocate frames that vlc can decode into.
        // Once playback starts, VLC does the following for each decoded frame:
        //   1. calls our 'lock' callback to get a pointer to a buffer.
        //   2. decodes into that buffer.
        //   3. calls our 'unlock' callback when it's done filling the buffer.
        //
        // C++11 'Lambda' function wrappers are used for these callbacks, just to access
        // the 'this' pointer and call our corresponding class methods.
        auto lockHandler = [] (void* userdata, void** pPixels) {
            RtpFrameSource::Stream* pThis = (RtpFrameSource::Stream*)userdata;
            unsigned char* pY;
            unsigned char* pU;
            unsigned char* pV;
            pThis->lockFrame(pY, pU, pV);
            pPixels[0] = (void*)pY;
            pPixels[1] = (void*)pU;
            pPixels[2] = (void*)pV;
            // (explicit cast needed here to define lambda-expression's return type)
            return (void*)*pPixels;
        };
        auto unlockHandler = [] (void* userdata, void* bufferId,
                void* const* pPixels) {
            RtpFrameSource::Stream* pThis = (RtpFrameSource::Stream*)userdata;
            pThis->unlockFrame(static_cast<unsigned char*>(pPixels[0]),
                    static_cast<unsigned char*>(pPixels[1]),
                    static_cast<unsigned char*>(pPixels[2]));
        };
        libvlc_video_set_callbacks(_mediaPlayer, lockHandler, unlockHandler, NULL,
                this); // userdata that will be passed back into lock/unlock callbacks

        auto setupHandler = [] (void **userdata, char *chroma,
                uint32_t *width, uint32_t *height, uint32_t *pitches, uint32_t *lines) {
            RtpFrameSource::Stream* pThis = (RtpFrameSource::Stream*)(*userdata);
            return pThis->vlcSetup(chroma, width, height, pitches, lines);
        };
        // setupHandler uses the same userdata previously set
        // via libvlc_video_set_callbacks
        libvlc_video_set_format_callbacks(_mediaPlayer, setupHandler, NULL);

        //
        // Finally, give VLC the RTSP url, and prepare to start playback.
        //
        _media = libvlc_media_new_location(_pVlcInstance, _url.c_str());
        if (_media == NULL) {
            throw runtime_error("Error initializing libvlc media");
        }
        libvlc_media_player_set_media(_mediaPlayer, _media);
    } catch (...) {
        cleanup();
        throw;
    }
}

//////////////////////////////////////////////////////////////////////////////////
RtpFrameSource::Stream::~Stream()
{
    cleanup();
}

//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::Stream::cleanup()
{
    if (_mediaPlayer != NULL) {
        libvlc_media_player_release(_mediaPlayer);
        _mediaPlayer = NULL;
    }
    if (_media != NULL) {
        libvlc_media_release(_media);
        _media = NULL;
    }
}


//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::Stream::start()
{
    libvlc_media_player_play(_mediaPlayer);
}

//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::Stream::stopMe()
{
    libvlc_media_player_stop(_mediaPlayer);
}

//////////////////////////////////////////////////////////////////////////////////
uint32_t RtpFrameSource::Stream::vlcSetup(char *chroma,
        uint32_t *width,
        uint32_t *height,
        uint32_t *pitches,
        uint32_t *lines)
{
    memcpy(chroma, "I420", 4);

    _width = *width;
    _height = *height;

    pitches[0] = *width;
    pitches[1] = (*width) / 2;
    pitches[2] = (*width) / 2;
    lines[0] = *height;
    lines[1] = (*height) / 2;
    lines[2] = (*height) / 2;

    size_t numAllocatedFrames =
        _pFrameSource->_setupCallback(_streamIndex, *width, *height, _layoutMetadata, _thisIsSitAwareness);

    libvlc_track_description_t* pDesc = libvlc_video_get_track_description(_mediaPlayer);
    while (pDesc != NULL) {
        const char* pName = pDesc->psz_name;
        pDesc = pDesc->p_next;
    }
    libvlc_track_description_list_release(pDesc);

    int trackIndex = libvlc_video_get_track(_mediaPlayer);
    int i = 0;
    if (trackIndex == 0) {
        i = -1;
    }
    else if (trackIndex == 1) {
        i = 0;
    }
    else if (trackIndex == 2) {
        i = 1;
    }

    return uint32_t(numAllocatedFrames);
}

// #define ENABLE_FRAME_DEBUG
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


const static int AVG_SIZE = 10;
static int readByteIndex = 0;
static int lastReadBytes[AVG_SIZE] = { 0 };
static double lastTime[AVG_SIZE] = { 0.0 };
static int lastLostPicture = 0;
static double firstTimeEver = 0.0;
//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::Stream::lockFrame(unsigned char*& pY,
        unsigned char*& pU, unsigned char*& pV)
{
    _pFrameSource->_getBufferCallback(_streamIndex, pY, pU, pV);

    /*
     *  DOES NOT WORK - ONLY GIVES ZERO :(
     * 
     *
    libvlc_media_track_t ** pMediaTrack = NULL;
    unsigned numberOfTracks = libvlc_media_tracks_get(_media, &pMediaTrack);

    for (unsigned i = 0; i < numberOfTracks; i++) {
        if (pMediaTrack[i]->i_type == libvlc_track_video) {
            unsigned int bitRate = pMediaTrack[1]->i_bitrate;
            DBOUT("Bit Rate:  " << bitRate << "\n");
        }
    }
    libvlc_media_tracks_release(pMediaTrack, numberOfTracks);
    */

    // Only get statics if you are still running.
    //  If you are stopping, this can cause a lock where libvlc_media_player_get_time and
    //   libvlc_media_player_stop are called in different threads and neither returns
    //
    if (_pFrameSource->_stoppingAllStreamsNow == true) {
        return;
    }

    libvlc_media_stats_t mediaStats;
    if (libvlc_media_get_stats(_media, &mediaStats)) {
        libvlc_time_t timeMs = libvlc_media_player_get_time(_mediaPlayer);
        if (timeMs != 0) {
            double time = timeMs / 1000.0;
            if (firstTimeEver == 0.0)  {
                firstTimeEver = time;
                time = 0;
            }
            else {
                // If the time goes backwards, then start over
                if (time < firstTimeEver){
                    firstTimeEver = time;
                    time = 0;
                    // Reset everything to zero when this happens
                    readByteIndex = 0;
                    for (int i = 0; i < AVG_SIZE; i++) {
                        lastReadBytes[i] = 0;
                        lastTime[i] = 0.0;
                    }
                }
                else {
                    time -= firstTimeEver;
                    // if this is more than a second, than do not bother with this time.
                    //  In this case, start over
                    if (abs(time - lastTime[(readByteIndex + AVG_SIZE - 1) % AVG_SIZE]) > 5.0) {
                        firstTimeEver = timeMs / 1000.0;
                        time = 0;
                        // Reset everything to zero when this happens
                        readByteIndex = 0;
                        for (int i = 0; i < AVG_SIZE; i++) {
                            lastReadBytes[i] = 0;
                            lastTime[i] = 0.0;
                        }
                    }
                }
            }
                
            if (mediaStats.i_lost_pictures != lastLostPicture) {
                // Statistics are not accurate of this one - start over???
                // Actually the time and data seems to increase, so just keep going
                lastLostPicture = mediaStats.i_lost_pictures;
                /*
                readByteIndex = 0;
                for (int i = 0; i < AVG_SIZE; i++) {
                    lastReadBytes[i] = 0;
                    lastTime[i] = 0.0;
                }
                firstTimeEver = timeMs / 1000.0;
                */
            }
            else if (lastTime[(readByteIndex + AVG_SIZE - 1) % AVG_SIZE] != time) {
                lastReadBytes[readByteIndex] = mediaStats.i_demux_read_bytes;
                lastTime[readByteIndex] = time;

                // Sum the last AVG_SIZE
                int dataSizeInBytes = lastReadBytes[readByteIndex] - lastReadBytes[(readByteIndex + 1) % AVG_SIZE];
                double timeDifference = lastTime[readByteIndex] - lastTime[(readByteIndex + 1) % AVG_SIZE];
                //assert(timeDifference >= 0.0);
                //assert(dataSizeInBytes >= 0);


                readByteIndex++;
                readByteIndex %= AVG_SIZE;

                if (time != 0)
                {
                    _pFrameSource->_averageBitRate = ((mediaStats.i_demux_read_bytes * 8.0) / time);
                    _pFrameSource->_rollingBitRate = ((dataSizeInBytes * 8.0) / timeDifference);
                }

                DBOUT("LAST 10:  " << _pFrameSource->_rollingBitRate << "\n");
                DBOUT("Bit Rate:     " << (mediaStats.f_demux_bitrate) << "\n");
                DBOUT("Lost Frames:  " << (mediaStats.i_lost_pictures) << "\n");
                DBOUT("Antoher BR:   " << _pFrameSource->_averageBitRate << "\n");
            }
            else {
                // Same data as last time - don't worry about it . .. .
            }
        }
    }

}

//////////////////////////////////////////////////////////////////////////////////
void RtpFrameSource::Stream::unlockFrame(unsigned char* pY,
        unsigned char* pU, unsigned char* pV)
{
    ++_pFrameSource->_frameCount;

    // TODO: should be time-synchonrizing here.  For now just assume
    // that we're getting frames from all streams at the same rate on average,
    // so we should only render after every _streams.size() frames is received.
    bool lastInGroup =
        (_pFrameSource->_frameCount % _pFrameSource->_streams.size()) == 0;
    _pFrameSource->_releaseBufferCallback(_streamIndex, pY, pU, pV,
            lastInGroup, _pFrameSource->_averageBitRate, _pFrameSource->_rollingBitRate);
}

