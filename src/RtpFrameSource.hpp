//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_RTP_FRAME_SOURCE_HPP__
#define __PELCO_IMEXTK_RTP_FRAME_SOURCE_HPP__

#include <string>
#include <vector>
#include <functional>
#include <vlc/vlc.h>
#include <chrono>
#include "BaseFrameSource.hpp"

class RtpFrameSource : public BaseFrameSource {
    public:
        RtpFrameSource(const RtpFrameSource&) = delete;
        RtpFrameSource& operator=(const RtpFrameSource&) = delete;

        // attaches to camera, and gets relevant stream info.
        RtpFrameSource(const std::string& cameraIP,
                const StreamSetupCallback& setup, const GetBufferCallback& getBuffer,
                const ReleaseBufferCallback& releaseBuffer,
                bool highRes, bool connectAsNonOpteraType, bool uniStream, bool primaryStream, const std::vector<std::string>& discoveryURLs, bool multicast,
                bool thisIsSitAwareness, int encoderChannelSelected);

        virtual ~RtpFrameSource();

        // start/stop the RTP stream.  Frames will be delivered to ImageUpdateCallback as they
        // are received.
        // Note if there are outstanding FrameBuffers when stop() is called,
        // some tearing may restult.  The best approach is to stop flush all outstanding
        // frames before calling stop().
        void start();
        void stop();

    private:
        void cleanup();

        const std::string      _cameraIP;
        const std::vector<std::string>      _discoveryURLs;
        libvlc_instance_t*     _vlcInstance;

        StreamSetupCallback _setupCallback;
        GetBufferCallback   _getBufferCallback;
        ReleaseBufferCallback _releaseBufferCallback;

        class Stream {
            public:
                Stream(RtpFrameSource* pFrameSource, size_t streamIndex,
                        const std::string& url, libvlc_instance_t* _instance, const std::string& layoutMetadata,
                        bool thisIsSitAwareness);
                virtual ~Stream();

                void start();
                void stopMe();

            private:
                void cleanup();
                uint32_t vlcSetup(char *chroma,
                        uint32_t *width,
                        uint32_t *height,
                        uint32_t *pitches,
                        uint32_t *lines);

                void lockFrame(unsigned char*& pY, unsigned char*& pU, unsigned char*& pV);
                void unlockFrame(unsigned char* pY, unsigned char* pU, unsigned char* pV);

                RtpFrameSource*            _pFrameSource;
                const size_t               _streamIndex;
                std::string                _url;
                libvlc_instance_t*         _pVlcInstance;
                std::string                _layoutMetadata;
                libvlc_media_t*            _media;
                libvlc_media_player_t*     _mediaPlayer;
                int                        _width;
                int                        _height;
                bool                       _thisIsSitAwareness;
        };
        friend class Stream;

        std::vector<Stream*> _streams;
        int _frameCount;
        double _averageBitRate;
        double _rollingBitRate;
        bool _stoppingAllStreamsNow;
};

#endif // __PELCO_IMEXTK_RTP_FRAME_SOURCE_HPP__
