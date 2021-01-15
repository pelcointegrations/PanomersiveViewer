//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_BASE_FRAME_SOURCE_HPP__
#define __PELCO_IMEXTK_BASE_FRAME_SOURCE_HPP__

#include <string>
#include <vector>
#include <functional>

class BaseFrameSource
{
public:
    BaseFrameSource();
    virtual ~BaseFrameSource();

    // So we are on the same page
    static const size_t kMaxDragonFlyVStreams = 6; //support for 270/360 cameras with more streams and beyond (limit to 6 for now?)


    // Configures a stream for playback.
    // Returns the number of frame buffers available via GetBufferCallback.
    // Important: Only I420 format is supported, and each buffer should
    // consist of 3 image planes (Y,U, and V).  Image planes must also be
    // 32-byte aligned in memory.
    typedef std::function<size_t(size_t streamIndex,
        size_t frameWidth, size_t frameHeight,
        const std::string& layoutMetadata, bool sitAwarenessStream)>
        StreamSetupCallback;

    // Get a buffer into which the next frame can be decoded.  Buffers
    // are returned as 3 separate planes.
    typedef std::function<void(size_t streamIndex, unsigned char*& pYPlane,
        unsigned char*& pUPlane, unsigned char*& pVPlane)>
        GetBufferCallback;

    // Return a filled-in frame buffer.
    // lastInGroup indicates whether the view should be re-rendered.
    // For high-resolution playback there are multiple video streams, and
    // lastInGroup will only be set to true after all (synchronized) video streams
    // have been updated.
    typedef std::function<void(size_t streamIndex, unsigned char* yPlane,
        unsigned char* uPlane, unsigned char* vPlane, bool lastInGroup, 
        double avgBitRate, double rollingBitRate)>
        ReleaseBufferCallback;

    // start/stop the RTP stream.  Frames will be delivered to ImageUpdateCallback as they
    // are received.
    // Note if there are outstanding FrameBuffers when stop() is called,
    // some tearing may restult.  The best approach is to stop flush all outstanding
    // frames before calling stop().
    virtual void start() = 0;
    virtual void stop() = 0;

    // This will return a string if there is one in the video metadata
    virtual bool hasUserMetadataText();
    virtual std::string userMetadataText();

};

#endif // __PELCO_IMEXTK_BASE_FRAME_SOURCE_HPP__