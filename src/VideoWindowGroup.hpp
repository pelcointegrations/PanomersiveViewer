//============================================================================
// Copyright (c) 2015 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_VIDEO_WINDOW_GROUP_HPP__
#define __PELCO_IMEXTK_VIDEO_WINDOW_GROUP_HPP__

#include "VideoWindow.hpp"
#include <string>
#include <pelco/imextk/RtpFrameSource.hpp>
#include <pelco/imextk/MultiThreadFifo.hpp>
#include <set>
#include <mutex>

// forward decl
class ImExDemoApp;

//
// Helper for ImExDemoApp.
// Manages the connection between an RtpFrameSource and
// a set of views of that source.
//
class VideoWindowGroup {
    public:
        VideoWindowGroup(const VideoWindowGroup&) = delete;
        VideoWindowGroup& operator=(const VideoWindowGroup&) = delete;

        VideoWindowGroup(ImExDemoApp* pApp);
        ~VideoWindowGroup();

        void restartStream(const std::string& ipAddress, bool hiRes);
        void stopStream();

        // add a view to the group.  Doesn't take ownership, just
        // forwards frames to the given window from the current source.
        void linkView(VideoWindow*);
        void unlinkView(VideoWindow*);
        void unlinkAllViews();

        // called by ImExDemoApp on UI thread
        void copyQueuedFramesToViews();

    private:
        friend class ImExDemoApp;
        void onStreamImageReceived(const pelco::imextk::FrameBufferRef& frame);

        ImExDemoApp* _app;
        pelco::imextk::RtpFrameSource* _rtpStream;
        std::mutex _viewMutex;
        std::set<VideoWindow*> _views;
        pelco::imextk::MTFifo<pelco::imextk::FrameBufferRef> _frameFifo;
};

#endif // __PELCO_IMEXTK_VIDEO_WINDOW_GROUP_HPP__
