//============================================================================
// Copyright (c) 2015 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "VideoWindowGroup.hpp"
#include "ImExDemoApp.hpp"

using namespace std;
using namespace pelco::imextk;

//////////////////////////////////////////////////////////////////////////////////
VideoWindowGroup::VideoWindowGroup(ImExDemoApp* app)
    : _app(app),
      _rtpStream(NULL),
      _viewMutex(),
      _views(),
      _frameFifo()
{
}

//////////////////////////////////////////////////////////////////////////////////
VideoWindowGroup::~VideoWindowGroup()
{
    stopStream();
}


//////////////////////////////////////////////////////////////////////////////////
void VideoWindowGroup::stopStream()
{
    if (_rtpStream != NULL) {
        _rtpStream->stop();
        delete _rtpStream;
        _rtpStream = NULL;
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindowGroup::restartStream(const std::string& ip, bool highRes)
{
    stopStream();

    _rtpStream = new RtpFrameSource(ip,
            [this](const FrameBufferRef& frame) {
                onStreamImageReceived(frame);
            },
            highRes);
//            bind(&VideoWindowGroup::onStreamImageReceived, this, placeholders::_1),
//        highRes);
    _rtpStream->start();
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindowGroup::onStreamImageReceived(const FrameBufferRef& frame)
{
    _frameFifo.write(frame);
    _app->postUpdateFrames(this);
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindowGroup::copyQueuedFramesToViews()
{
    unique_lock<mutex> lk(_viewMutex);
    while (!_frameFifo.willReadBlock()) {
        FrameBufferRef frame;
        _frameFifo.read(frame);
        set<VideoWindow*>::iterator itr;
        for (itr = _views.begin(); itr != _views.end(); ++itr) {
            (*itr)->UpdateStreamImage(frame);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindowGroup::linkView(VideoWindow* vw)
{
    unique_lock<mutex> lk(_viewMutex);
    _views.insert(vw);
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindowGroup::unlinkView(VideoWindow* vw)
{
    unique_lock<mutex> lk(_viewMutex);
    _views.erase(vw);
}

//////////////////////////////////////////////////////////////////////////////////
void VideoWindowGroup::unlinkAllViews()
{
    unique_lock<mutex> lk(_viewMutex);
    _views.clear();
}
