//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "PerformanceStats.hpp"

using namespace std;
using namespace chrono;

//////////////////////////////////////////////////////////////////////////////////
PerformanceStats::PerformanceStats()
    : _streams(7)
{
}

//////////////////////////////////////////////////////////////////////////////////
PerformanceStats::~PerformanceStats()
{
}

//////////////////////////////////////////////////////////////////////////////////
int PerformanceStats::addView()
{
    unique_lock<mutex> lock(_mutex);
    // get a unique ID.
    map<int, ViewStats>::const_iterator itr;
    int maxId = int(_views.size()) - 1;
    for (itr = _views.begin(); itr != _views.end(); ++itr) {
        maxId = max(itr->first, maxId);
    }

    int id = maxId + 1;
    _views[id] = ViewStats();

    return id;
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::removeView(int viewId)
{
    unique_lock<mutex> lock(_mutex);
    _views.erase(viewId);
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::resetStreamStats(size_t streamIndex)
{
    unique_lock<mutex> lock(_mutex);
    _streams.at(streamIndex) = StreamStats();
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::activateStream(size_t streamIndex)
{
    unique_lock<mutex> lock(_mutex);
    _streams.at(streamIndex)._active = true;
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::incrementDroppedFrames(size_t streamIndex)
{
    unique_lock<mutex> lock(_mutex);
    _streams.at(streamIndex)._droppedFrames += 1;
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::incrementRenderedFrames(size_t streamIndex)
{
    unique_lock<mutex> lock(_mutex);
    if (_streams.at(streamIndex)._renderedFrames == 0) {
        _streams.at(streamIndex)._firstRenderedFrameTime = steady_clock::now();
    }
    _streams.at(streamIndex)._renderedFrames += 1;
}

void PerformanceStats::setAverageBitrate(size_t streamIndex, double bitrate)
{
    unique_lock<mutex> lock(_mutex);
    _streams.at(streamIndex)._averageBitRate = bitrate;
}

void PerformanceStats::setRollingBitrate(size_t streamIndex, double bitrate)
{
    unique_lock<mutex> lock(_mutex);
    _streams.at(streamIndex)._rollingBitRate = bitrate;
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::getStreamStats(size_t streamIndex, StreamStats& stats)
{
    unique_lock<mutex> lock(_mutex);
    stats = _streams.at(streamIndex);
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::incrementRenderedViews(int viewId)
{
    unique_lock<mutex> lock(_mutex);
    if (_views.at(viewId)._renderedViews == 0) {
        _views.at(viewId)._firstRenderedViewTime = steady_clock::now();
    }
    _views.at(viewId)._renderedViews += 1;
}

//////////////////////////////////////////////////////////////////////////////////
void PerformanceStats::getViewStats(int viewId, ViewStats& stats)
{
    unique_lock<mutex> lock(_mutex);
    stats = _views.at(viewId);
}

//////////////////////////////////////////////////////////////////////////////////
double PerformanceStats::getRateInSeconds(size_t count,
        const steady_clock::time_point& since)
{
    auto timeDiff = steady_clock::now() - since;
    return double(count) /
        (double(duration_cast<milliseconds>(timeDiff).count())
                / 1000.0);
}
