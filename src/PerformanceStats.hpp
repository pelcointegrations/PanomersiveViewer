//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_PERFORMANCE_STATS_HPP__
#define __PELCO_IMEXTK_PERFORMANCE_STATS_HPP__

#include <vector>
#include <map>
#include <mutex>
#include <chrono>

//
// Keeps track of video decode and rendering performance for up to 6 streams,
// and any number of views.
//
class PerformanceStats {
    public:
        PerformanceStats();
        virtual ~PerformanceStats();

        // returns a viewId;
        int addView();
        void removeView(int viewId);

        void resetStreamStats(size_t streamIndex);
        void activateStream(size_t streamIndex);
        void incrementDroppedFrames(size_t streamIndex);
        void incrementRenderedFrames(size_t streamIndex);
        void setAverageBitrate(size_t streamIndex, double bitRate);
        void setRollingBitrate(size_t streamIndex, double bitRate);

        static double getRateInSeconds(size_t count,
                const std::chrono::steady_clock::time_point& since);

        struct StreamStats {
                StreamStats() : _active(false),
                                _droppedFrames(0),
                                _renderedFrames(0),
                                _firstRenderedFrameTime(),
                                _averageBitRate(0.0),
                                _rollingBitRate(0.0)
                {
                }
                double getRenderedFramesPerSecond() const {
                    return PerformanceStats::getRateInSeconds(_renderedFrames,
                            _firstRenderedFrameTime);
                }
                double getTotalFramesPerSecond() const {
                    return PerformanceStats::getRateInSeconds(_droppedFrames + _renderedFrames,
                            _firstRenderedFrameTime);
                }
                double getAverageBitrate() const {
                    return _averageBitRate;
                }
                double getRollingBitrate() const {
                    return _rollingBitRate;
                }

                bool _active;
                size_t _droppedFrames;
                size_t _renderedFrames;
                std::chrono::steady_clock::time_point _firstRenderedFrameTime;
                double _averageBitRate;
                double _rollingBitRate;
        };
        void getStreamStats(size_t streamIndex, StreamStats& stats);

        void incrementRenderedViews(int viewId);

        struct ViewStats {
                ViewStats() : _renderedViews(0),
                              _firstRenderedViewTime()
                {
                }

                double getRenderedViewsPerSecond() const {
                    return PerformanceStats::getRateInSeconds(_renderedViews,
                            _firstRenderedViewTime);
                }

                size_t _renderedViews;
                std::chrono::steady_clock::time_point _firstRenderedViewTime;
        };
        void getViewStats(int viewId, ViewStats& stats);

    private:

        std::vector<StreamStats> _streams;

        std::map<int, ViewStats> _views;

        std::mutex _mutex;
};

#endif // __PELCO_IMEXTK_PERFORMANCE_STATS_HPP__
