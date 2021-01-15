//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include <atlstr.h>
#include <algorithm>
#include <functional>
#include <assert.h>
#include <chrono>
#include <limits>
#include <sstream>
#include "ObjectFilter.hpp"

using namespace std;

//#define _DEBUG_OUTPUT
//#define _OBJ_DEBUG_OUT
//#define _SELECT_DEBUG_OUT

#ifdef _DEBUG_OUTPUT
#define DOUT( s )              \
{                              \
   std::wostringstream os_;    \
   os_ << s;                   \
   OutputDebugStringW( os_.str().c_str() );  \
}
#else
#define DOUT( s )
#endif

#ifdef _OBJ_DEBUG_OUT
#define OBJ_DOUT( s )                        \
{                                            \
   std::wostringstream os_;                  \
   os_ << s;                                 \
   OutputDebugStringW( os_.str().c_str() );  \
}
#else
#define OBJ_DOUT( s )
#endif

#ifdef _SELECT_DEBUG_OUT
#define SEL_DOUT( s )                        \
{											 \
   std::wostringstream os_;					 \
   os_ << s;								 \
   OutputDebugStringW( os_.str().c_str() );  \
}
#else
#define SEL_DOUT( s )
#endif


namespace {

    //////////////////////////////////////////////////////////////////////////////////
    bool pointInRect(float x, float y, const ObjectRect& obj)
    {
        return ((x > obj.panLeft) && (x < obj.panRight) && (y > obj.tiltBottom) && (y < obj.tiltTop));
    }

    //////////////////////////////////////////////////////////////////////////////////
    void printObjectRect(char *name, vector<ObjectRect>& panTilt)
    {
        DOUT(endl << "print " << name <<  " size " << panTilt.size() << endl);
        for (auto obj : panTilt) {
            DOUT(" id: " << obj.id << " lt: " << obj.left << "x" << obj.top << "  rb: " << obj.right << "x" << obj.bottom << endl);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////
    float calculateSpeed(const ObjectRect& first, const ObjectRect& second, float *diffX = NULL,
        float *diffY = NULL)
    {
        int64_t timeDiff = (first._objTimeMs > second._objTimeMs) ? (int64_t)(first._objTimeMs - second._objTimeMs) : (int64_t) (second._objTimeMs - first._objTimeMs);
        float distance = first.distance(second, diffX, diffY);
		float pan1, tilt1, pan2, tilt2;
		first.getCenter(pan1, tilt1);
		second.getCenter(pan2, tilt2);
		float speed = distance * 1000 / (float)timeDiff;
		//DOUT(" calculateSpeed: id: " << first.id << " time1: " << first._objTimeMs << " time2: " << second._objTimeMs << " timeDiff: " << timeDiff << " pan1: " << pan1 << " tilt1: " << tilt1 << " pan2: " << pan2 << " tilt2: " << tilt2 << " distance: " << distance << " speed: " << speed << endl);
		return distance * 1000 / (float)timeDiff;
	}
}

//////////////////////////////////////////////////////////////////////////////////
uint64_t getCurrentTimeMs(void)
{
    return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
}

//////////////////////////////////////////////////////////////////////////////////
ObjectFilter::ObjectFilter(CameraInfos& cameraInfos)
	: _lastObject()
    , _tracks()
	, _followObject()
	, _cameraInfos(cameraInfos)
{
}

//////////////////////////////////////////////////////////////////////////////////
void ObjectFilter::removeOldTracks(void)
{
    // remove old tracks
    uint64_t timeMs = getCurrentTimeMs();
    for (std::map<int, ObjectTrackInfo>::iterator it = _tracks.begin(); it != _tracks.end();) {
        if ((timeMs - it->second._objTimeMs) > MAX_TRACK_TIME_MSEC) {
            it = _tracks.erase(it);
        }
        else {
            ++it;
        }
    }
    // trim tracks
    if (_tracks.size() > MAX_NUMBER_TRACKS) {
        vector<uint64_t> objectTimes;
        for (auto track : _tracks) {
            objectTimes.push_back(track.second._objTimeMs);
        }
        uint64_t referenceMs = objectTimes[MAX_NUMBER_TRACKS - 1];
        std::sort(objectTimes.begin(), objectTimes.end(), std::greater<uint64_t>());
        for (map<int, ObjectTrackInfo>::iterator it = _tracks.begin(); it != _tracks.end();) {
            if (it->second._objTimeMs < referenceMs) {
                it = _tracks.erase(it);
            }
            else {
                ++it;
            }
        }
    }
    // trim objects in track
    for (auto track : _tracks) {
        track.second.trimObjects();
    }
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::findPreviousObject(const std::vector<ObjectRect>& panTilt, int &idx, bool& holdLastObject)
{
	CString str;
    if (_lastObject.id != -1) {
        int id = _lastObject.id;
        // check if there is object with the same id, if so, use it
        vector<ObjectRect>::const_iterator it = find_if(panTilt.begin(), panTilt.end(),
			[&id](const ObjectRect& obj) { return obj.id == id; });
		float speed = 0, aveSpeed = 0, distMoved = 0;
		getObjectSpeed(id, speed, aveSpeed);
		objMoveInfo move;
		getObjectMoveInfo(id, GROUND_DIST_NON_OVERLAPPED, move);
		int64_t timeSinceLastMove = getCurrentTimeMs() - move.lastMovingTimeMs;
		int64_t timeSinceLastObject = getCurrentTimeMs() - _lastObject._objTimeMs;
        if (it != panTilt.end()) {
            // abandon this object if it has stayed at the same location longer than MAX_STILL_OBJECT_WAIT_TIME
			if (!(move.distanceNSec < _cameraInfos._cameraOptera.getMinGroundMoveDistance() && (timeSinceLastMove > MAX_STILL_OBJECT_WAIT_TIME_MSEC))) {
				str.Format(_T("SEL: PREV_ID:0 %3d distMoved: %8.5f aveSpeed: %8.4f timeSinceLastObject: %4d timeSinceLastMove: %4d"), id, move.distanceNSec, aveSpeed, timeSinceLastObject, timeSinceLastMove);
				SEL_DOUT((LPCTSTR)str);
				str.Format(_T(" NEW_ID: %3d reason: %2d distMoved: %8.5f aveSpeed: %8.4f timeSinceLastMove: %4d currTime: %14llu\n"), id, SELECT_PREV_OBJ_MOVING_OR_WAITING, move.distanceNSec, aveSpeed, timeSinceLastMove, getCurrentTimeMs());
				SEL_DOUT((LPCTSTR)str);
				idx = (int)(it - panTilt.begin());
                return true;
            }
        }
		// last object disappeared, check wait time before abandoning
		str.Format(_T("SEL: PREV_ID:1 %3d distMoved: %8.5f aveSpeed: %8.4f timeSinceLastObject: %4d timeSinceLastMove: %4d"), id, move.distanceNSec, aveSpeed, timeSinceLastObject, timeSinceLastMove);
		SEL_DOUT((LPCTSTR)str);
		if (timeSinceLastObject < DISAPPEARED_OBJECT_WAIT_TIME_MSEC) {
			holdLastObject = true;  //
			idx = -1;               // this will cause nothing to happen this frame
			str.Format(_T(" NEW_ID: %3d reason: %2d distMoved: %8.5f aveSpeed: %8.4f timeSinceLastMove: %4d currTime: %14llu\n"), id, SELECT_PREV_OBJ_MISSING_WAITING, move.distanceNSec, aveSpeed, timeSinceLastMove, getCurrentTimeMs());
			SEL_DOUT((LPCTSTR)str);
			return true;
		}
		else {
			return false;
		}
    }
	// this is 1st object in track - no previous objects
	str.Format(_T("SEL: PREV_ID:2 %3d distMoved: %8.5f aveSpeed: %8.4f timeSinceLastObject: %4d timeSinceLastMove: %4d"), -1, -1.0f, -1.0f, -1, -1);
	SEL_DOUT((LPCTSTR)str);
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::findAdjacentObject(const std::vector<ObjectRect>& panTilt, int &idx) {

	CString str;
	// check time lapse - only do this if not too long since last object moved
	if ((getCurrentTimeMs() - _lastObject._objTimeMs) < MAX_STILL_OBJECT_WAIT_TIME_MSEC) {
		// find closest object that is still moving
		float minDis = FLT_MAX;
		float speed = 0, aveSpeed = 0;
		objMoveInfo move;
		int index = -1;
		for (int i = 0; i < panTilt.size(); ++i) {
			float dis = panTilt[i].distance(_lastObject);
			getObjectSpeed(panTilt[i].id, speed, aveSpeed);	
			getObjectMoveInfo(panTilt[i].id, GROUND_DIST_OVERLAPPED, move);
			if ((dis < minDis) && (aveSpeed > MIN_FOLLOW_SPEED) && (move.distanceNSec > _cameraInfos._cameraOptera.getMinGroundMoveDistance())) {
				minDis = dis;
				index = i;
			}
		}
		if (index != -1) {
			// check distance
			if (minDis < MAX_OBJECT_DISTANCE) {
				getObjectSpeed(panTilt[index].id, speed, aveSpeed);
				getObjectMoveInfo(panTilt[index].id, GROUND_DIST_OVERLAPPED, move);
				idx = index;	
				str.Format(_T(" NEW_ID: %3d reason: %2d distMoved: %8.5f aveSpeed: %8.4f timeSinceLastMove: %4d currTime: %14llu\n"), panTilt[index].id, SELECT_ADJACENT_OBJ_FOUND, move.distanceNSec, aveSpeed, getCurrentTimeMs() - move.lastMovingTimeMs, getCurrentTimeMs());
				SEL_DOUT((LPCTSTR)str);
				return true;
			}
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::findLargestObject(const std::vector<ObjectRect>& panTilt, int &idx) {

	// find biggest object that is still moving - objects are already sorted by size
	CString str;
	idx = -1;
	for (int i = 0; i < panTilt.size(); ++i) {
		// get speed of object
		float speed = 0, aveSpeed = 0, distMoved = 0;
		getObjectSpeed(panTilt[i].id, speed, aveSpeed);

		objMoveInfo move;
		getObjectMoveInfo(panTilt[i].id, GROUND_DIST_OVERLAPPED, move);
		if (aveSpeed > MIN_FOLLOW_SPEED && move.distanceNSec > _cameraInfos._cameraOptera.getMinGroundMoveDistance()) {
			idx = i;	
			str.Format(_T(" NEW_ID: %3d reason: %2d distMoved: %8.5f aveSpeed: %8.4f timeSinceLastMove: %4d currTime: %14llu\n"), panTilt[i].id, SELECT_LARGEST_OBJ_FOUND, move.distanceNSec, aveSpeed, getCurrentTimeMs() - move.lastMovingTimeMs, getCurrentTimeMs());
			SEL_DOUT((LPCTSTR)str);
			return true;;
		}
	}
	str.Format(_T(" NEW_ID: %3d reason: %2d distMoved: %8.5f aveSpeed: %8.4f lastMoveTime: %4d currTime: %14llu\n"), -1, SELECT_NO_OBJ_FOUND, -1.0f, -1.0f, -1, getCurrentTimeMs());
	SEL_DOUT((LPCTSTR)str);
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
void ObjectFilter::updateTracks(const std::vector<ObjectRect>& panTilt)
{
	uint64_t timeMs = getCurrentTimeMs();
	CString str;

    // house keeping
    removeOldTracks();

    // add object to tracks
    for (auto obj : panTilt) {
        _tracks[obj.id].addObject(obj, timeMs);
        DOUT(" ObjectFilter: obj id " << obj.id << endl);
    }

    // calculate speed for each object ID
    for (map<int, ObjectTrackInfo>::iterator it = _tracks.begin(); it != _tracks.end(); ++it) {
		if (it->second._objs.size() > 1) {
			int thisIndex = (int)it->second._objs.size() - 1;
			int lastIndex = thisIndex - 1;
			// check if an obj was entered this frame
			if (it->second._objs[thisIndex]._objTimeMs == timeMs) {
				// calc object speed
				it->second._objs[thisIndex].speed = calculateSpeed(it->second._objs[thisIndex], it->second._objs[lastIndex]);

				// calc track speed (average last 2 obj speeds)
				// ignore zero object speeds unless they occur 2 or more times in a row
				if ((it->second._objs[thisIndex].speed > 0) ||
					((it->second._objs[thisIndex].speed == 0) && (it->second._objs[lastIndex].speed == 0))) {
					it->second._speed = (it->second._objs[thisIndex].speed + it->second._objs[it->second._lastMoveIndex].speed) / 2;
					it->second._lastMoveIndex = thisIndex;
				}

				// calc average track speed (average last n obj speeds)
				// ignore zero object speeds unless they occur 2 or more times in a row
				float accum = 0; int count = 0;	bool skipZeroSpeed = false;
				for (deque<ObjectRect>::reverse_iterator it2 = it->second._objs.rbegin(); it2 != it->second._objs.rend(); ++it2) {  // reverse search
					if ((it2->speed > 0) || (skipZeroSpeed)) {
						accum += it2->speed;
						count++;
						skipZeroSpeed = false;
						if (count >= AVERAGE_SPEED_SAMPLES) break;
					}
					else {
						skipZeroSpeed = true;
					}
				}
				it->second._objs[thisIndex].aveSpeed = accum / count;
				it->second._averageSpeed = it->second._objs[thisIndex].aveSpeed;

				// calc if edges moved
				it->second._objs[thisIndex].edgeChanged.top    = (it->second._objs[thisIndex].top    != it->second._objs[lastIndex].top)    ? 1 : 0;
				it->second._objs[thisIndex].edgeChanged.bottom = (it->second._objs[thisIndex].bottom != it->second._objs[lastIndex].bottom) ? 1 : 0;
				it->second._objs[thisIndex].edgeChanged.left   = (it->second._objs[thisIndex].left   != it->second._objs[lastIndex].left)   ? 1 : 0;
				it->second._objs[thisIndex].edgeChanged.right  = (it->second._objs[thisIndex].right  != it->second._objs[lastIndex].right)  ? 1 : 0;

				// calc if this is a non-overlapping box
				if (lastIndex == 0) {
					it->second._objs[lastIndex].nonOverlapped = 0;
				}
				if ((it->second._objs[thisIndex].panLeft     > it->second._objs[lastIndex].panRight   ||
				 	 it->second._objs[thisIndex].panRight    < it->second._objs[lastIndex].panLeft    ||
					 it->second._objs[thisIndex].tiltTop     < it->second._objs[lastIndex].tiltBottom ||
					 it->second._objs[thisIndex].tiltBottom  > it->second._objs[lastIndex].tiltTop)) {
					 it->second._objs[thisIndex].nonOverlapped = 1;
				}
				else {
					it->second._objs[thisIndex].nonOverlapped = 0;
				}

				// calc dist moved in last N seconds
				int timeIndexOverlapped = 0, timeIndexNonOverlapped = 0, objIndex = 0;
				for (objIndex = thisIndex; objIndex >= 0; objIndex--) {  // reverse search
					// calc 2 times: 
					//   - one up to N seconds back extending over non-overlapping boxes
					//   - one up to N seconds back not allowed to extend over non-overlapping boxes
					if (timeIndexOverlapped == 0) {
						if ((timeMs - it->second._objs[objIndex]._objTimeMs > DIST_CALC_TIME_MSEC) || (it->second._objs[objIndex].nonOverlapped)) {
							timeIndexOverlapped = objIndex;
						}
					}
					// allow distance to extend over non-overlapping boxes
					if (timeIndexNonOverlapped == 0) {
						if (timeMs - it->second._objs[objIndex]._objTimeMs > DIST_CALC_TIME_MSEC) {
							timeIndexNonOverlapped = objIndex;
						}
					}
					if (timeIndexOverlapped && timeIndexNonOverlapped) { break; }
				}
				//it->second._distanceNSec = it->second._objs[thisIndex].distance(it->second._objs[timeIndexNonOverlapped]);

				// calc change in angular distances - overlapped and non-overlapped cases
				it->second._angularMoveOverlapped.distanceNSec    = it->second._objs[thisIndex].distance(it->second._objs[timeIndexOverlapped]);
				it->second._angularMoveNonOverlapped.distanceNSec = it->second._objs[thisIndex].distance(it->second._objs[timeIndexNonOverlapped]);

				// Calc change in ground distance over N seconds
				// get current ground location
				float opteraToTargetXFeet0Sec, opteraToTargetYFeet0Sec, opteraToTargetXFeetNSec, opteraToTargetYFeetNSec;
				float opteraPanRad, opteraTiltRad, correctedOpteraTiltRad;
				ObjectRect opteraObj = it->second._objs[thisIndex];
				float targetPointHeight = _cameraInfos.getTargetHeightFromOpteraRadians(opteraObj);
				opteraObj.getCenter(opteraPanRad, opteraTiltRad);
				_cameraInfos.calcOpteraToTargetLocation(opteraPanRad, opteraTiltRad, targetPointHeight, correctedOpteraTiltRad, opteraToTargetXFeet0Sec, opteraToTargetYFeet0Sec, _cameraInfos.HORZ_TILT_AT_90_DEG);

				// get ground location extending N sec back only allowing overlapping boxes  
				opteraObj = it->second._objs[timeIndexOverlapped];
				targetPointHeight = _cameraInfos.getTargetHeightFromOpteraRadians(opteraObj);
				_cameraInfos.calcOpteraToTargetLocation(opteraPanRad, opteraTiltRad, targetPointHeight, correctedOpteraTiltRad, opteraToTargetXFeetNSec, opteraToTargetYFeetNSec, _cameraInfos.HORZ_TILT_AT_90_DEG);
				float deltaXMove = opteraToTargetXFeetNSec - opteraToTargetXFeet0Sec;
				float deltaYMove = opteraToTargetYFeetNSec - opteraToTargetYFeet0Sec;
				it->second._groundMoveOverlapped.distanceNSec = sqrt((deltaXMove * deltaXMove) + (deltaYMove * deltaYMove));

				// get ground location extending N sec back allowing non-overlapping boxes  
				opteraObj = it->second._objs[timeIndexNonOverlapped];
				targetPointHeight = _cameraInfos.getTargetHeightFromOpteraRadians(opteraObj);
				_cameraInfos.calcOpteraToTargetLocation(opteraPanRad, opteraTiltRad, targetPointHeight, correctedOpteraTiltRad, opteraToTargetXFeetNSec, opteraToTargetYFeetNSec, _cameraInfos.HORZ_TILT_AT_90_DEG);
				deltaXMove = opteraToTargetXFeetNSec - opteraToTargetXFeet0Sec;
				deltaYMove = opteraToTargetYFeetNSec - opteraToTargetYFeet0Sec;
				it->second._groundMoveNonOverlapped.distanceNSec = sqrt((deltaXMove * deltaXMove) + (deltaYMove * deltaYMove));
				
				// record last time object moved more than min threshold
				if (it->second._angularMoveOverlapped.distanceNSec > _cameraInfos._cameraOptera.getMinAngularMoveDistance()) {
					it->second._angularMoveOverlapped.lastMovingTimeMs = timeMs;
				}
				if (it->second._angularMoveNonOverlapped.distanceNSec > _cameraInfos._cameraOptera.getMinAngularMoveDistance()) {
					it->second._angularMoveNonOverlapped.lastMovingTimeMs = timeMs;
				}
				if (it->second._groundMoveOverlapped.distanceNSec > _cameraInfos._cameraOptera.getMinGroundMoveDistance()) {
					it->second._groundMoveOverlapped.lastMovingTimeMs = timeMs;
				}
				if (it->second._groundMoveNonOverlapped.distanceNSec > _cameraInfos._cameraOptera.getMinGroundMoveDistance()) {
					it->second._groundMoveNonOverlapped.lastMovingTimeMs = timeMs;
				}
				str.Format(_T("OBJ: ID: %3d track_speed: %8.4f track_aveSpeed: %8.4f tiltTop: %7.5f tiltBottom: %7.5f panLeft: %7.5f panRight: %7.5f currTime: %14llu groundOlDistanceNsec: %8.5f groundNolDistanceNsec: %8.5f angularOlDistanceNsec: %7.5f angularNolDistanceNsec: %7.5f groundOlLastMovingTimeMs: %6llu groundNolLastMovingTimeMs: %6llu angularOlLastMovingTimeMs: %6llu angularNolLastMovingTimeMs: %6llu\n"),
					it->second._objs[thisIndex].id, it->second._speed, it->second._averageSpeed, it->second._objs[thisIndex].tiltTop, it->second._objs[thisIndex].tiltBottom, it->second._objs[thisIndex].panLeft, it->second._objs[thisIndex].panRight, timeMs, it->second._groundMoveOverlapped.distanceNSec, it->second._groundMoveNonOverlapped.distanceNSec, it->second._angularMoveOverlapped.distanceNSec, it->second._angularMoveNonOverlapped.distanceNSec, timeMs - it->second._groundMoveOverlapped.lastMovingTimeMs, timeMs - it->second._groundMoveNonOverlapped.lastMovingTimeMs, timeMs - it->second._angularMoveOverlapped.lastMovingTimeMs, timeMs - it->second._angularMoveNonOverlapped.lastMovingTimeMs);
				OBJ_DOUT((LPCTSTR)str);
			}
			else {
				// no object this frame, set track speed, aveSpeed = 0
				// TODO: should calc dist from last position to N sec back from current time.
				it->second._speed = 0;
				it->second._averageSpeed = 0;
			}
			DOUT("updateTracks: ID: " << it->second._objs[thisIndex].id << " obj speed: " << it->second._objs[thisIndex].speed << " track speed: " << it->second._speed << " track aveSpeed: " << it->second._averageSpeed << " objTime: " << it->second._objs[thisIndex]._objTimeMs << " currTime: " << timeMs << endl);
        }
        else {
			// this is the first object entered for this track
			it->second._objs.back().speed = 0;
			it->second._objs.back().aveSpeed = 0;
			it->second._speed = -1;
            it->second._averageSpeed = -1;
			it->second._lastMoveIndex = 0;
			it->second._objs.back().edgeChanged.top = 0;
			it->second._objs.back().edgeChanged.bottom = 0;
			it->second._objs.back().edgeChanged.left = 0;
			it->second._objs.back().edgeChanged.right = 0;
			it->second._angularMoveOverlapped.distanceNSec = 0;
			it->second._angularMoveNonOverlapped.distanceNSec = 0;
			it->second._groundMoveOverlapped.distanceNSec = 0;
			it->second._groundMoveNonOverlapped.distanceNSec = 0;
			it->second._angularMoveOverlapped.lastMovingTimeMs = 0;
			it->second._angularMoveNonOverlapped.lastMovingTimeMs = 0;
			it->second._groundMoveOverlapped.lastMovingTimeMs = 0;
			it->second._groundMoveNonOverlapped.lastMovingTimeMs = 0;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
int ObjectFilter::applyObjectFilters(const std::vector<ObjectRect>& panTilt)
{
    int idx = -1;
	bool holdLastObject = false;
    if (!panTilt.empty()) {
		// add pan/tilt obj to tracks, calc speed, avespeed, and dist
        updateTracks(panTilt);

        // check if any new objects have same ID as last object (and is not at same position for too long)
        if (!findPreviousObject(panTilt, idx, holdLastObject)) {
            // previous object gone or no longer moving - search for closest moving object
			if (!findAdjacentObject(panTilt, idx)) {
				// no movong objects close enough - search for biggest object that is still moving
				findLargestObject(panTilt, idx);
			}
        }

        // check to see if follow object
        //_followObject.findObject(panTilt, idx); // not working (not saving _time), is also redundant to above code.

		if (!holdLastObject) {
			_lastObject.id = (idx >= 0) ? panTilt[idx].id : -1;
		}
        DOUT(" applyObjectFilters: pantilt size " << panTilt.size() << " idx = " << idx << " objectId " << _lastObject.id << endl);
    }
    return idx;
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::hasTrackObject(const std::vector<ObjectRect>& panTilt, ObjectRect& object)
{
	// obtain index of object to track
    int idx = applyObjectFilters(panTilt);
    if (idx >= 0 && idx < (int) panTilt.size()) {
        object = panTilt[idx];
        if (!_lastObject.isSameLocation(object)) {
            _lastObject.setObject(object, getCurrentTimeMs());
        }
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::getObjectSpeed(int id, float &speed, float &aveSpeed)
{
    std::map<int, ObjectTrackInfo>::const_iterator it = _tracks.find(id);
    if (it != _tracks.end()) {
        speed = it->second._speed;
        aveSpeed = it->second._averageSpeed;
        return true;
    }
    speed = -1;
    aveSpeed = -1;
    return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::getObjectEdgesChanged(int id, RECT &edgeChanged)
{
	std::map<int, ObjectTrackInfo>::const_iterator it = _tracks.find(id);
	if (it != _tracks.end()) {
		edgeChanged = it->second._objs.back().edgeChanged;
		return true;
	}
	edgeChanged.bottom = -1;
	edgeChanged.top = -1;
	edgeChanged.right = -1;
	edgeChanged.left = -1;
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::getObjectMoveInfo(int id, eDistType distType, objMoveInfo& move)
{
	std::map<int, ObjectTrackInfo>::const_iterator it = _tracks.find(id);
	if (it != _tracks.end()) {
		switch (distType) {
			case ANGULAR_DIST_OVERLAPPED: {
				move = it->second._angularMoveOverlapped;
				break;
			}
			case ANGULAR_DIST_NON_OVERLAPPED: {
				move = it->second._angularMoveNonOverlapped;
				break;
			}
			case GROUND_DIST_OVERLAPPED: {
				move = it->second._groundMoveOverlapped;
				break;
			}
			case GROUND_DIST_NON_OVERLAPPED: {
				move = it->second._groundMoveNonOverlapped;
				break;
			}
		}
		return true;
	}
	move.distanceNSec = 0;
	move.lastMovingTimeMs = 0;
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
bool ObjectFilter::setObjectMoveInfo(int id, eDistType distType, objMoveInfo move)
{
	std::map<int, ObjectTrackInfo>::iterator it = _tracks.find(id);
	if (it != _tracks.end()) {
		switch (distType) {
			case ANGULAR_DIST_OVERLAPPED: {
				it->second._angularMoveOverlapped = move;
				break;
			}
			case ANGULAR_DIST_NON_OVERLAPPED: {
				it->second._angularMoveNonOverlapped = move;
				break;
			}
			case GROUND_DIST_OVERLAPPED: {
				it->second._groundMoveOverlapped = move;
				break;
			}
			case GROUND_DIST_NON_OVERLAPPED: {
				it->second._groundMoveNonOverlapped = move;
				break;
			}
		}
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////
FollowObject::FollowObject(void)
    : _x(0)
    , _y(0)
    , _id(-1)
    , _time()
    , _mutex()
{
}

//////////////////////////////////////////////////////////////////////////////////
bool FollowObject::findObject(const std::vector<ObjectRect>& panTilt, int &idx)
{
	unique_lock<mutex> lk(_mutex);
    uint64_t ctime = getCurrentTimeMs();
    bool follow = (ctime - _time < MAX_FOLLOW_WAIT_TIME_MSEC) ? true : false;
    if (follow) {
        // check if there is object with the same id, if yes, use it
        vector<ObjectRect>::const_iterator it = find_if(panTilt.begin(), panTilt.end(),
            [this](const ObjectRect& obj) { return obj.id == _id; });
        if (it != panTilt.end()) {
            idx = (int)(it - panTilt.begin());
            // update location
            it->getCenter(_x, _y);
        }
        else {
            // check for object that contains the center of the last tracked object
            for (int i = 0; i < (int) panTilt.size(); ++i) {
                if (pointInRect(_x, _y, panTilt[i])) {
                    _id = panTilt[i].id;
                    // update x and y location to center
                    panTilt[i].getCenter(_x, _y);
                    idx = i;
                    break;
                }
            }
        }
        if (idx >= 0) {
            DOUT("FollowObject: follow find obj " << _x << "x" << _y << " idx " << idx << " id " << _id << " follow " << follow << endl);
        }
    }
    else {
        _id = -1;
    }
    return follow;
}

//////////////////////////////////////////////////////////////////////////////////
ZoomTimer::ZoomTimer(void)
	: _timeMs(0)
	, _lastZoom(0)
    , _zoomOutCount(0)
{
}

//////////////////////////////////////////////////////////////////////////////////
float ZoomTimer::determineNextZoom(float zoom, float speed, float aveSpeed, float zoomStep, int *zoomState)
{
    uint64_t ctime = getCurrentTimeMs();
    bool update = false;
	float speedZoomAdjustFactor = 0;
	float speedZoomLimit = 0;
	float zoomAdjusted = 0;
	*zoomState = 0;
	// Don't zoom in to size of analytics box, back off to allow some margin
	zoomAdjusted = zoom / ZOOM_REDUCTION_FACTOR;
	// Limit amount of zoom due to object speed
	speedZoomAdjustFactor = (MAX_OBJECT_SPEED - aveSpeed) / (MAX_OBJECT_SPEED);
	speedZoomLimit = speedZoomAdjustFactor > 0 ? zoomAdjusted * speedZoomAdjustFactor : 0;
	if (zoomAdjusted > speedZoomLimit) {
		zoomAdjusted = speedZoomLimit;
		*zoomState = 1;
	}
	if (zoomAdjusted != _lastZoom) {
		if (zoomAdjusted > _lastZoom) {
            // zoom in
            if (aveSpeed >= 0 && (ctime - _timeMs) > MIN_ZOOM_WAIT_TIME_MSEC) {
				_lastZoom = min((_lastZoom + zoomAdjusted) / 2, _lastZoom + zoomStep);
				*zoomState = *zoomState + 2;
                update = true;
            }
        }
        else {
            // zoom out
            if ((ctime - _timeMs) > MIN_ZOOM_WAIT_TIME_MSEC / 4) {
				_lastZoom = (_lastZoom + zoomAdjusted) / 2;
				*zoomState = *zoomState + 6;
                update = true;
            }
        }

		if (update) {
			_timeMs = ctime;
			_zoomOutCount = 0;
		}
    }
	if (timeToZoomOut(_lastZoom)) {
		*zoomState = *zoomState + 8;
	}
	DOUT("determineNextZoom: lastzoom: " << _lastZoom << " zoom: " << zoom << " zoomAdjust: " << speedZoomAdjustFactor << " zoomAdjusted: " << zoomAdjusted << " last time: " << _timeMs << " this time: " << ctime << " speed: " << speed << " aveSpeed: " << aveSpeed << " zoomState: " << *zoomState << " update: " << update << endl);
    return _lastZoom;
}

//////////////////////////////////////////////////////////////////////////////////
bool  ZoomTimer::timeToZoomOut(float& zoom)
{
    uint64_t ctime = getCurrentTimeMs();
    if ((ctime - _timeMs) > MAX_ZOOM_OUT_TIME_MSEC) {
        // minimum step
        _lastZoom = (_zoomOutCount > 4) ? 0 : (_lastZoom / 2);
        _zoomOutCount++;
        _timeMs = ctime;
        zoom = _lastZoom;
        DOUT(" time to zoom  " << _lastZoom << " time " << _timeMs << endl);
        return true;
	}
	zoom = _lastZoom;
    return false;
}

//////////////////////////////////////////////////////////////////////////////////
WriteObjectToFile::WriteObjectToFile(void)
    : _fileName()
    , _outFile() {
    setFileName("c:\\temp\\opteraMetadata.txt");
}

//////////////////////////////////////////////////////////////////////////////////
WriteObjectToFile::~WriteObjectToFile(void)
{
    if (_outFile.is_open()) {
        _outFile.close();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void WriteObjectToFile::setFileName(const std::string& fileName) {
    if (_outFile.is_open()) {
        _outFile.close();
    }
    _fileName = fileName;
    _outFile.open(_fileName.c_str());
}

//////////////////////////////////////////////////////////////////////////////////
void WriteObjectToFile::writeObjects(const std::vector<ObjectRect>& mosaics)
{
    _outFile << mosaics.size();
    for (auto obj : mosaics) {
        _outFile << " " << obj.id << " " << obj.left << " " << obj.top << " " << obj.right << " " << obj.bottom;
    }
    _outFile << endl;
}

//////////////////////////////////////////////////////////////////////////////////
ReadObjectFromFile::ReadObjectFromFile(void)
    : _fileName()
    , _inFile()
{
    setFileName("c:\\temp\\oldopteraMetadata.txt");
}

//////////////////////////////////////////////////////////////////////////////////
ReadObjectFromFile::~ReadObjectFromFile(void)
{
    if (_inFile.is_open()) {
        _inFile.close();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void ReadObjectFromFile::setFileName(const std::string& fileName) {
    if (_inFile.is_open()) {
        _inFile.close();
    }
    _fileName = fileName;
    _inFile.open(_fileName.c_str());
}

//////////////////////////////////////////////////////////////////////////////////
void ReadObjectFromFile::readObjects(std::vector<ObjectRect>& mosaics) {
    mosaics.clear();
    if (!_inFile.eof()) {
        int size;
        _inFile >> size;
        for (int i = 0; i < size; ++i) {
            ObjectRect obj;
            _inFile >> obj.id;
            _inFile >> obj.left;
            _inFile >> obj.top;
            _inFile >> obj.right;
            _inFile >> obj.bottom;
            mosaics.push_back(obj);
        }
        printObjectRect("test", mosaics);
    }
}
