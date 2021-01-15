//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================

#ifndef __PELCO_IMEXTK_OBJECTS_HPP__
#define __PELCO_IMEXTK_OBJECTS_HPP__

#include <cstdint>
#include <vector>
#include <map>
#include <deque>
#include <mutex>
#include <fstream>

// coordinate in Endura coordinate range from 0 to 10000
// where 10000 means 100%
struct ObjectRect {
	unsigned int id;
	int left;
	int right;
	int top;
	int bottom;
	float panLeft;
	float panRight;
	float tiltTop;
	float tiltBottom;
	float speed;
	float aveSpeed;
	RECT edgeChanged;
	int nonOverlapped;
	uint64_t _objTimeMs; // time in millisecond
	float radSize(void) const { return abs(panRight - panLeft) * abs(tiltTop - tiltBottom); }
	void getCenter(float &centerX, float &centerY) const {
		centerX = (panLeft + panRight) / 2;
		centerY = (tiltTop + tiltBottom) / 2;
	}
	void getDimensions(float &height, float &width) const {
		width = abs(panLeft - panRight);
		height = abs(tiltTop - tiltBottom);
	}
	float distance(const ObjectRect& obj, float *diffX = NULL, float *diffY = NULL) const {
		float cx, cy;
		getCenter(cx, cy);
		float ox, oy;
		obj.getCenter(ox, oy);
		if (diffX != NULL) {
			*diffX = cx - ox;
		}
		if (diffY != NULL) {
			*diffY = cy - oy;
		}
		return (float) sqrt((cx - ox) * (cx - ox) + (cy - oy) * (cy - oy));
	}
	bool isSameObject(const ObjectRect& obj)
	{
		return ((id == obj.id) && (panLeft == obj.panLeft) && (panRight == obj.panRight) && (tiltTop == obj.tiltTop) &&
			(tiltBottom == obj.tiltBottom));
	}
};

struct LastObject : ObjectRect {
    void setObject(const ObjectRect& obj, uint64_t timeMs);
    bool isSameLocation(const ObjectRect& obj);
    LastObject(void);
};

struct objMoveInfo {
	float    distanceNSec;      // distance moved over last N seconds
	uint64_t lastMovingTimeMs;  // last time distance moved exceeded min threshold
};

struct ObjectTrackInfo {
	uint64_t    _objTimeMs;                 // time in millisecond
	float       _diffX;                     // degree in second
	float       _diffY;		               
	float       _speed;		               
	float       _averageSpeed;              // degree in second
	float       _predX;                     // predicted x
	float       _predY;                     // predicted y
	int         _lastMoveIndex;             // index of last time object moved
	objMoveInfo _angularMoveOverlapped;     // angular move info (overlapped)
	objMoveInfo _angularMoveNonOverlapped;  // angular move info (non-overlapped)
	objMoveInfo _groundMoveOverlapped;      // ground move info (overlapped)
	objMoveInfo _groundMoveNonOverlapped;   // ground move info (non-overlapped)
	std::deque<ObjectRect> _objs;
	ObjectTrackInfo(void);
	void addObject(const ObjectRect& obj, uint64_t timeMs);
	void trimObjects(void);
};

#endif
