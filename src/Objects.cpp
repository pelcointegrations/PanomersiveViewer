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
#include "Objects.hpp"

using namespace std;

namespace {

#define MAX_NUMBER_TRACK_OBJECTS     300  // maximum number of track objects

}

//////////////////////////////////////////////////////////////////////////////////
ObjectTrackInfo::ObjectTrackInfo(void)
    : _objTimeMs()
    , _speed(0)
    , _objs()
	, _averageSpeed(0)
	, _lastMoveIndex(0)
{
	_angularMoveOverlapped.distanceNSec = 0;
	_angularMoveOverlapped.lastMovingTimeMs = 0;
	_angularMoveNonOverlapped.distanceNSec = 0;
	_angularMoveNonOverlapped.lastMovingTimeMs = 0;
	_groundMoveOverlapped.distanceNSec = 0;
	_groundMoveOverlapped.lastMovingTimeMs = 0;
	_groundMoveNonOverlapped.distanceNSec = 0;
	_groundMoveNonOverlapped.lastMovingTimeMs = 0;
}

//////////////////////////////////////////////////////////////////////////////////
void ObjectTrackInfo::trimObjects(void)
{
    while (_objs.size() > MAX_NUMBER_TRACK_OBJECTS) {
        _objs.pop_front();
    }
}

//////////////////////////////////////////////////////////////////////////////////
void ObjectTrackInfo::addObject(const ObjectRect& obj, uint64_t timeMs)
{
    _objTimeMs = timeMs;
    _objs.push_back(obj);
    _objs.back()._objTimeMs = timeMs;
}

//////////////////////////////////////////////////////////////////////////////////
LastObject::LastObject(void)
    : ObjectRect()
{
}

//////////////////////////////////////////////////////////////////////////////////
void LastObject::setObject(const ObjectRect& obj, uint64_t timeMs)
{
    id = obj.id;
    left = obj.left;
    right = obj.right;
    top = obj.top;
    bottom = obj.bottom;
    panLeft = obj.panLeft;
    panRight = obj.panRight;
    tiltTop = obj.tiltTop;
    tiltBottom = obj.tiltBottom;
    _objTimeMs = timeMs;
}

//////////////////////////////////////////////////////////////////////////////////
bool LastObject::isSameLocation(const ObjectRect& obj)
{
    return ((id == obj.id) && (panLeft == obj.panLeft) && (panRight == obj.panRight) && (tiltTop == obj.tiltTop) &&
        (tiltBottom == obj.tiltBottom));
}
