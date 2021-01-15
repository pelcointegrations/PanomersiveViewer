//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================

#ifndef __PELCO_IMEXTK_OBJECTFILTER_HPP__
#define __PELCO_IMEXTK_OBJECTFILTER_HPP__

#include <cstdint>
#include <vector>
#include <map>
#include <deque>
#include <mutex>
#include <fstream>
#include "CameraInfos.hpp"
#include "ObjectFilter.hpp"
#include "Objects.hpp"

#define MAX_NUMBER_TRACK_OBJECTS           300  // maximum number of track objects
#define MAX_NUMBER_TRACKS                   30  // maximum number of tracks
#define MAX_TRACK_TIME_MSEC              15000  // maximum number of wait time before deleting an inactive track
#define MAX_STILL_OBJECT_WAIT_TIME_MSEC   3000  // maximum number of wait time for the same object
#define MAX_FOLLOW_WAIT_TIME_MSEC        15000  // maximum number of wait time for follow object
#define MAX_LAST_OBJECT_WAIT_TIME_MSEC    3000  // maximum number of wait time for the last object before switching to new object
#define MAX_OBJECT_DISTANCE               1.0f  // adjacent object distance (radians)
#define MAX_OBJECT_SPEED                 0.50f  // max object speed for zoom
#define MIN_ZOOM_WAIT_TIME_MSEC           4000  // wait time between zooms, except zoom out
#define MAX_ZOOM_OUT_TIME_MSEC           10000  // zoom out time if no activity
#define ZOOM_REDUCTION_FACTOR             3.0f  // expand zoom FOV larger than object box by this much 
#define AVERAGE_SPEED_SAMPLES                5  // number of samples used in average speed calc
#define MIN_FOLLOW_SPEED                 0.05f  // only follow objects at or above this speed 
#define DIST_CALC_TIME_MSEC               3000  // time to look back for distance calculation
#define MIN_ANGULAR_DIST_MOVED           0.12f  // min angular distance (radians) object must move to be considered a moving object
#define MIN_GROUND_DIST_MOVED            12.0f  // min ground distance (feet) object must move to be considered a moving object
#define DISAPPEARED_OBJECT_WAIT_TIME_MSEC 1500  // time to hold object position after it disappears

typedef enum {
	SELECT_PREV_OBJ_MOVING_OR_WAITING,
	SELECT_PREV_OBJ_MISSING_WAITING,
	SELECT_ADJACENT_OBJ_FOUND,
	SELECT_LARGEST_OBJ_FOUND,
	SELECT_NO_OBJ_FOUND
} eSelectReasons;

typedef enum {
	ANGULAR_DIST_OVERLAPPED,
	ANGULAR_DIST_NON_OVERLAPPED,
	GROUND_DIST_OVERLAPPED,
	GROUND_DIST_NON_OVERLAPPED
} eDistType;

uint64_t getCurrentTimeMs(void);

struct FollowObject {
    float _x;
    float _y;
    int _id;
    uint64_t   _time;
    std::mutex _mutex;
    // return if object is found and should be followed
    // also return true when current time and _time is less than MAX_FOLLOW_WAIT_TIME
    bool findObject(const std::vector<ObjectRect>& panTilt, int& index);
    FollowObject(void);
};

struct ObjectFilter {
	LastObject                                  _lastObject;
	std::map<int, ObjectTrackInfo>              _tracks;
	FollowObject                                _followObject;
	CameraInfos&                                _cameraInfos;
	// check if an object has been detected
	bool hasTrackObject(const std::vector<ObjectRect>& panTilt, ObjectRect& object);
	// filter by distance, time or track
	int applyObjectFilters(const std::vector<ObjectRect>& panTilt);
	// check to see if we should continue to follow last found object
	bool findPreviousObject(const std::vector<ObjectRect>& panTilt, int &idx, bool& holdLastObject);
	// find adjacent object
	bool findAdjacentObject(const std::vector<ObjectRect>& panTilt, int &idx);
	bool findLargestObject(const std::vector<ObjectRect>& panTilt, int &idx);
	void removeOldTracks(void);
	void updateTracks(const std::vector<ObjectRect>& panTilt);
	bool getObjectSpeed(int id, float &speed, float &aveSpeed);
	bool getObjectEdgesChanged(int id, RECT &edgeChanged);
	bool getObjectMoveInfo(int id, eDistType distType, objMoveInfo& moveInfo);
	bool setObjectMoveInfo(int id, eDistType distType, objMoveInfo moveInfo);
	ObjectFilter(CameraInfos& cameraInfos);
};

class WriteObjectToFile {
public:
    WriteObjectToFile(void);
    ~WriteObjectToFile(void);
    void setFileName(const std::string& fileName);
    void writeObjects(const std::vector<ObjectRect>& mosaics);
private:
    std::string _fileName;
    std::ofstream _outFile;
};

class ReadObjectFromFile {
public:
    ReadObjectFromFile(void);
    ~ReadObjectFromFile(void);
    void setFileName(const std::string& fileName);
    void readObjects(std::vector<ObjectRect>& mosaics);
private:
    std::string _fileName;
    std::ifstream _inFile;
};

struct ZoomTimer {
    uint64_t   _timeMs;
    float      _lastZoom;
    float      _zoomOutCount;
	float determineNextZoom(float zoom, float speed, float aveSpeed, float zoomStep, int* zoomState);
    bool  timeToZoomOut(float& zoom);
    ZoomTimer(void);
};

#endif
