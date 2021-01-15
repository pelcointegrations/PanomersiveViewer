//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "StreamLayoutMetadata.hpp"
#include "TcpFetcher.hpp"
#include <sstream>
#include <stdexcept>
#include <vector>
#include <functional>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////
string parseRtspUrl(const string& rtspUrl)
{
    const string prefix("rtsp://");
    if (rtspUrl.find(prefix) != 0) {
        throw runtime_error("Not an rtsp url");
    }

    size_t endPos = rtspUrl.find('/', prefix.size());
    if (endPos == string::npos) {
        endPos = rtspUrl.size();
    }
    return (endPos > prefix.size()) ?
        rtspUrl.substr(prefix.size(), endPos - prefix.size())
        : string();
}

//////////////////////////////////////////////////////////////////////////////////
void parseSdpString(const string& ln, char sep, string& key, string& val)
{
    string line = ln; // copy to allow same variable to be passed in to line and key or val
    size_t pos = line.find_first_of(sep);
    if (pos == string::npos) {
        //throw runtime_error(string("Missing separator in sdp line: ") + line);
    }
    key = line.substr(0, pos);
    val = line.substr(pos+1, line.length()-(pos+1));
    if (key.empty()) {
        throw runtime_error(string("Missing key in sdp line: ") + line);
    }
    if (val.empty()) {
        throw runtime_error(string("Missing value in sdp line: ") + line);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void parseSdpLine(const string& line, string& key, string& val)
{
    parseSdpString(line, '=', key, val);
}

//////////////////////////////////////////////////////////////////////////////////
void parseSdpAttr(const string& line, string& key, string& val)
{
    parseSdpString(line, ':', key, val);
}

//////////////////////////////////////////////////////////////////////////////////
void splitOn(vector<string>& result, const string& text,
        function<bool (const string& str, size_t& offset)> passDelimiter)
{
    size_t start = 0;
    size_t i = 0;
    while (i < text.size()) {
        size_t end = i;
        if (passDelimiter(text, i)) {
            result.push_back(text.substr(start, end-start));
            start = i;
        }
        else {
            ++i;
        }
    }

    if (i > start) {
        result.push_back(text.substr(start, i-start));
    }
}

//////////////////////////////////////////////////////////////////////////////////
void splitLines(vector<string>& result, const string& text)
{
    splitOn(result, text,
            [](const string& str, size_t& offset) {
                if (str[offset] == '\r') {
                    ++offset;
                    if ((str.size() == offset) ||
                            (str[offset] == '\n')) {
                        ++offset;
                    }
                    return true;
                }
                else {
                    return false;
                }
            }
        );
}

//////////////////////////////////////////////////////////////////////////////////
void splitWords(vector<string>& result, const string& text)
{
    splitOn(result, text, [](const string& str, size_t& offset) {
                bool retval = false;
                while ((offset < str.size()) && (isspace(str[offset]) != 0)) {
                    ++offset;
                    retval = true;
                }
                return retval;
            }
        );
}

//////////////////////////////////////////////////////////////////////////////////
#define CRLF "\r\n"
void GetStreamLayoutMetadatas(const string& rtspUrl, std::vector<MediaDescriptorsInfo>& mediaDescriptorsInfo)
{
    // Send RTSP DESCRIBE request to the rtspUrl and get the response.
    stringstream strm;
    strm << "DESCRIBE " << rtspUrl << " RTSP/1.0" << CRLF
        << "CSeq: 1" << CRLF
        << "User-Agent: Pelco Example Client" << CRLF
        << "Accept: application/sdp" << CRLF << CRLF;
    string ip = parseRtspUrl(rtspUrl);
    string response = tcpFetch(ip, 554, strm.str());

    vector<string> lines;
    splitLines(lines, response);

    // Header and body are separated by an empty line
    // - move to first non-empty line after the first empty line
    size_t bodyIndex = 0;
    for (size_t i = 0; i < lines.size(); ++i) {
        if (lines[i].empty()) {
            while ((++i < lines.size()) && lines[i].empty()) {
            }
            bodyIndex = i;
            break;
        }
    }
    if (bodyIndex == lines.size()) {
        throw runtime_error("Missing body in RTSP DESCRIBE response");
    }

    // Extract the custom video-layout-metadata attribute from
    // the video media descriptor
    string videoLayoutMetadata;
    string videoURLSubStr;
    bool videoLayoutMetadataFound = false;
    bool videoURLSubStrFound = false;
    size_t i = bodyIndex;
    while (i < lines.size()) {
        string key;
        string val;
        parseSdpLine(lines[i++], key, val);
        if (key == "m") { // start of media descriptor
            // is this a "video" media descriptor?
            vector<string> parts;
            splitWords(parts, val);
            if (parts.size() != 4) {
                throw runtime_error(
                    string("wrong number of parts in media description: ") + val);
            }
            ++i;
            if (parts[0] == "video") {
                // parse additional associated lines until we run out or
                // hit the next "m=" line
                videoLayoutMetadataFound = false;
                videoURLSubStrFound = false;
                while (i < lines.size()) {
                    // read/parse next key=val pair
                    string key;
                    string val;
                    parseSdpLine(lines[i++], key, val);
                    // current implementaion only handles a few keys
                    if (key == "a") {
                        // parse attribute
                        string attrKey;
                        string attrVal;
                        parseSdpAttr(val, attrKey, attrVal);
                        if (attrKey == "x-pelco-video-layout") {
                            videoLayoutMetadata = attrVal;
                            videoLayoutMetadataFound = true;
                            //break;
                        }
                        else if (attrKey == "control") {
                            videoURLSubStr = attrVal;
                            if (videoURLSubStr.compare("video") == 0) {
                                videoURLSubStr = "/mosaic";
                            }
                            else {
                                size_t pos = videoURLSubStr.find("_video", 0);
                                videoURLSubStr = "/" + videoURLSubStr.substr(0, pos + 2);
                            }
                            videoURLSubStrFound = true;
                        }
                    }
                    else if (key == "m") {
                        i--; //backup so while loop can parse this and check for next video type
                        break; // reached next media descriptor
                    }
                }

                //if we have both pieces of information then add to our list
                if (videoLayoutMetadataFound && videoURLSubStrFound) {
                    mediaDescriptorsInfo.push_back({ videoURLSubStr, videoLayoutMetadata });
                }
            }
        }
    }

    return;
}

