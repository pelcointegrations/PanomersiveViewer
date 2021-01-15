//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_STREAM_LAYOUT_METADATA_H__
#define __PELCO_STREAM_LAYOUT_METADATA_H__

#include <string>
#include <vector>

typedef struct MediaDescriptorsInfo {
    MediaDescriptorsInfo(void) : rtspSubURL(NULL), layoutMetadata(NULL) {};
    MediaDescriptorsInfo(const std::string& rtspURLStr, const std::string& layoutMetadataStr) : rtspSubURL(rtspURLStr), layoutMetadata(layoutMetadataStr) {};
    std::string rtspSubURL;
    std::string layoutMetadata;
} LayoutMetadataInfo;

//
// Makes an RTSP DESCRIBE query to the given URL, and extracts the
// x-pelco-video-layout SDP attribute from the response.
// also extracts information from which we determine our substr for rtspURL
// if both sets of information are found they are added to the mediaDescriptorsInfo list.
// (Note: libVLC is also making an RTSP DESCRIBE call, but it's not clear
// how to access any of the results that libVLC doesn't directly make use of.
// Hence the need to duplicate some of that functionality here.)
//
extern void GetStreamLayoutMetadatas(const std::string& rtspUrl, std::vector<MediaDescriptorsInfo>& mediaDescriptorsInfo);

#endif // __PELCO_STREAM_LAYOUT_METADATA_H__
