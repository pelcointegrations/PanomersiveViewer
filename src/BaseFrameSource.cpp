#include "stdafx.h"
#include "BaseFrameSource.hpp"


BaseFrameSource::BaseFrameSource()
{
}


BaseFrameSource::~BaseFrameSource()
{
}

bool BaseFrameSource::hasUserMetadataText() {
    return false;
}

std::string BaseFrameSource::userMetadataText() 
{
    return NULL;
}