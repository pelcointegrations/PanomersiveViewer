//============================================================================
// Copyright (c) 2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "ErrorHandling.hpp"
#include <sstream>
#include <stdexcept>
#include <string.h>
#include <vector>

void throwImextkError(const std::string& prefix, PelcoImextk_Result res,
    PelcoImextk_Context ctx)
{
    std::stringstream errMsg;
    errMsg << prefix  << ": ";
    switch (res) {
        case PELCO_IMEXTK_ERR_UNKNOWN_OBJECT:
            errMsg << "Internal error: Invalid object ID passed to imextk function";
            break;
        case PELCO_IMEXTK_ERR_INVALID_ARGS:
            errMsg << "Internal error: Invalid argument passed to imextk function";
            break;
        case PELCO_IMEXTK_ERR_MEM_ALLOC:
            errMsg << "Memory Allocation Error";
            break;
        case PELCO_IMEXTK_ERR_OPEN_GL_LIMITS_EXCEEDED:
            errMsg << "OpenGl Limits exceeded - need to upgrade graphics hardware and/or driver";
            break;
        case PELCO_IMEXTK_ERR_UNKNOWN:
            errMsg << "ImExTk Internal Error";
            break;
        case PELCO_IMEXTK_ERR_CAMERA_VERSION:
            errMsg << "Camera version mismatch - need to update ImExTk to view this camera";
            break;
        case PELCO_IMEXTK_ERR_OUT_OF_BOUNDS_ARGS:
            errMsg << "Internal error: Out of bounds arg(s) passed to imextk function";
            //return; //not a critical error
            break;
    }

    int errStringLength = 0;
    if ((pelco_imextk_get_last_error_details(ctx, NULL, 0, &errStringLength) ==
                    PELCO_IMEXTK_ERR_BUFFER_TOO_SHORT) &&
            (errStringLength > 0)) {
        std::vector<char> errBuf(errStringLength + 1, '\0');
        if ((pelco_imextk_get_last_error_details(ctx, &errBuf[0], int(errBuf.size()),
                                &errStringLength) ==
                        PELCO_IMEXTK_NO_ERROR)) {
            errMsg << ": " << &errBuf[0];
        }
    }

    throw std::runtime_error(errMsg.str());
}
