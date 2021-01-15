//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "wgl.hpp"
#include <GL/glew.h>
#include <GL/wglew.h>
#include <sstream>

using namespace std;

//////////////////////////////////////////////////////////////////////////////////
HGLRC createGlContext(HDC forDC, HGLRC shareWith)
{
    UINT bufferOptions = PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW |
        PFD_DOUBLEBUFFER;

    PIXELFORMATDESCRIPTOR pfd = {
        sizeof(PIXELFORMATDESCRIPTOR),  /* size */
        1,                              /* version */
        bufferOptions,
        PFD_TYPE_RGBA,                  /* color type */
        32,                             /* prefered color depth */
        0, 0, 0, 0, 0, 0,               /* color bits (ignored) */
        0,                              /* no alpha buffer */
        0,                              /* alpha bits (ignored) */
        0,                              /* no accumulation buffer */
        0, 0, 0, 0,                     /* accum bits (ignored) */
        32,                             /* depth buffer */
        0,                              /* no stencil buffer */
        0,                              /* no auxiliary buffers */
        PFD_MAIN_PLANE,                 /* main layer */
        0,                              /* reserved */
        0, 0, 0,                        /* no layer, visible, damage masks */
    };
    int pixelFormat = ChoosePixelFormat(forDC, &pfd);
    if (pixelFormat == 0) {
        throw runtime_error("ChoosePixelFormat failed");
    }

    if (SetPixelFormat(forDC, pixelFormat, &pfd) != TRUE) {
        throw runtime_error("SetPixelFormat failed");
    }

    HGLRC hGLRC = NULL;
    if (shareWith == NULL) {
        hGLRC = wglCreateContext(forDC);
    }
    else {
        GLenum err = glewInit();
        if (err != GLEW_OK) {
            stringstream msg;
            msg << "glewInit failed: " << glewGetErrorString(err);
            throw runtime_error(msg.str());
        }

        PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB = 0;
        wglCreateContextAttribsARB = (PFNWGLCREATECONTEXTATTRIBSARBPROC) wglGetProcAddress("wglCreateContextAttribsARB");
        if(wglCreateContextAttribsARB == 0) {
            throw runtime_error("Required wgl extension not found -- this app requires newer graphics hardware");
        }

        hGLRC = wglCreateContextAttribsARB(forDC, shareWith, NULL);
    }
    if (hGLRC == NULL) {
        throw runtime_error("Unable to create opengl context");
    }

    return hGLRC;
}



