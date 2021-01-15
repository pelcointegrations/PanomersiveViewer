//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_WGL_HPP__
#define __PELCO_IMEXTK_WGL_HPP__

#include "Window.hpp"

extern HGLRC createGlContext(HDC forDC, HGLRC shareWith);

//////////////////////////////////////////////////////////////////////////////////
// Class: OpenGlGuard
// Description: makes sure imextk opengl methods are called in the context of the
//  given opengl context, and that after the call, the opengl context is restored
//  to its former state.
//////////////////////////////////////////////////////////////////////////////////
class OpenGlGuard {
    public:
        OpenGlGuard(HDC dc, HGLRC glc)
            : _oldDC(wglGetCurrentDC()),
              _oldGLRC(wglGetCurrentContext())
        {
            wglMakeCurrent(dc, glc);
        }

        virtual ~OpenGlGuard() {
            wglMakeCurrent(_oldDC, _oldGLRC);
        }

    private:
        HDC   _oldDC;
        HGLRC _oldGLRC;
};


#endif // __PELCO_IMEXTK_WGL_HPP__
