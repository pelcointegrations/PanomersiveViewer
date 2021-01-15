//============================================================================
// Copyright (c) 2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "SplitterLayout.hpp"
#include <cassert>
#include <iostream>

using namespace std;

namespace {
const int kSplitterWidth = 6;
}

//////////////////////////////////////////////////////////////////////////////////
SplitterLayout::Splitter::Splitter(bool horizontal,
        HINSTANCE hInst, HWND parent, RECT r,
        SplitterLayout* pLayout)
    : Window(splitterClassInfo(hInst, horizontal),
            parent, WS_CHILD, r, wstring(), NULL),
      _pLayout(pLayout),
      _horizontal(horizontal)
{
}

//////////////////////////////////////////////////////////////////////////////////
WNDCLASSEX SplitterLayout::Splitter::splitterClassInfo(HINSTANCE hInstance,
    bool horizontal)
{
    WNDCLASSEX wcex = defaultWindowClassInfo(hInstance);
    wcex.hCursor = LoadCursor(NULL, horizontal ? IDC_SIZEWE : IDC_SIZENS);
    wcex.lpszClassName = horizontal ? L"PelcoImextkHSplitter" : L"PelcoImextkVSplitter";

    return wcex;
}

//////////////////////////////////////////////////////////////////////////////////
void SplitterLayout::Splitter::OnLMouseDown(const POINTS& pt)
{
    // if we're not already capturing the mouse for some other operation...
    if (GetCapture() != _hWnd) {
        _mouseMoveAnchor = pt;
        SetCapture(_hWnd);
    }
}

//////////////////////////////////////////////////////////////////////////////////
void SplitterLayout::Splitter::OnMouseMove(const POINTS& spt)
{
    if (GetCapture() == _hWnd) {
        RECT r = clientRect();
        double layoutPixels = 0.0;
        double deltaPixels = 0.0;

        if (_horizontal) {
            layoutPixels = double(_pLayout->clientWidth());
            deltaPixels = (double(spt.x) - (double(clientWidth()) / 2.0));
        }
        else {
            layoutPixels = double(_pLayout->clientHeight());
            deltaPixels = (double(spt.y) - (double(clientHeight()) / 2.0));
        }

        if (layoutPixels > 0) {
            double newSplit = _pLayout->getSplit() + (deltaPixels / layoutPixels);
            newSplit = max(0.0, min(1.0, newSplit));
            _pLayout->setSplit(newSplit);
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////
void SplitterLayout::Splitter::OnLMouseUp(const POINTS& pt)
{
    if (GetCapture() == _hWnd) {
        ReleaseCapture();
    }
}

//////////////////////////////////////////////////////////////////////////////////
SplitterLayout::SplitterLayout(Type st, HINSTANCE hInstance, HWND hParentWnd,
        const RECT& area, WindowArea* topOrLeft, WindowArea* bottomOrRight)
    : _horizontal(st == HORIZONTAL),
      _area(area),
      _splitterPos(0.5),
      _splitter(_horizontal, hInstance, hParentWnd, splitterRect(), this)
{
    assert(topOrLeft != NULL);
    _children[0] = topOrLeft;
    topOrLeft->move(topOrLeftRect());

    assert(bottomOrRight != NULL);
    _children[1] = bottomOrRight;
    bottomOrRight->move(bottomOrRightRect());
}

//////////////////////////////////////////////////////////////////////////////////
SplitterLayout::~SplitterLayout()
{
}

//////////////////////////////////////////////////////////////////////////////////
RECT SplitterLayout::rect() const
{
    return _area;
}

//////////////////////////////////////////////////////////////////////////////////
RECT SplitterLayout::clientRect() const
{
    return {0, 0, long(clientWidth()), long(clientHeight())};
}

//////////////////////////////////////////////////////////////////////////////////
void SplitterLayout::setSplit(double splitVal)
{
    assert((splitVal >= 0.0) && (splitVal <= 1.0));
    _splitterPos = splitVal;
    move(_area); // force re-layout
}

//////////////////////////////////////////////////////////////////////////////////
RECT SplitterLayout::splitterRect() const
{
    RECT retval = _area;
    if (_horizontal) {
        int center = int(clientWidth() * _splitterPos + 0.5);
        retval.left = center - (kSplitterWidth / 2);
        retval.right = retval.left + kSplitterWidth;
    }
    else {
        int center = int(clientHeight() * _splitterPos + 0.5);
        retval.top = center - (kSplitterWidth / 2);
        retval.bottom = retval.top + kSplitterWidth;
    }

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
RECT SplitterLayout::topOrLeftRect() const
{
    RECT retval = _area;
    if (_horizontal) {
        int center = int(clientWidth() * _splitterPos + 0.5);
        retval.right = center - (kSplitterWidth / 2);
    }
    else {
        int center = int(clientHeight() * _splitterPos + 0.5);
        retval.bottom = center - (kSplitterWidth / 2);
    }

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
RECT SplitterLayout::bottomOrRightRect() const
{
    RECT retval = _area;
    if (_horizontal) {
        int center = int(clientWidth() * _splitterPos + 0.5);
        retval.left = center - (kSplitterWidth / 2) + kSplitterWidth;
    }
    else {
        int center = int(clientHeight() * _splitterPos + 0.5);
        retval.top = center - (kSplitterWidth / 2) + kSplitterWidth;
    }

    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
void SplitterLayout::move(const RECT& r) {
    _area = r;

    _splitter.move(splitterRect());
    _children[0]->move(topOrLeftRect());
    _children[1]->move(bottomOrRightRect());
}

//////////////////////////////////////////////////////////////////////////////////
size_t SplitterLayout::clientWidth() const
{
    return _area.right - _area.left;
}

//////////////////////////////////////////////////////////////////////////////////
size_t SplitterLayout::clientHeight() const
{
    return _area.bottom - _area.top;
}

//////////////////////////////////////////////////////////////////////////////////
void SplitterLayout::show()
{
    _splitter.show();
    _children[0]->show();
    _children[1]->show();
}

//////////////////////////////////////////////////////////////////////////////////
void SplitterLayout::hide()
{
    _splitter.hide();
    _children[0]->hide();
    _children[1]->hide();
}
