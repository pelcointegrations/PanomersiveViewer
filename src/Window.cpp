//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "Window.hpp"
#include "Resource.h"
#include <cassert>
#include <sstream>

using namespace std;

map<wstring, int> Window::_instanceCount;

//////////////////////////////////////////////////////////////////////////////////
Window::Window(const WNDCLASSEX& wcex,
        HWND hParentWnd, DWORD windowStyle,
        const RECT& area, const wstring& title,
        HMENU hMenu)
    : _className(wcex.lpszClassName),
      _hInstance(wcex.hInstance),
      _hWnd(NULL),
      _hMenu(hMenu),
      _hDC(NULL),
      _windowThreadId(GetCurrentThreadId()),
      _showHourglass(false),
      _showingErrorPopup(false)
{
    ++_instanceCount[_className];
    if (_instanceCount[_className] == 1) {
        RegisterClassEx(&wcex);
    }

    int x = area.left;
    int y = area.top;
    int w = area.right == CW_USEDEFAULT ? CW_USEDEFAULT : (area.right - area.left);
    int h = area.bottom - area.top;

    _hWnd = CreateWindow(_className.c_str(), title.c_str(), windowStyle,
            x, y, w, h,
            hParentWnd, hMenu, _hInstance, NULL);
    if (_hWnd == NULL) {
        cleanup();
        throw runtime_error("Error creating window");
    }
    _hDC = GetDC(_hWnd);

    SetWindowLongPtr(_hWnd, GWLP_USERDATA, (LONG_PTR)this);
}

//////////////////////////////////////////////////////////////////////////////////
Window::~Window()
{
    if (_hWnd != NULL) {
        SetWindowLongPtr(_hWnd, GWLP_USERDATA, (LONG_PTR)NULL);
        DestroyWindow(_hWnd);
    }
    cleanup();
}

//////////////////////////////////////////////////////////////////////////////////
void Window::cleanup()
{
    --_instanceCount[_className];
    if (_instanceCount[_className] == 0) {
        UnregisterClass(_className.c_str(), NULL);
    }
}

//////////////////////////////////////////////////////////////////////////////////
WNDCLASSEX Window::defaultWindowClassInfo(HINSTANCE hInstance)
{
    WNDCLASSEX wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style              = CS_OWNDC | CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS;
    wcex.lpfnWndProc        = Window::wndProc;
    wcex.cbClsExtra         = 0;
    wcex.cbWndExtra         = 0;
    wcex.hInstance          = hInstance;
    wcex.hIcon              = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_DEMOAPP));
    wcex.hCursor            = LoadCursor(NULL, IDC_ARROW);
    wcex.hbrBackground      = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName       = NULL;
    wcex.lpszClassName      = L"PelcoImextkDemoAppWindow";
    wcex.hIconSm            = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return wcex;
}

//////////////////////////////////////////////////////////////////////////////////
LRESULT CALLBACK Window::wndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    int wmId, wmEvent;
    PAINTSTRUCT ps;
    HDC hdc;

    Window* pWindow = (Window*)GetWindowLongPtr(hWnd, GWLP_USERDATA);

    if (pWindow == NULL) {
        // we may get wndProc calls before setWindowLongPtr is called in the constructor
        // or after it's called in the destructor.
        return DefWindowProc(hWnd, message, wParam, lParam);
    }

    LRESULT retval = 0;

    try {
        switch (message) {
            case WM_CREATE:
                // WM_CREATE may be called before CreateWindow returns, so no pWindow here
                break;

            case WM_COMMAND:
                wmId = LOWORD(wParam);
                wmEvent = HIWORD(wParam);
                pWindow->OnMenuCommand(wmId);
                break;

            case WM_PAINT:
                hdc = BeginPaint(hWnd, &ps);
                pWindow->OnPaint(ps);
                EndPaint(hWnd, &ps);
                break;

            case WM_ERASEBKGND:
                retval = pWindow->EraseBackground();
                if (!retval) {
                    retval = DefWindowProc(hWnd, message, wParam, lParam);
                }
                break;

            case WM_LBUTTONDOWN:
                SetFocus(hWnd);
                pWindow->OnLMouseDown(MAKEPOINTS(lParam));
                break;

            case WM_LBUTTONUP:
                pWindow->OnLMouseUp(MAKEPOINTS(lParam));
                break;

            case WM_LBUTTONDBLCLK:
                pWindow->OnLMouseDblClk(MAKEPOINTS(lParam));
                break;

            case WM_MBUTTONDOWN:
                SetFocus(hWnd);
                pWindow->OnMMouseDown(MAKEPOINTS(lParam));
                break;

            case WM_MBUTTONUP:
                pWindow->OnMMouseUp(MAKEPOINTS(lParam));
                break;

            case WM_MBUTTONDBLCLK:
                SetFocus(hWnd);
                pWindow->OnMMouseDblClk(MAKEPOINTS(lParam));
                break;

            case WM_RBUTTONDOWN:
                SetFocus(hWnd);
                pWindow->OnRMouseDown(MAKEPOINTS(lParam));
                break;

            case WM_RBUTTONUP:
                pWindow->OnRMouseUp(MAKEPOINTS(lParam));
                break;

            case WM_RBUTTONDBLCLK:
                pWindow->OnRMouseDblClk(MAKEPOINTS(lParam));
                break;

            case WM_MOUSEMOVE:
                pWindow->OnMouseMove(MAKEPOINTS(lParam));
                break;

            case WM_MOUSEWHEEL:
                SetFocus(hWnd);
                pWindow->OnMouseWheel(short(HIWORD(wParam)),
                        MAKEPOINTS(lParam));
                break;

            case WM_SETCURSOR:
                if (pWindow->_showHourglass && (LOWORD(lParam) == HTCLIENT)) {
                    SetCursor(LoadCursor(NULL, IDC_WAIT));
                    retval = true;
                }
                else {
                    retval = DefWindowProc(hWnd, message, wParam, lParam);
                }
                break;

            case WM_SIZE:
                // WM_SIZE may be called before CreateWindow returns
                pWindow->OnResize((int)LOWORD(lParam), (int)HIWORD(lParam));
                break;

            case WM_CLOSE:
                if (pWindow->OnCloseRequest()) {
                    DestroyWindow(hWnd);
                    retval = true;
                }
                break;

            case WM_DESTROY:
                // WM_DESTROY may get get called via this class' destructor,
                // in which case user-data LONG gets set to NULL
                pWindow->OnDestroy();
                break;

            case WM_TIMER:
                pWindow->OnTimer(static_cast<UINT>(wParam));
                break;

            case WM_KEYDOWN:
                pWindow->OnKeyDown(wParam, lParam);
                break;
            case WM_KEYUP:
                pWindow->OnKeyUp(wParam, lParam);
                break;

            default:
                if ((message >= WM_USER) && (message < WM_APP)) {
                    retval = pWindow->OnUserMessage(message, wParam, lParam);
                }
                else {
                    retval = DefWindowProc(hWnd, message, wParam, lParam);
                }
        }
    }
    catch (const std::exception& e) {
        wstringstream msg;
        msg << "Critical Error: " << e.what();
        pWindow->showErrorPopup(NULL, L"Critical Error", msg.str());
        PostQuitMessage(0);
    }
    return retval;
}

//////////////////////////////////////////////////////////////////////////////////
void Window::showErrorPopup(HWND parent, const wstring& title,
        const wstring& message)
{
    if (!_showingErrorPopup) {
        _showingErrorPopup = true;
        aboutToShowErrorPopup();
        MessageBox(parent, message.c_str(), title.c_str(), MB_OK);
        _showingErrorPopup = false;
    }
}


//////////////////////////////////////////////////////////////////////////////////
void Window::show()
{
    ShowWindow(_hWnd, SW_SHOW);
}

//////////////////////////////////////////////////////////////////////////////////
void Window::hide()
{
    ShowWindow(_hWnd, SW_HIDE);
}

//////////////////////////////////////////////////////////////////////////////////
void Window::move(const RECT& r)
{
    MoveWindow(_hWnd, r.left, r.top,
            r.right - r.left, r.bottom - r.top, true);
}

//////////////////////////////////////////////////////////////////////////////////
RECT Window::rect() const
{
    RECT r;
    GetWindowRect(_hWnd, &r);

    // map from screen coordinates to an area within this window's parent
    POINT upperLeft = { r.left, r.top };
    POINT lowerRight = { r.right, r.bottom };
    HWND hParent = GetParent(_hWnd);
    ScreenToClient(hParent, &upperLeft);
    ScreenToClient(hParent, &lowerRight);
    r.left = upperLeft.x;
    r.top = upperLeft.y;
    r.right = lowerRight.x;
    r.bottom = lowerRight.y;

    return r;
}

//////////////////////////////////////////////////////////////////////////////////
RECT Window::clientRect() const
{
    RECT r;
    GetClientRect(_hWnd, &r);
    return r;
}

//////////////////////////////////////////////////////////////////////////////////
size_t Window::clientWidth() const
{
    RECT r;
    GetClientRect(_hWnd, &r);
    return r.right - r.left;
}

//////////////////////////////////////////////////////////////////////////////////
size_t Window::clientHeight() const
{
    RECT r;
    GetClientRect(_hWnd, &r);
    return r.bottom - r.top;
}

//////////////////////////////////////////////////////////////////////////////////
bool Window::OnCloseRequest()
{
    return true;
}

//////////////////////////////////////////////////////////////////////////////////
void Window::OnDestroy()
{
    _hWnd = NULL;
}

//////////////////////////////////////////////////////////////////////////////////
void Window::PostRedraw(bool clear)
{
    InvalidateRect(_hWnd, NULL, clear);
}

//////////////////////////////////////////////////////////////////////////////////
void Window::setHourglass(bool show)
{
    _showHourglass = show;
    POINT pt;
    GetCursorPos(&pt);
    ScreenToClient(_hWnd, &pt);
    RECT cr = clientRect();
    if ((pt.x >= cr.left) && (pt.x < cr.right) &&
            (pt.y >= cr.top) && (pt.y < cr.bottom)) {
        SetCursor(LoadCursor(NULL, show ? IDC_WAIT : IDC_ARROW));
    }
}
