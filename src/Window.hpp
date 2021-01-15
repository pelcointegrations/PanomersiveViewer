//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_WINDOW_HPP__
#define __PELCO_IMEXTK_WINDOW_HPP__

#include <string>
#include <map>

class WindowArea {
    public:
        virtual void show() = 0;
        virtual void hide() = 0;

        virtual void move(const RECT& r) = 0;

        // rect is relative to parent window's client area.
        virtual RECT rect() const = 0;

        // client area of this window
        virtual RECT clientRect() const = 0;
        virtual size_t clientWidth() const = 0;
        virtual size_t clientHeight() const = 0;
};

class Window : public WindowArea {
    public:
        Window(const WNDCLASSEX& wcex,
                HWND hParentWnd, DWORD windowStyle,
                const RECT& area, const std::wstring& title,
                HMENU hMenu);

        virtual ~Window();

        virtual void show();
        virtual void hide();

        virtual void move(const RECT& r);
        virtual RECT rect() const;
        virtual RECT clientRect() const;
        virtual size_t clientWidth() const;
        virtual size_t clientHeight() const;

    protected:
        static WNDCLASSEX defaultWindowClassInfo(HINSTANCE hInstance);

        // enforces one-message-at-time behavior, preventing a recursive chain
        // of dialogs, should errors keep hapenning while this is shown
        void showErrorPopup(HWND parent, const std::wstring& title,
                const std::wstring& message);

        // called by showErrorPopup prior to showing modal error dialog.
        // Allows sub-classes to clean up first or even abort.
        virtual void aboutToShowErrorPopup() { }

        // returns true if the command was handled, false otherwise.
        virtual bool OnMenuCommand(int menuID) {
            return false;
        }

        // user wants to close: returns true if it's ok to close
        virtual bool OnCloseRequest();

        // overrides must call base class impl.
        virtual void OnDestroy();

        virtual void OnPaint(const PAINTSTRUCT& ps) {
        }

        // returns true if the background was erased (or cleared to
        // whatever values the derived class wants)
        virtual bool EraseBackground() {
            return false;
        }

        virtual void OnResize(int width, int height) {
        }

        // Left-button mouse events
        virtual void OnLMouseDown(const POINTS& pt) {
        }
        virtual void OnLMouseUp(const POINTS& pt) {
        }
        virtual void OnLMouseDblClk(const POINTS& pt) {
        }

        // Middle-button mouse events
        virtual void OnMMouseDown(const POINTS& pt) {
        }
        virtual void OnMMouseUp(const POINTS& pt) {
        }
        virtual void OnMMouseDblClk(const POINTS& pt) {
        }

        // Right-button mouse events
        virtual void OnRMouseDown(const POINTS& pt) {
        }
        virtual void OnRMouseUp(const POINTS& pt) {
        }
        virtual void OnRMouseDblClk(const POINTS& pt) {
        }

        virtual void OnMouseMove(const POINTS& pt) {
        }

        virtual void OnMouseWheel(short wheelValue, const POINTS& pt) {
        }
        virtual void OnTimer(UINT timerID) {
        }

        virtual void OnKeyDown(WPARAM virtualKeyCode, LPARAM repeatFlags) {
        }
        virtual void OnKeyUp(WPARAM virtualKeyCode, LPARAM repeatFlags) {
        }

        void setHourglass(bool show);

        virtual LRESULT OnUserMessage(UINT msgID,
                WPARAM wparam, LPARAM lparam) {
            return 0;
        }

        void PostRedraw(bool clear = false);

        const std::wstring _className;
        const HINSTANCE _hInstance;
        HWND            _hWnd;
        HMENU           _hMenu;
        HDC             _hDC;
        const DWORD     _windowThreadId;

        static const std::wstring _defaultWindowClassName;
    private:
        void cleanup();
        static LRESULT CALLBACK wndProc(HWND, UINT, WPARAM, LPARAM);

        static std::map<std::wstring, int> _instanceCount;

        bool _showHourglass;
        bool _showingErrorPopup;
};

#endif // __PELCO_IMEXTK_WINDOW_HPP__
