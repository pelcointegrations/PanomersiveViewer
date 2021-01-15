//============================================================================
// Copyright (c) 2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_SPLITTER_LAYOUT_HPP__
#define __PELCO_IMEXTK_SPLITTER_LAYOUT_HPP__

#include "Window.hpp"

class SplitterLayout : public WindowArea {
    public:
        enum Type {
            HORIZONTAL,
            VERTICAL
        };

        // two types: horizontal or vertical, indicating the layout of the child
        // areas.  Splitter bar is opposite (e.g., splitter bar is vertical
        // for horizontal layout).
        // topOrLeft and bottomOrRight must both be non-null.
        SplitterLayout(Type st, HINSTANCE hInstance, HWND hParentWnd,
                const RECT& area, WindowArea* topOrLeft, WindowArea* bottomOrRight);

        virtual ~SplitterLayout();

        virtual void show();
        virtual void hide();

        virtual void move(const RECT& r);
        virtual RECT rect() const;
        virtual RECT clientRect() const;
        virtual size_t clientWidth() const;
        virtual size_t clientHeight() const;

        const WindowArea* getTopOrLeft() const { return _children[0]; }
        WindowArea* getTopOrLeft() { return _children[0]; }
        const WindowArea* getBottomOrRight() const { return _children[1]; }
        WindowArea* getBottomOrRight() { return _children[1]; }

        // split position is the ratio of left/right widths or
        // or top/bottom heights.  0 means bottom or right is maximized,
        // 1 means top or left is maximized.
        double getSplit() const { return _splitterPos; }
        void setSplit(double splitVal /* in (0, 1) */);
    private:
        RECT splitterRect() const;
        RECT topOrLeftRect() const;
        RECT bottomOrRightRect() const;

        bool            _horizontal;
        RECT            _area;

        double          _splitterPos; // in (0, 1)
        WindowArea*     _children[2]; // top/left, bottom/right

        class Splitter : public Window {
            public:
                Splitter(bool isHorizontal, HINSTANCE hInst, HWND parent, RECT r,
                        SplitterLayout* pLayout);
            protected:
                virtual void OnLMouseDown(const POINTS& pt);
                virtual void OnMouseMove(const POINTS& pt);
                virtual void OnLMouseUp(const POINTS& pt);
            private:
                static WNDCLASSEX splitterClassInfo(HINSTANCE hInstance,
                        bool horizontal);
                SplitterLayout* _pLayout;
                bool _horizontal;
                POINTS _mouseMoveAnchor;
        };

        Splitter        _splitter;
};

#endif // __PELCO_IMEXTK_SPLITTER_LAYOUT_HPP__
