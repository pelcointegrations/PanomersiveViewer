//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================

#include "stdafx.h"
#include "Resource.h"
#include "PanomersiveViewer.hpp"
#include <string>
#include <cassert>

#include <iostream>

using namespace std;

//
// Entry point for the PanomersiveViewewr application
//
int APIENTRY _tWinMain(_In_ HINSTANCE hInstance,
        _In_opt_ HINSTANCE hPrevInstance,
        _In_ LPTSTR    lpCmdLine,
        _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);
    MSG msg;
    HACCEL hAccelTable;

    auto_ptr<PanomersiveViewer> demoApp;
    try {
        demoApp.reset(new PanomersiveViewer(hInstance));
    } catch (const std::exception& e) {
        MessageBox(NULL, wstring(e.what(), e.what() + strlen(e.what())).c_str(),
                L"Application initialization failure",
                MB_OK);
        exit(1);
    }
    demoApp->show();

    hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_DEMOAPP));

    // Main message loop:
    while (GetMessage(&msg, NULL, 0, 0)) {
        if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }

    return (int) msg.wParam;
}


