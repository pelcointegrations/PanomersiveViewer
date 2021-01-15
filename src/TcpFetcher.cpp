//============================================================================
// Copyright (c) 2014-2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#include "stdafx.h"
#include "TcpFetcher.hpp"
#include <sstream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <stdexcept>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#endif

using namespace std;

//////////////////////////////////////////////////////////////////////////////////
string tcpFetch(const string& ipAddress, uint16_t port, const string& query)
{
#ifdef _WIN32
    #define GET_ERR_STRING() WSAGetLastError()
#else
    typedef int SOCKET;
    const int INVALID_SOCKET = -1;
    const int SOCKET_ERROR = -1;
    typedef sockaddr_in SOCKADDR_IN;
    typedef sockaddr SOCKADDR;
    const int SD_SEND = SHUT_RD;
    #define closesocket(sock) close(sock)
    #define GET_ERR_STRING() strerror(errno)
#endif

    SOCKET sock = INVALID_SOCKET;
    bool connected = false;
    string retval;

    try {
        // Create our socket
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET) {
            throw runtime_error("Socket creation Failed!");
        }

        // Setup our socket address structure
        SOCKADDR_IN sockAddr;
        sockAddr.sin_port = htons(port);
        sockAddr.sin_family = AF_INET;

        // Resolve IP address for hostname
        int ret = 0;
#ifdef _WIN32
        PADDRINFOA pAddrInfo;
        ret = getaddrinfo(ipAddress.c_str(), NULL, NULL, &pAddrInfo);
        if (ret != 0) {
            throw runtime_error("Error getting address info");
        }
        sockAddr.sin_addr = ((struct sockaddr_in*)pAddrInfo->ai_addr)->sin_addr;
#else
        // Resolve IP address for hostname
        inet_aton(ipAddress.c_str(), &sockAddr.sin_addr);
#endif

        // Attempt to connect to server
        if (connect(sock, (SOCKADDR*)(&sockAddr), sizeof(sockAddr)) !=0 ) {
            throw runtime_error("Failed to establish connection with server");
        }
        connected = true;

        // Send query
        ret = send(sock, query.c_str(), static_cast<int>(query.size()), 0);
        if (ret == SOCKET_ERROR) {
            throw runtime_error("send failed");
        }

        // Read response
        const size_t bufLength = 4098; // should be plenty for DESCRIBE response
        char buf[bufLength];
        ret = recv(sock, buf, bufLength - 1, 0);
        if (ret == SOCKET_ERROR) {
            throw runtime_error("recv failure");
        }
        retval = string(buf, ret);
    } catch (const std::exception& e) {
        if (connected) {
            shutdown(sock, SD_SEND);
        }

        if (sock != INVALID_SOCKET) {
            closesocket(sock);
        }
        stringstream msg;
        msg << e.what() << ": " << GET_ERR_STRING();
        throw runtime_error(msg.str());
    }

    shutdown(sock, SD_SEND);
    closesocket(sock);

    return retval;
}

