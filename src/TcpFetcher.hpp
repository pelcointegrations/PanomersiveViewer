//============================================================================
// Copyright (c) 2014 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_TCP_FETCHER_H__
#define __PELCO_TCP_FETCHER_H__

#include <string>
#include <stdint.h>

// Simple TCP fetcher: opens TCP connection to (ipAddress, port),
// sends the given query, reads response to EOF, and returns response
// as a string.
extern std::string tcpFetch(const std::string& ipAddress, uint16_t port,
        const std::string& query);

#endif // __PELCO_TCP_H__
