//============================================================================
// Copyright (c) 2016 Pelco.  All rights reserved.
//
// This file contains trade secrets of Pelco.  No part may be reproduced or
// transmitted in any form by any means or for any purpose without the express
// written permission of Pelco.
//============================================================================
#ifndef __PELCO_IMEXTK_ERR_HANDLING_HPP__
#define __PELCO_IMEXTK_ERR_HANDLING_HPP__

#include <pelco/imextk/Context.h>
#include <string>

extern void throwImextkError(const std::string& prefix, PelcoImextk_Result res,
    PelcoImextk_Context ctx);

// Note: this macro relies on the existence of the variable _imextkContext, which
// is not the best form.  TODO: It's probably cleaner to add the context parameter
// to the macro call, but that would require changing every existing call to this macro.
#define IMEXTK_CHECK(funcall) {                                 \
    PelcoImextk_Result res = (funcall);                         \
    if (res != PELCO_IMEXTK_NO_ERROR) {                         \
        throwImextkError("Error on call "#funcall, res, _imextkContext);     \
    }                                                           \
}

#define IMEXTK_CHECK_CTX(funcall, ctx) {                        \
    PelcoImextk_Result res = (funcall);                         \
    if (res != PELCO_IMEXTK_NO_ERROR) {                         \
        throwImextkError("Error on call "#funcall, res, ctx);   \
    }                                                           \
}

#endif // __PELCO_IMEXTK_ERR_HANDLING_HPP__
