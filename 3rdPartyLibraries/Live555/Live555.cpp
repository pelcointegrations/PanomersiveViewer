// Live555.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "Live555.h"


// This is an example of an exported variable
LIVE555_API int nLive555=0;

// This is an example of an exported function.
LIVE555_API int fnLive555(void)
{
	return 42;
}

// This is the constructor of a class that has been exported.
// see Live555.h for the class definition
CLive555::CLive555()
{
	return;
}
