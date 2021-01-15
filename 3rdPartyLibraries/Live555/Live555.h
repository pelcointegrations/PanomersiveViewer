// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the LIVE555_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// LIVE555_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef LIVE555_EXPORTS
#define LIVE555_API __declspec(dllexport)
#else
#define LIVE555_API __declspec(dllimport)
#endif

// This class is exported from the Live555.dll
class LIVE555_API CLive555 {
public:
	CLive555(void);
	// TODO: add your methods here.
};

extern LIVE555_API int nLive555;

LIVE555_API int fnLive555(void);
