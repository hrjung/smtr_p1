//###########################################################################
//
// FILE:   common_tools.h
//
// TITLE:
//
//###########################################################################

#ifndef __COMMON_TOOLS_H__
#define __COMMON_TOOLS_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_obj.h"
#include "hal.h"


//*****************************************************************************





//*****************************************************************************




//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void UTIL_setShaftBrake(void);
extern void UTIL_releaseShaftBrake(void);

extern void UTIL_setFanOn(void);
extern void UTIL_setFanOff(void);

extern int UTIL_controlLed(int type, int on_off);
extern void UTIL_testbit(int on_off);
extern void UTIL_testbitG(int on_off);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __COMMON_TOOLS_H__


