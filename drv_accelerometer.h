//###########################################################################
//
// FILE:   drv_accelerometer.h
//
// TITLE:  LIS2DE12 register definition.
//
//###########################################################################

#ifndef __DRV_ACCELEROMETER_H__
#define __DRV_ACCELEROMETER_H__

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




//*****************************************************************************
//	register address definition
//
//*****************************************************************************

#define 	ACCEL_WHOAMI		(0x0F)
#define 	ACCEL_CTRL_REG1		(0x20)
#define 	ACCEL_CTRL_REG4		(0x23)
#define 	ACCEL_CTRL_REG5		(0x24)
#define 	ACCEL_STATUS_REG	(0x27)
#define 	ACCEL_OUT_X			(0x29)
#define 	ACCEL_OUT_Y			(0x2B)
#define 	ACCEL_OUT_Z			(0x2D)
//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************



//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __DRV_ACCELEROMETER_H__


