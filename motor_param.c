//###########################################################################
//
// FILE:   motor_param.c
//
// TITLE:
//
//###########################################################################

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <Assert.h>

#include "main.h"
#include "inv_param.h"
#include "motor_param.h"


//*****************************************************************************
//
//! \addtogroup
//! @{
//
//*****************************************************************************
const motor_param_st TEST_0_25k = {
	2,		//pole_pairs;
	60, 	//rated_freq;
	220, 	//voltage_in;

	1.15, 	//noload_current;
	3.0, 	//max_current;
	10.1598806, 	//Rs;
	5.574939, //Rr;
	0.00392938871, //Ls;

	1.626346, 	//magnetize_current;
	2.953668, 	//rated_flux;
};

const motor_param_st SY_1_5k = {
	2,		//pole_pairs;
	60, 	//rated_freq;
	380, 	//voltage_in;

	2.0, 	//noload_current;
	3.4, 	//max_current;
	2.5, 	//Rs;
	2.14568, //Rr;
	0.013955, //Ls;

	2.828427, 	//magnetize_current;
	4.923143, 	//rated_flux;
};

const motor_param_st SY_1_5k_ie3 = {
	2,		//pole_pairs;
	60, 	//rated_freq;
	380, 	//voltage_in;

	2.0, 	//noload_current;
	3.4, 	//max_current;
	2.5, 	//Rs;
	2.14568, //Rr;
	0.013955, //Ls;

	2.828427, 	//magnetize_current;
	4.923143, 	//rated_flux;
};

const motor_param_st SY_2_2k = {
	2,		//pole_pairs;
	60, 	//rated_freq;
	380, 	//voltage_in;

	3.235, 	//noload_current;
	5.3, 	//max_current;
	2.86, 	//Rs;
	1.14793, //Rr;
	0.01092, //Ls;

	2.828427, 	//magnetize_current;
	4.923143, 	//rated_flux;
};

motor_param_st mtr_param;
//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

void MPARAM_init(uint16_t type)
{
	switch(type)
	{
	case MOTOR_TEST_0_25k_TYPE: mtr_param = TEST_0_25k; break;
	case MOTOR_SY_1_5K_TYPE: mtr_param = SY_1_5k; break;
	case MOTOR_SY_2_2K_TYPE: mtr_param = SY_2_2k; break;

	case MOTOR_SY_1_5K_IE3_TYPE:
	default:
		mtr_param = SY_1_5k_ie3;
		break;
	}
}


void MPARAM_setDciPwmRate(float_t rate)
{
	dev_const.dci_pwm_rate = rate/100.0 * mtr_param.max_current*mtr_param.Rs;
}

void MPARAM_setOvlTripLevel(uint32_t level)
{
	dev_const.trip_level = mtr_param.max_current*(float_t)level/100.0;
}

void MPARAM_setOvlWarnLevel(uint32_t level)
{
	dev_const.warn_level = mtr_param.max_current*(float_t)level/100.0;
}


float_t FREQ_convertToSpeed(float_t freq)
{
    float_t spd_rpm = (freq*60.0 / mtr_param.pole_pairs);

    return spd_rpm;
}
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


