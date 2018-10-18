//###########################################################################
//
// FILE:   parameters.c
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
#include "parameters.h"
#include "common_tools.h"
#include "uartstdio.h"

#include "freq.h"
#include "drive.h"
#include "state_func.h"

//*****************************************************************************
//
//! \addtogroup
//! @{
//
//*****************************************************************************

//typedef struct
//{
//
//};


inv_parameter_st iparam[INV_PARAM_INDEX_MAX];
inv_parameter_st err_info[ERR_CODE_MAX];
inv_parameter_st inv_status[INV_STATUS_MAX];

extern MOTOR_working_st m_status;
extern motor_param_st mtr_param;
extern const char *res_str[2];

extern void STA_printInvState(void);
//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

void PARAM_init(void)
{
	iparam[FREQ_VALUE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[FREQ_VALUE_INDEX].value.f = 10.0;

	iparam[ACCEL_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[ACCEL_TIME_INDEX].value.f = 10.0;

	iparam[DECEL_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[DECEL_TIME_INDEX].value.f = 10.0;

	iparam[DIRECTION_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[DIRECTION_INDEX].value.f = 1.0;

	iparam[VF_FOC_SEL_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[VF_FOC_SEL_INDEX].value.l = VF_CONTROL; //FOC_CONTROL

	iparam[ENERGY_SAVE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[ENERGY_SAVE_INDEX].value.l = ESAVE_UNUSED; //  ESAVE_BOTH;

	iparam[PWM_FREQ_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[PWM_FREQ_INDEX].value.l = PWM_4KHz;

	iparam[JUMP_ENABLE0_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE0_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW0_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW0_INDEX].value.f = 0.0;

	iparam[JUMP_HIGH0_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH0_INDEX].value.f = 0.0;

	iparam[JUMP_ENABLE1_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE1_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW1_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW1_INDEX].value.f = 0.0;

	iparam[JUMP_HIGH1_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH1_INDEX].value.f = 0.0;

	iparam[JUMP_ENABLE2_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[JUMP_ENABLE2_INDEX].value.l = NOT_USED;

	iparam[JUMP_LOW2_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_LOW2_INDEX].value.f = 0.0;

	iparam[JUMP_HIGH2_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[JUMP_HIGH2_INDEX].value.f = 0.0;

	iparam[V_BOOST_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[V_BOOST_INDEX].value.f = 0.0;

	iparam[FOC_TORQUE_LIMIT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[FOC_TORQUE_LIMIT_INDEX].value.f = 180.0;

	iparam[BRK_TYPE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[BRK_TYPE_INDEX].value.l = REDUCE_SPEED_BRAKE;

	iparam[BRK_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_FREQ_INDEX].value.f = 3.0;

	iparam[BRK_DCI_START_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_START_FREQ_INDEX].value.f = REDUCE_SPEED_BRAKE;

	iparam[BRK_DCI_BLOCK_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BLOCK_TIME_INDEX].value.f = 1.0;

	iparam[BRK_DCI_BRAKING_TIME_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BRAKING_TIME_INDEX].value.f = 5.0;

	iparam[BRK_DCI_BRAKING_RATE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[BRK_DCI_BRAKING_RATE_INDEX].value.f = 50.0;

	iparam[OVL_WARN_LIMIT_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_WARN_LIMIT_INDEX].value.l = 150;

	iparam[OVL_WR_DURATION_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_WR_DURATION_INDEX].value.l = 10;

	iparam[OVL_ENABLE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_ENABLE_INDEX].value.l = 1; //use

	iparam[OVL_TR_LIMIT_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_TR_LIMIT_INDEX].value.l = 180;

	iparam[OVL_TR_DURATION_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[OVL_TR_DURATION_INDEX].value.l = 30;

	iparam[REGEN_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[REGEN_RESISTANCE_INDEX].value.f = 200.0;

	iparam[REGEN_THERMAL_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[REGEN_THERMAL_INDEX].value.f = 5.0;

	iparam[REGEN_POWER_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[REGEN_POWER_INDEX].value.l = 400;

	iparam[REGEN_BAND_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[REGEN_BAND_INDEX].value.l = 0;

	iparam[FAN_COMMAND_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[FAN_COMMAND_INDEX].value.l = 0;

	iparam[STATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[STATOR_RESISTANCE_INDEX].value.f = mtr_param.Rs;

	iparam[ROTATOR_RESISTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[ROTATOR_RESISTANCE_INDEX].value.f = mtr_param.Rr;

	iparam[INDUCTANCE_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[INDUCTANCE_INDEX].value.f = mtr_param.Ls;

	iparam[NOLOAD_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[NOLOAD_CURRENT_INDEX].value.f = mtr_param.noload_current;

	iparam[RATED_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	iparam[RATED_CURRENT_INDEX].value.f = mtr_param.max_current;

	iparam[POLES_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[POLES_INDEX].value.l = mtr_param.pole_pairs;

	iparam[INPUT_VOLTAGE_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[INPUT_VOLTAGE_INDEX].value.l = mtr_param.voltage_in;

	iparam[RATED_FREQ_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[RATED_FREQ_INDEX].value.l = mtr_param.rated_freq;

	iparam[INV_RUN_STOP_INDEX].type = PARAMETER_TYPE_LONG;
	iparam[INV_RUN_STOP_INDEX].value.l = 0; // default stop

	// error parameter
	PARAM_initErrInfo();

	// inverter status
	PARAM_initInvStatus();

}


void PARAM_update(uint16_t index, uint16_t *buf)
{
	iparam[index].value.arr[0] = buf[0];
	iparam[index].value.arr[1] = buf[1];
}

uint16_t PARAM_getValue(uint16_t index, uint16_t *buf)
{
#if 0 // test only
	iparam[index].value.f = 3.14;
#endif
	buf[0] = iparam[index].value.arr[0];
	buf[1] = iparam[index].value.arr[1];

	return 2;
}

void PARAM_setFwdDirection(void)
{
	MAIN_setForwardDirection();
	STA_calcResolution4Reverse(m_status.cur_freq);
	UARTprintf("set direction forward\n");
}

void PARAM_startRun(void)
{
	if(!MAIN_isSystemEnabled())
	{
		MAIN_enableSystem();
		STA_calcResolution();
		UARTprintf("start running motor\n");

	    STA_printInvState();
	}
}

void PARAM_stopRun(void)
{
	if(MAIN_isSystemEnabled())
	{
		STA_setStopCondition();
		UARTprintf("STOP command\n");
	}
}

void PARAM_setRevDirection(void)
{
	MAIN_setReverseDirection();
	STA_calcResolution4Reverse(m_status.cur_freq);
	UARTprintf("set direction backward\n");
}

void PARAM_process(uint16_t index, union32_st data)
{
	uint32_t ldata;
	float_t fdata;
	int result;

	if(iparam[index].type == PARAMETER_TYPE_LONG)
		ldata = data.l;
	else
		fdata = data.f;

	switch(index)
	{
	case FREQ_VALUE_INDEX:
		result = FREQ_setFreqValue(fdata);
		UARTprintf("set frequency=%f, result=%s\n", fdata, res_str[result]);
		UARTprintf("resolution acc_res=%f, dec_res=%f\n", m_status.acc_res, m_status.dec_res);
		break;

	case ACCEL_TIME_INDEX:
		result = DRV_setAccelTime(fdata);
		UARTprintf("set accel time=%f, result=%s\n", fdata, res_str[result]);
		break;

	case DECEL_TIME_INDEX:
		result = DRV_setDecelTime(fdata);
		UARTprintf("set decel time=%f, result=%s\n", fdata, res_str[result]);
		break;

	case VF_FOC_SEL_INDEX:
		if(ldata == 0)
		{
			DRV_enableVfControl();
			UARTprintf("set VF control\n");
		}
		else
		{
			DRV_enableFocControl();
			UARTprintf("set FOC control\n");
		}
		break;
	}
}

void PARAM_initErrInfo(void)
{
	err_info[ERR_CODE_INDEX].type = PARAMETER_TYPE_LONG;
	err_info[ERR_CODE_INDEX].value.l = 0;

	err_info[ERR_CURRENT_INDEX].type = PARAMETER_TYPE_FLOAT;
	err_info[ERR_CURRENT_INDEX].value.f = 0.0;

	err_info[ERR_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	err_info[ERR_FREQ_INDEX].value.f = 0.0;
}

void PARAM_setErrInfo(uint16_t err_code, uint16_t err_status, float_t current, float_t freq)
{
	err_info[ERR_CODE_INDEX].value.arr[0] = err_code;
	err_info[ERR_CODE_INDEX].value.arr[1] = err_status;
	err_info[ERR_CURRENT_INDEX].value.f = current;
	err_info[ERR_FREQ_INDEX].value.f = freq;
}

uint16_t PARAM_getErrorInfo(uint16_t *buf)
{
#if 1 // test only
	err_info[ERR_CODE_INDEX].value.arr[0] = 0;
	err_info[ERR_CODE_INDEX].value.arr[1] = 1;
	err_info[ERR_CURRENT_INDEX].value.f = 10.5;
	err_info[ERR_FREQ_INDEX].value.f = 120.0;
#endif
	buf[0] = err_info[ERR_CODE_INDEX].value.arr[0];
	buf[1] = err_info[ERR_CODE_INDEX].value.arr[1];
	buf[2] = err_info[ERR_CURRENT_INDEX].value.arr[0];
	buf[3] = err_info[ERR_CURRENT_INDEX].value.arr[1];
	buf[4] = err_info[ERR_FREQ_INDEX].value.arr[0];
	buf[5] = err_info[ERR_FREQ_INDEX].value.arr[1];

	return (ERR_CODE_MAX*2);
}

void PARAM_initInvStatus(void)
{
	inv_status[INV_STATUS_INDEX].type = PARAMETER_TYPE_LONG;
	inv_status[INV_STATUS_INDEX].value.l = 0;

	inv_status[INV_I_RMS_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_I_RMS_INDEX].value.f = 0.0;

	inv_status[INV_RUN_FREQ_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_RUN_FREQ_INDEX].value.f = 0.0;

	inv_status[INV_DC_VOLTAGE_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = 0.0;

	inv_status[INV_IPM_TEMP_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_IPM_TEMP_INDEX].value.f = 0.0;

	inv_status[INV_MOTOR_TEMP_INDEX].type = PARAMETER_TYPE_FLOAT;
	inv_status[INV_MOTOR_TEMP_INDEX].value.f = 0.0;
}

void PARAM_setInvStatus(uint16_t run, uint16_t dir, float_t icurr, float_t freq, float_t vdc, float_t ipm_t, float_t mtr_t)
{
	inv_status[INV_STATUS_INDEX].value.arr[0] = run;
	inv_status[INV_STATUS_INDEX].value.arr[1] = dir;

	inv_status[INV_I_RMS_INDEX].value.f = icurr;
	inv_status[INV_RUN_FREQ_INDEX].value.f = freq;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = vdc;
	inv_status[INV_IPM_TEMP_INDEX].value.f = ipm_t;
	inv_status[INV_MOTOR_TEMP_INDEX].value.f = mtr_t;
}

uint16_t PARAM_getInvStatus(uint16_t *buf)
{
#if 1 // test only
	inv_status[INV_STATUS_INDEX].value.arr[0] = 0;
	inv_status[INV_STATUS_INDEX].value.arr[1] = 1;
	inv_status[INV_I_RMS_INDEX].value.f = 2.1;
	inv_status[INV_RUN_FREQ_INDEX].value.f = 30.2;
	inv_status[INV_DC_VOLTAGE_INDEX].value.f = 531.3;
	inv_status[INV_IPM_TEMP_INDEX].value.f = 63.4;
	inv_status[INV_MOTOR_TEMP_INDEX].value.f = 54.1;
#endif
	buf[0] = inv_status[INV_STATUS_INDEX].value.arr[0];
	buf[1] = inv_status[INV_STATUS_INDEX].value.arr[1];
	buf[2] = inv_status[INV_I_RMS_INDEX].value.arr[0];
	buf[3] = inv_status[INV_I_RMS_INDEX].value.arr[1];
	buf[4] = inv_status[INV_RUN_FREQ_INDEX].value.arr[0];
	buf[5] = inv_status[INV_RUN_FREQ_INDEX].value.arr[1];
	buf[6] = inv_status[INV_DC_VOLTAGE_INDEX].value.arr[0];
	buf[7] = inv_status[INV_DC_VOLTAGE_INDEX].value.arr[1];
	buf[8] = inv_status[INV_IPM_TEMP_INDEX].value.arr[0];
	buf[9] = inv_status[INV_IPM_TEMP_INDEX].value.arr[1];
	buf[10] = inv_status[INV_MOTOR_TEMP_INDEX].value.arr[0];
	buf[11] = inv_status[INV_MOTOR_TEMP_INDEX].value.arr[1];

	return (INV_STATUS_MAX*2);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


