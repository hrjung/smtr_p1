/*
 * parameters.h
 *
 *  Created on: 2018. 7. 18.
 *      Author: hrjung
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

typedef union
{
  uint16_t arr[2];
  float f;
  uint32_t l;
} union32_st;


typedef struct
{
	uint16_t	type;
	union32_st	value;
} inv_parameter_st;

enum {
	ERR_CODE_INDEX = 0,
	ERR_CURRENT_INDEX,
	ERR_FREQ_INDEX,
	ERR_CODE_MAX,
};

enum {
	INV_STATUS_INDEX = 0,
	INV_I_RMS_INDEX,
	INV_RUN_FREQ_INDEX,
	INV_DC_VOLTAGE_INDEX,
	INV_IPM_TEMP_INDEX,
	INV_MOTOR_TEMP_INDEX,
	INV_STATUS_MAX,
};

#define PARAMETER_TYPE_LONG			0
#define PARAMETER_TYPE_FLOAT		1

#define	FREQ_VALUE_INDEX			0
#define ACCEL_TIME_INDEX        	1
#define DECEL_TIME_INDEX        	2
#define DIRECTION_INDEX        		3
#define VF_FOC_SEL_INDEX			4
#define ENERGY_SAVE_INDEX			5
#define PWM_FREQ_INDEX 				6
#define JUMP_ENABLE0_INDEX			7
#define JUMP_ENABLE1_INDEX			8
#define JUMP_ENABLE2_INDEX			9
#define JUMP_LOW0_INDEX 			10
#define JUMP_LOW1_INDEX 			11
#define JUMP_LOW2_INDEX 			12
#define JUMP_HIGH0_INDEX 			13
#define JUMP_HIGH1_INDEX 			14
#define JUMP_HIGH2_INDEX 			15
#define V_BOOST_INDEX				16
#define FOC_TORQUE_LIMIT_INDEX		17
#define BRK_TYPE_INDEX				18
#define BRK_FREQ_INDEX 				19
#define BRK_DCI_START_FREQ_INDEX	20
#define BRK_DCI_BLOCK_TIME_INDEX	21
#define BRK_DCI_BRAKING_TIME_INDEX	22
#define BRK_DCI_BRAKING_RATE_INDEX	23
#define OVL_WARN_LIMIT_INDEX		24
#define OVL_WR_DURATION_INDEX 		25
#define OVL_ENABLE_INDEX			26
#define OVL_TR_LIMIT_INDEX			27
#define OVL_TR_DURATION_INDEX		28
#define REGEN_RESISTANCE_INDEX		29
#define REGEN_THERMAL_INDEX			30
#define REGEN_POWER_INDEX			31
#define REGEN_BAND_INDEX 			32
#define FAN_COMMAND_INDEX 			33

#define STATOR_RESISTANCE_INDEX		34
#define ROTATOR_RESISTANCE_INDEX	35
#define INDUCTANCE_INDEX			36
#define NOLOAD_CURRENT_INDEX		37
#define RATED_CURRENT_INDEX			38
#define POLES_INDEX					39
#define INPUT_VOLTAGE_INDEX			40
#define RATED_FREQ_INDEX			41

#define INV_RUN_STOP_INDEX			42

#define	INV_PARAM_INDEX_MAX			43


extern inv_parameter_st iparam[];
////////////////////////////////////////////////

extern void PARAM_init(void);

extern void PARAM_startRun(void);
extern void PARAM_stopRun(void);
extern int PARAM_setFreq(union32_st value);
extern int PARAM_setAccel(union32_st value);
extern int PARAM_setDecel(union32_st value);
extern int PARAM_setDirection(union32_st value);
extern int PARAM_setVfFoc(union32_st value);
extern int PARAM_setEnergySave(union32_st value);
extern int PARAM_setPwmFreq(union32_st value);
extern int PARAM_setEnableJump0(union32_st value);
extern int PARAM_setEnableJump1(union32_st value);
extern int PARAM_setEnableJump2(union32_st value);
extern int PARAM_setJumpFreqLow0(union32_st value);
extern int PARAM_setJumpFreqLow1(union32_st value);
extern int PARAM_setJumpFreqLow2(union32_st value);
extern int PARAM_setJumpFreqHigh0(union32_st value);
extern int PARAM_setJumpFreqHigh1(union32_st value);
extern int PARAM_setJumpFreqHigh2(union32_st value);
extern int PARAM_setVoltageBoost(union32_st value);
extern int PARAM_setTorqueLimit(union32_st value);
extern int PARAM_setBrakeType(union32_st value);
extern int PARAM_setBrakeFreq(union32_st value);
extern int PARAM_setDciBrakeStartFreq(union32_st value);
extern int PARAM_setDciBrakeBlockTime(union32_st value);
extern int PARAM_setDciBrakeTime(union32_st value);
extern int PARAM_setDciBrakeRate(union32_st value);
extern int PARAM_setOvlWarnLevel(union32_st value);
extern int PARAM_setOvlWarnTime(union32_st value);
extern int PARAM_setOvlEnableTrip(union32_st value);
extern int PARAM_setOvlTripLevel(union32_st value);
extern int PARAM_setOvlTripTime(union32_st value);
extern int PARAM_setRegenResistance(union32_st value);
extern int PARAM_setRegenResistThermal(union32_st value);
extern int PARAM_setRegenResistPower(union32_st value);
extern int PARAM_setRegenBand(union32_st value);
extern int PARAM_setFanControl(union32_st value);

extern void PARAM_update(uint16_t index, uint16_t *buf);
extern uint16_t PARAM_getValue(uint16_t index, uint16_t *buf);
extern int PARAM_process(uint16_t index, union32_st data);

extern void PARAM_initErrInfo(void);
extern void PARAM_setErrInfo(uint16_t err_code, uint16_t err_status, float current, float freq);
extern uint16_t PARAM_getErrorInfo(uint16_t *buf);

extern void PARAM_initInvStatus(void);
extern void PARAM_setInvStatus(void);
extern uint16_t PARAM_getInvStatus(uint16_t *buf);

#endif /* PARAMETERS_H_ */
