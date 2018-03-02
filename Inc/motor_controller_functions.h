/*
 * motor_controller_functions.h
 *
 *  Created on: Dec 23, 2016
 *      Author: ben ng for purdue electric racing
 */

#ifndef MOTOR_CONTROLLER_FUNCTIONS_H_
#define MOTOR_CONTROLLER_FUNCTIONS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "car.h"

//function prototypes
int taskProcessMotorControllerFrame();
void mcCmdTransmissionRequestPermenant (uint8_t regid, uint8_t retransmitTimeMS);
void mcCmdTransmissionRequestSingle(uint8_t regid);
void mcCmdTransmissionAbortPermenant(uint8_t regid);
void mcCmdTorque(uint16_t);

//CAN Defines

//DLC defines
#define DLC_CMD_REQUEST_DATA		3
#define DLC_CMD_TORQUE				3

//REGID, page 15 ndrive manual
#define REGID_SPEED_CMD_VAL			0x5d
#define REGID_SPEED_CMD_VAL_BRAMP	0x31
#define REGID_SPEED_CMD_VAL_ARAMP	0x32	//use this
#define REGID_SPEED_ACTUAL			0x30  //rpm
#define REGID_SPEED_ACTUAL_FILTER 	0xa8
#define REGID_CMD_REQUEST_DATA		0x3d
#define REGID_CMD_TORQUE			0x90
#define REGID_CMD_SPEED				0x31
#define REGID_CMD_POS				0x91
#define REGID_MESSAGES				0x8F
#define REGID_MODE					0x51
#define REGID_STATE					0x40
#define REGID_I_ACTUALD				0x28
#define REGID_I_ACTUALQ				0x27
#define REGID_P_MOTOR				0xF6
#define REGID_P_REGEN				0x45 //high
#define REGID_T_MOTOR				0x49
#define REGID_T_IGBT				0x4A
#define REGID_T_AIR					0x4B
#define REGID_I_REDA				0x48
#define	REGID_V_OUT					0x8A
#define REGID_I_ACT					0x20


//0x40 status bits
#define STATUS_ENABLE_MASK								1 << STATUS_ENABLE_OFFSET
#define STATUS_DRIVE_STOPPED_MASK						1 << STATUS_DRIVE_STOPPED_OFFSET
#define STATUS_LIM_PLUS_MASK							1 << STATUS_LIM_PLUS_OFFSET
#define STATUS_LIM_MINUS_MASK							1 << STATUS_LIM_MINUS_OFFSET									//bit 4 is free
#define STATUS_CONTINUOUS_CURRENT_MASK					1 << STATUS_CONTINUOUS_CURRENT_OFFSET
#define STATUS_POS_CURRENT_CONTROLLER_MASK				1 << STATUS_POS_CURRENT_CONTROLLER_OFFSET
#define STATUS_POS_CONTROLLER_MASK						1 << STATUS_POS_CONTROLLER_OFFSET
#define STATUS_SPEED_CONTROL_MASK						1 << STATUS_SPEED_CONTROL_OFFSET
#define STATUS_SPEED_IS_0_MASK							1 << STATUS_SPEED_IS_0_OFFSET							//speed inferior to 0.1%
#define STATUS_REF_INPUT_SELECTED_MASK					1 << STATUS_REF_INPUT_SELECTED_OFFSET
#define STATUS_REF_RUN_MASK								1 << STATUS_REF_RUN_OFFSET
#define STATUS_REF_POS_IDENTIFIED_MASK					1 << STATUS_REF_POS_IDENTIFIED_OFFSET
#define STATUS_POS_IN_TOLERANCE_MASK					1 << STATUS_POS_IN_TOLERANCE_OFFSET
#define STATUS_READY_MASK								1 << STATUS_READY_OFFSET
#define STATUS_ACTIVE_BRAKE_MASK						1 << STATUS_ACTIVE_BRAKE_OFFSET
#define STATUS_INVERTED_CMD_MASK						1 << STATUS_INVERTED_CMD_OFFSET
#define STATUS_SPEED_LIM_VIA_SWITCH_MASK				1 << STATUS_SPEED_LIM_VIA_SWITCH_OFFSET
#define STATUS_POS_SPEED_LIM_VIA_SWITCH_MASK			1 << STATUS_POS_SPEED_LIM_VIA_SWITCH_OFFSET
#define STATUS_NEG_SPEED_LIM_VIA_SWITCH_MASK			1 << STATUS_NEG_SPEED_LIM_VIA_SWITCH_OFFSET
#define STATUS_CURRENT_LIM_VIA_SWITCH_MASK				1 << STATUS_CURRENT_LIM_VIA_SWITCH_OFFSET
#define STATUS_ACTIVE_CURRENT_REDUC_MASK				1 << STATUS_ACTIVE_CURRENT_REDUC_OFFSET
#define STATUS_CURRENT_REDUC_VIA_SPEED_MASK				1 << STATUS_CURRENT_REDUC_VIA_SPEED_OFFSET
#define STATUS_CURRENT_REDUC_VIA_OSTEMP_MASK			1 << STATUS_CURRENT_REDUC_VIA_OSTEMP_OFFSET				//output stage temperature
#define STATUS_CURRENT_LIM2CONTINUOUS_VIA_OSTEMP_MASK	1 << STATUS_CURRENT_LIM2CONTINUOUS_VIA_OSTEMP_OFFSET	//output stage temperature
#define STATUS_CURRENT_REDUC_4FREQ_MASK					1 << STATUS_CURRENT_REDUC_4FREQ_OFFSET					//current reduction when freq < 2Hz
#define STATUS_CURRENT_REDUC_VIA_MOTOR_TEMP_MASK		1 << STATUS_CURRENT_REDUC_VIA_MOTOR_TEMP_OFFSET			//motor temperature
#define STATUS_CURRENT_LIM_VIA_INPUT2_MASK				1 << STATUS_CURRENT_LIM_VIA_INPUT2_OFFSET

#define STATUS_ENABLE_OFFSET						0
#define STATUS_DRIVE_STOPPED_OFFSET					1
#define STATUS_LIM_PLUS_OFFSET						2
#define STATUS_LIM_MINUS_OFFSET						3		//bit 4 is free
#define STATUS_CONTINUOUS_CURRENT_OFFSET				5
#define STATUS_POS_CURRENT_CONTROLLER_OFFSET				6
#define STATUS_POS_CONTROLLER_OFFSET					7
#define STATUS_SPEED_CONTROL_OFFSET					8
#define STATUS_SPEED_IS_0_OFFSET					9		//speed inferior to 0.1%
#define STATUS_REF_INPUT_SELECTED_OFFSET				10
#define STATUS_REF_RUN_OFFSET						11
#define STATUS_REF_POS_IDENTIFIED_OFFSET				12
#define STATUS_POS_IN_TOLERANCE_OFFSET					13
#define STATUS_READY_OFFSET						14
#define STATUS_ACTIVE_BRAKE_OFFSET					15
#define STATUS_INVERTED_CMD_OFFSET					16
#define STATUS_SPEED_LIM_VIA_SWITCH_OFFSET				17
#define STATUS_POS_SPEED_LIM_VIA_SWITCH_OFFSET				18
#define STATUS_NEG_SPEED_LIM_VIA_SWITCH_OFFSET				19
#define STATUS_CURRENT_LIM_VIA_SWITCH_OFFSET				20
#define STATUS_ACTIVE_CURRENT_REDUC_OFFSET				21
#define STATUS_CURRENT_REDUC_VIA_SPEED_OFFSET				22
#define STATUS_CURRENT_REDUC_VIA_OSTEMP_OFFSET				23		//output stage temperature
#define STATUS_CURRENT_LIM2CONTINUOUS_VIA_OSTEMP_OFFSET			24		//output stage temperature
#define STATUS_CURRENT_REDUC_4FREQ_OFFSET				25		//current reduction when freq < 2Hz
#define STATUS_CURRENT_REDUC_VIA_MOTOR_TEMP_OFFSET			26		//motor temperature
#define STATUS_CURRENT_LIM_VIA_INPUT2_OFFSET				27


//0x51 mode selection bits
#define MODE_SPEED0_MASK				1 << MODE_SPEED0_OFFSET
#define MODE_ENABLEOFF_MASK				1 << MODE_ENABLEOFF_OFFSET
#define MODE_CANCELCALCYCLE_MASK		1 << MODE_CANCELCALCYCLE_OFFSET
#define MODE_STATUSCAN_MASK				1 << MODE_STATUSCAN_OFFSET
#define MODE_ICLIPON_MASK				1 << MODE_ICLIPON_OFFSET
#define MODE_NCLIPON_MASK				1 << MODE_NCLIPON_OFFSET
#define MODE_MIXANAON_MASK				1 << MODE_MIXANAON_OFFSET
#define MODE_ALLOWSYNC_MASK				1 << MODE_ALLOWSYNC_OFFSET
#define MODE_HANDWHEEL_MASK				1 << MODE_HANDWHEEL_OFFSET
#define MODE_SPEED0_OFFSET				1
#define MODE_ENABLEOFF_OFFSET			2
#define MODE_CANCELCALCYCLE_OFFSET		3
#define MODE_STATUSCAN_OFFSET			4
#define MODE_ICLIPON_OFFSET				5
#define MODE_NCLIPON_OFFSET				6
#define MODE_MIXANAON_OFFSET			7
#define MODE_ALLOWSYNC_OFFSET			8
#define MODE_HANDWHEEL_OFFSET			9


//0x8f fault masks, messages, errors, warnings, page 111
#define MESSAGES_BADPARAS_MASK			1 << MESSAGES_BADPARAS_OFFSET
#define MESSAGES_POWERFAULT_MASK		1 << MESSAGES_POWERFAULT_OFFSET
#define MESSAGES_RFEFAULT_MASK			1 << MESSAGES_RFEFAULT_OFFSET
#define MESSAGES_BUSTIMEOUT_MASK		1 << MESSAGES_BUSTIMEOUT_OFFSET
#define MESSAGES_FEEDBACK_MASK			1 << MESSAGES_FEEDBACK_OFFSET
#define MESSAGES_POWERVOLTAGE_MASK		1 << MESSAGES_POWERVOLTAGE_OFFSET
#define MESSAGES_MOTORTEMP_MASK			1 << MESSAGES_MOTORTEMP_OFFSET
#define MESSAGES_DEVICETEMP_MASK		1 << MESSAGES_DEVICETEMP_OFFSET
#define MESSAGES_OVERVOLTAGE_MASK		1 << MESSAGES_OVERVOLTAGE_OFFSET
#define MESSAGES_IPEAK_MASK				1 << MESSAGES_IPEAK_OFFSET
#define MESSAGES_RACEAWAY_MASK			1 << MESSAGES_RACEAWAY_OFFSET
#define MESSAGES_USER_MASK				1 << MESSAGES_USER_OFFSET
#define MESSAGES_I2R_MASK				1 << MESSAGES_I2R_OFFSET //bit 13 reserved, not skipped
#define MESSAGES_HWFAIL_MASK			1 << MESSAGES_HWFAIL_OFFSET
#define MESSAGES_BALLAST_MASK			1 << MESSAGES_BALLAST_OFFSET
#define MESSAGES_WARNING0_MASK			1 << MESSAGES_WARNING0_OFFSET
#define MESSAGES_ILLEGALSTATUR_MASK		1 << MESSAGES_ILLEGALSTATUR_OFFSET
#define MESSAGES_WARNING2_MASK			1 << MESSAGES_WARNING2_OFFSET
#define MESSAGES_POWERVOLTAGE_MASK		1 << MESSAGES_POWERVOLTAGE_OFFSET
#define MESSAGES_MOTORTEMP_MASK			1 << MESSAGES_MOTORTEMP_OFFSET
#define MESSAGES_DEVICETEMP_MASK		1 << MESSAGES_DEVICETEMP_OFFSET
#define MESSAGES_OVERVOLTAGE_MASK		1 << MESSAGES_OVERVOLTAGE_OFFSET
#define MESSAGES_IPEAK_MASK				1 << MESSAGES_IPEAK_OFFSET

#define MESSAGES_BADPARAS_OFFSET		0
#define MESSAGES_POWERFAULT_OFFSET		1
#define MESSAGES_RFEFAULT_OFFSET		2
#define MESSAGES_BUSTIMEOUT_OFFSET		3
#define MESSAGES_FEEDBACK_OFFSET		4
#define MESSAGES_POWERVOLTAGE_OFFSET	5
#define MESSAGES_MOTORTEMP_OFFSET		6
#define MESSAGES_DEVICETEMP_OFFSET		7
#define MESSAGES_OVERVOLTAGE_OFFSET		8
#define MESSAGES_IPEAK_OFFSET			9
#define MESSAGES_RACEAWAY_OFFSET		10
#define MESSAGES_USER_OFFSET			11
#define MESSAGES_I2R_OFFSET				12 //bit 13 reserved, not skipped
#define MESSAGES_HWFAIL_OFFSET			14
#define MESSAGES_BALLAST_OFFSET			15
#define MESSAGES_WARNING0_OFFSET		16
#define MESSAGES_ILLEGALSTATUR_OFFSET	17
#define MESSAGES_WARNING2_OFFSET		18
//#define MESSAGES_POWERVOLTAGE_OFFSET	21
//#define MESSAGES_MOTORTEMP_OFFSET		22
//#define MESSAGES_DEVICETEMP_OFFSET		23
//#define MESSAGES_OVERVOLTAGE_OFFSET		24
//#define MESSAGES_IPEAK_OFFSET			25

//range definitions
#define RETRANSMISSION_SINGLE		0x00;
#define RETRANSMISSION_ABORT		0xFF;  //example 10, BAMOCAR CAN MANUAL

//function parameters


/*mc notes
 * N = speed = rpm
 *
 * little endian, {[7:0], [15:8], ...}
 *
 * page 105 command modes
 */
#endif /* MOTOR_CONTROLLER_FUNCTIONS_H_ */
