/***************************************************************************
*
*     File Information
*
*     Name of File: car.c
*
*     Authors (Include Email):
*       1. Ben Ng				xbenng@gmail.com
*
*     File dependents: (header files, flow charts, referenced documentation)
*       1. car.h
*
*     File Description:
*     	Functions to control the physical car
*
***************************************************************************/

#include "car.h"


void carSetBrakeLight(Brake_light_status_t status)
/***************************************************************************
*
*     Function Information
*
*     Name of Function: setBrakeLight
*
*     Programmer's Name: Ben Ng xbenng@gmail.com
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1. Brake_light_status_t status, value to write to GPIO pin
*
*      Global Dependents:
*
*     Function Description:
*			turns brakelight on or off
***************************************************************************/
{
	HAL_GPIO_WritePin(BRAKE_LIGHT_PORT, BRAKE_LIGHT_PIN, status);
}


void carInit() {
	car.state = CAR_STATE_INIT;
	car.pb_mode = PEDALBOX_MODE_DIGITAL;
	car.throttle_acc = 0;
	car.brake = 0;
	car.phcan = &hcan1;
	car.calibrate_flag = CALIBRATE_NONE;
	car.throttle1_min = 0x0aa0;
	car.throttle1_max = 0x0000;
	car.throttle2_min = 0x0940;
	car.throttle2_max = 0x0000;
	car.brake1_min = 0x0290;
	car.brake1_max = 0x0900;
	car.brake2_min = 0x0290;
	car.brake2_max = 0x0900;
	car.pb_msg_rx_time = 4294967295;


}

void ISR_StartButtonPressed() {
	if (car.state == CAR_STATE_INIT)
	{
		//if (//car.brake >= BRAKE_PRESSED_THRESHOLD &&//check if brake is pressed before starting car
		//	HAL_GPIO_ReadPin(Precharge_IN_GPIO_Port, Precharge_IN_Pin) == GPIO_PIN_SET //check if precharge has finished
		//	)
		car.state = CAR_STATE_PREREADY2DRIVE;
	} else {
		xTaskCreate(taskSoundBuzzer, "buzzer is on", 64, (void *) 300, 1, NULL);

		car.state = CAR_STATE_INIT;
	}
}


// haha VVVVV
void taskLaunchControl() {
	while (1) {
		vTaskDelay(LAUNCH_CONTROL_INTERVAL_MS);
	}
}


//TODO Potential MC ping function
//TODO BMS functions

int mainModuleWatchdogTask() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: mainModuleTimeCheckIdle
*
*     Programmer's Name: Kai Strubel
*     					 Ben Ng			xbenng@gmail.com
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*	    1.bool launchControl
*		2.float MMPB_TIME time pedal box message handler function was last run
*		3.float MMWM_TIME time wheel module handler function was last run
*		4.float torque
*		5.float currentTime
*
*     Function Description:
*		Checks if wheel module and pedal box are still communicating
*
***************************************************************************/
	while (1) {
		/*
		//check how old the wheel module data is, if its too old, then turn off LC
		if (current_time_ms - MMWM_TIME > LC_THRESHOLD) {
			launchControl = 0;
			//error
		}*/
		vTaskDelay(500);
	}
}

int taskHeartbeat() {
/***************************************************************************
*.
*     Function Information
*
*     Name of Function: heartbeatIdle
*
*     Programmer's Name: Kai Strubel
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*      Global Dependents:
*
*     Function Description:
*		Heart beat to communicate that main module is alive
*
***************************************************************************/
	// write to GPIO
	while (1) {
		HAL_GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
		vTaskDelay(HEARTBEAT_PERIOD);
	}
}

void initRTOSObjects() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: startTasks
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: int
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*		all xTaskCreate calls
*		all xQueueCreate calls
*
***************************************************************************/

	/* Create Queues */

	car.q_rxcan = 			xQueueCreate(QUEUE_SIZE_RXCAN, sizeof(CanRxMsgTypeDef));
	car.q_txcan = 			xQueueCreate(QUEUE_SIZE_TXCAN, sizeof(CanTxMsgTypeDef));
	car.q_pedalboxmsg = 	xQueueCreate(QUEUE_SIZE_PEDALBOXMSG, sizeof(Pedalbox_msg_t));
	car.q_mc_frame = 		xQueueCreate(QUEUE_SIZE_MCFRAME, sizeof(CanRxMsgTypeDef));

	car.m_CAN =				xSemaphoreCreateMutex(); //mutex to protect CAN peripheral

	/* Create Tasks */

	//todo optimize stack depths http://www.freertos.org/FAQMem.html#StackSize
	xTaskCreate(taskPedalBoxMsgHandler, "PedalBoxMsgHandler", 512, NULL, 1, NULL);
	xTaskCreate(taskCarMainRoutine, "CarMainRoutine", 512, NULL, 1, NULL);
	xTaskCreate(taskTXCAN, "TX CAN", 256, NULL, 1, NULL);
	xTaskCreate(taskRXCANProcess, "RX CAN Process", 256, NULL, 1, NULL);
	xTaskCreate(taskRXCAN, "RX CAN", 256, NULL, 1, NULL);
	xTaskCreate(taskBlink, "blink", 256, NULL, 1, NULL);
	//xTaskCreate(taskSendAccelero, "accel", 256, NULL, 1, NULL);
	//xTaskCreate(taskMotorControllerPoll, "Motor Poll", 256, NULL, 1, NULL);
 }
extern uint8_t variable;
void taskBlink(void* can)
{
	//vTaskDelay(5000); //TESTING1
	while (1)
	{
		//HAL_GPIO_TogglePin(FRG_RUN_CTRL_GPIO_Port, FRG_RUN_CTRL_Pin);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);

//		CanTxMsgTypeDef tx;
//		tx.IDE = CAN_ID_STD;
//		tx.RTR = CAN_RTR_DATA;
//		tx.StdId = 0x200;
//		tx.DLC = 1;
//		if (car.state == CAR_STATE_INIT)
//		{
//			tx.Data[0] = 0;
//		}
//		else if (car.state == CAR_STATE_READY2DRIVE)
//		{
//			tx.Data[0] = 1;
//		}
//		//req regid 40
//		car.phcan->pTxMsg = &tx;
//		//HAL_CAN_Transmit_IT(car.phcan);						//transmit staged message
//
//		xQueueSendToBack(car.q_txcan, &tx, 100);
		mcCmdTransmissionRequestSingle(0x40);
		vTaskDelay(255);
	}
}


void taskSoundBuzzer(int* time_ms) {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskSoundBuzzer
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*		ready to drive sound task
*
***************************************************************************/
	while (1) {
		HAL_GPIO_WritePin(Buzzer_CTRL_GPIO_Port, Buzzer_CTRL_Pin, GPIO_PIN_SET); //turn on buzzer
		//enable FRG/RUN 0.5s after RFE.
		vTaskDelay((uint32_t) time_ms / portTICK_RATE_MS);
		HAL_GPIO_WritePin(Buzzer_CTRL_GPIO_Port, Buzzer_CTRL_Pin, GPIO_PIN_RESET); //turn off buzzer
		vTaskDelete(NULL);
	}
}

void taskSendAccelero() {
/***************************************************************************
*
*     Function Information
*
*     Name of Function: taskSend
*
*     Programmer's Name: Ben Ng
*
*     Function Return Type: void
*
*     Parameters (list data type, name, and comment one per line):
*       1.
*
*     Global Dependents:
*
*     Function Description:
*     polls accelerometer, and sends the acceleration data over CAN
*
***************************************************************************/
	while (1) {
		Acceldata_t adata;
		Accelro_Read_Axes(&adata);
		CanTxMsgTypeDef tx;
		tx.StdId = 0x420;
		tx.Data[0] = (adata.x >> 8) & 0xFF;
		tx.Data[1] = adata.x & 0xFF;
		tx.Data[2] = (adata.y >> 8) & 0xFF;
		tx.Data[3] = adata.y & 0xFF;
		tx.Data[4] = (adata.z >> 8) & 0xFF;
		tx.Data[5] = adata.z & 0xFF;
		tx.DLC = 6;
		tx.RTR = CAN_RTR_DATA;
		tx.IDE = CAN_ID_STD;
		car.phcan->pTxMsg = &tx;
		HAL_CAN_Transmit_IT(car.phcan);
		vTaskDelay(10);
	}
}


void taskCarMainRoutine() {
	while (1)
	{
		//get current time in ms
		uint32_t current_time_ms = xTaskGetTickCount() / portTICK_PERIOD_MS;
		uint16_t torque_to_send = 0;

		//always active block
		//Brake
		//check if brake level is greater than the threshold level
		if (car.brake >= BRAKE_PRESSED_THRESHOLD) {
			//brake is presssed
			carSetBrakeLight(BRAKE_LIGHT_ON);  //turn on brake light


			//EV 2.5, check if the throttle level is greater than 25% while brakes are on
//				if (throttle_avg > APPS_BP_PLAUS_THRESHOLD) {
//					//set apps-brake pedal plausibility error
//					car.apps_bp_plaus = PEDALBOX_STATUS_ERROR;
//				}
		} else {
			//brake is not pressed
			carSetBrakeLight(BRAKE_LIGHT_OFF);  //turn off brake light
		}

		if(car.apps_state_eor == PEDALBOX_STATUS_ERROR)
		{
			CanTxMsgTypeDef tx;
			tx.StdId = 0x333;
			tx.Data[0] = 1;
			tx.DLC = 1;
			tx.IDE = CAN_ID_STD;
			tx.RTR = CAN_RTR_DATA;
			car.phcan->pTxMsg = &tx;
			HAL_CAN_Transmit_IT(car.phcan);
			vTaskDelay(10);
		}

		//state dependent block
		if (car.state == CAR_STATE_INIT)
		{
			disableMotor();
			HAL_GPIO_WritePin(Pump_Relay_CTRL_GPIO_Port, Pump_Relay_CTRL_Pin, GPIO_PIN_RESET); //turn on pump


			//assert these pins always
			HAL_GPIO_WritePin(SDC_CTRL_GPIO_Port, SDC_CTRL_Pin, GPIO_PIN_SET); //close SDC
			HAL_GPIO_WritePin(Motor_Controller_Relay_CTRL_GPIO_Port, Motor_Controller_Relay_CTRL_Pin, GPIO_PIN_SET); //turn on mc
		}
		if (car.state == CAR_STATE_PREREADY2DRIVE)
		{

			HAL_GPIO_WritePin(Pump_Relay_CTRL_GPIO_Port, Pump_Relay_CTRL_Pin, GPIO_PIN_SET); //turn on pump
			//bamocar 5.2
			//Contacts of the safety device closed,
			//enable FRG/RUN 0.5s after RFE.
			enableMotorController();
			//turn on buzzer
			xTaskCreate(taskSoundBuzzer, "buzzer is on", 64, (void *) 1000, 1, NULL);
			car.state = CAR_STATE_READY2DRIVE;  //car is started
		}
		if (car.state == CAR_STATE_READY2DRIVE)
		{
			//assert these pins during r2d
			HAL_GPIO_WritePin(Pump_Relay_CTRL_GPIO_Port, Pump_Relay_CTRL_Pin, GPIO_PIN_SET);

			//check if the age of the pedalbox message is greater than the timeout
			if (current_time_ms - car.pb_msg_rx_time > PEDALBOX_TIMEOUT && PEDALBOX_TIMEOUT != 0)
			{
				torque_to_send = 0;
				car.apps_state_timeout = PEDALBOX_STATUS_ERROR;
				//todo send a CAN message to dash?
			}
			else
			{
				car.apps_state_timeout = PEDALBOX_STATUS_NO_ERROR;
			}
			if (car.apps_state_bp_plaus == PEDALBOX_STATUS_NO_ERROR &&
				car.apps_state_eor == PEDALBOX_STATUS_NO_ERROR &&
				car.apps_state_imp == PEDALBOX_STATUS_NO_ERROR &&
				car.apps_state_timeout == PEDALBOX_STATUS_NO_ERROR)
			{
				//launch control
				/*if (lc_status == LC_ACTIVATED) {
					if(throttle > apps_max) {
						// apps - accelerator pedal position system
						// bse - brake system encoder
						apps_max = throttle;
						throttle *= scaleFactor;
					}
					else {
						if (apps_max - throttle > LC_THRESHOLD) {
							launchControl = false;
						}
						else {
							throttle *= scaleFactor;

						}
					}

				}*/

				torque_to_send = car.throttle_acc; //gets average

				//car.throttle_acc = 0;
				//car.throttle_cnt = 0;
			}
		}
		if (car.state == CAR_STATE_ERROR)
		{
			disableMotor();
		}

		// calculate
//			calcTorqueLimit = (80000 / (actualDC * 10 * actualV * 10)); //(DCLimit / (actualDC * 10)) * actualTorque;
//			if(torque_to_send/MAX_THROTTLE_LEVEL > calcTorqueLimit)
//			{
//				torque_to_send = calcTorqueLimit * torque_to_send;
//			}

		mcCmdTorque(torque_to_send);  //command the MC to move the motor


		//wait until
		vTaskDelay(TORQUE_SEND_PERIOD);

	}

}
