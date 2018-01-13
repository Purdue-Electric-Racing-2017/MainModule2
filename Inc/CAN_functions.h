/*
 * CAN_functions.h
 *
 *  Created on: Dec 26, 2016
 *      Author: ben
 */

#ifndef CAN_FUNCTIONS_H_
#define CAN_FUNCTIONS_H_

#include "stm32f4xx_hal.h"
//rtos includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

typedef struct {
	CAN_HandleTypeDef*  phcan;
	QueueHandle_t	 	queue_tx;
	QueueHandle_t	 	queue_rx;
	SemaphoreHandle_t	mutex;
} RTOS_CAN_t;

//function prototypes
BaseType_t initRTOS_CAN(RTOS_CAN_t *can, CAN_HandleTypeDef *hcan);
void taskTXCAN(RTOS_CAN_t *can);

#endif /* CAN_FUNCTIONS_H_ */
