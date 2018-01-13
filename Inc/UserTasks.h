/*
 * UserTasks.h
 *
 *  Created on: Jan 20, 2017
 *      Author: Kai Strubel
 */

#ifndef USERTASKS_H_
#define USERTASKS_H_

#define DELAY_SEND_THROTTLE				500 /portTICK_RATE_MS // in ms
#define DELAY_SEND_BRAKE				10 /portTICK_RATE_MS // in ms


void taskBlink_LED(int *ledID);
void taskSendThrottleRaw();
void taskSendBrakeRaw();
//void taskBlink_LED(void *pvParams);


#endif /* USERTASKS_H_ */
