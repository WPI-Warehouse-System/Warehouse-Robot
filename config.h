/*
 * config.h
 *
 *  Created on: Nov 5, 2018
 *      Author: hephaestus
 */

#ifndef SRC_CONFIG_H_
#define SRC_CONFIG_H_

#define USE_WIFI

// Pins
#define MOTOR1_PWM 18
#define MOTOR2_PWM 16
#define MOTOR3_PWM 17

#define MOTOR1_DIR 15
#define MOTOR2_DIR 19

#define MOTOR1_ENCA 36
#define MOTOR1_ENCB 39

#define MOTOR2_ENCA 34
#define MOTOR2_ENCB 35

#define MOTOR3_ENCA 25
#define MOTOR3_ENCB 26

#define SERVO_PIN 27

enum RobotStatus {
	Ready_for_new_task = 0,
	Heading_to_pickup = 1,
	Waiting_for_approval_to_pickup = 2,
	Picking_up = 3,
	Heading_to_Dropoff = 4,
	Waiting_for_approval_to_dropoff = 5,
	Dropping_off = 6,
	Heading_to_safe_zone = 7,
	Fault_failed_pickup = 8,
	Fault_failed_dropoff = 9,
	Fault_excessive_load = 10,
	Fault_obstructed_path = 11,
	Fault_E_Stop_pressed = 12
};


#endif /* SRC_CONFIG_H_ */
