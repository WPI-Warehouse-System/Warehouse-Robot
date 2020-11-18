/*
 * StudentsRobot.h
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#ifndef STUDENTSROBOT_H_
#define STUDENTSROBOT_H_
#include "config.h"
#include <Arduino.h>
#include "src/pid/ServoEncoderPIDMotor.h"
#include "src/pid/HBridgeEncoderPIDMotor.h"
#include "src/pid/ServoAnalogPIDMotor.h"
#include <ESP32Servo.h>
#include "DrivingChassis.h"
#include "LineFollower.h"
#include "Navigation.h"
#include "Parking.h"
#include "Pose.h"
#include "LiftControl.h"
#include "BinHandling.h"
#include "src/commands/IRCamSimplePacketComsServer.h"
#include "src/commands/GetIMU.h"

/**
 * @enum RobotStateMachine
 * These are sample values for a sample state machine.
 * Feel free to add ot remove values from here
 */
enum RobotStateMachine {
	StartupRobot = 0, StartRunning = 1, Running = 2, Halting = 3, Halt = 4, WAIT_FOR_MOTORS_TO_FINNISH=5, WAIT_FOR_TIME=6,
	Testing = 7, Navigating = 8, ParkingRobot = 9, HomingLift = 10, MovingLiftFromGUI = 11,  DeliveringBin = 12, ReturningBin = 13, //was 13

};

const String StringStates[14] = {"StartupRobot", "StartRunning", "Running", "Halting", "Halt", "WFMF", "WFT", "Test", "NAV", "PRK", "Homing", "MovingLift", "DLV", "RTN"};

/**
 * @enum ComStackStatusState
 * These are values for the communications stack
 * Don't add any more or change these. This is how you tell the GUI
 * what state your robot is in.
 */
enum ComStackStatusState {
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

/**
 * @enum ParkingStates
 */
enum ParkingStates {
	SETTING_PARKING_GOAL = 0,
	GOING_TO_PARKING_SPACE = 1,
	PARKING = 2,
};

/**
 * @enum NavigatingStates
 */
enum NavigatingStates {
	CHECKING_IF_PARKED = 0,
	LEAVING_PARKING_SPOT = 1,
	SETTING_NAV_GOAL = 2,
	NAVIGATING = 3,
};

/**
 * @enum BinDeliveryStates
 */
enum BinDeliveryStates {
	SETTING_DELIVERY_LOCATION = 0,
	GOING_TO_BIN = 1,
	PROCURING_BIN = 2,
	GOING_TO_USER = 3,
};

/**
 * @enum BinReturnStates
 */
enum BinReturnStates {
	SETTING_RETURN_LOCATION = 0,
	GOING_TO_SHELF = 1,
	RETURNING_BIN = 2,
};


enum HomingLiftStates {
	STARTING_HOME = 0,
	MOVING_TO_LOWER_LIMIT = 1,
	MOVING_TO_UPPER_LIMIT = 2,
	DONE_HOMING = 3

};

enum MovingLiftFromGUIStates {
	SET_LIFT_HEIGHT = 0,
	WAIT_FOR_HEIGHT_REACHED = 1,
	DONE_LIFTING = 2
};


/**
 * @class StudentsRobot
 */
class StudentsRobot {
private:
	PIDMotor * motor1;
	PIDMotor * motor2;
	PIDMotor * motor3;
	Servo * servo;
	DrivingChassis robotChassis;
	float lsensorVal=0;
	float rsensorVal=0;
	long nextTime =0;
  long startTime =0;
	RobotStateMachine nextStatus = StartupRobot;
	IRCamSimplePacketComsServer * IRCamera;
	GetIMU * IMU;
public:
	float liftHeight = 0;//In mm
	bool robotParked = false; // make false if not starting false
	/**
	 * Constructor for StudentsRobot
	 *
	 * attach the 4 actuators
	 *
	 * these are the 4 actuators you need to use for this lab
	 * all 4 must be attached at this time
	 * DO NOT reuse pins or fail to attach any of the objects
	 *
	 */
	StudentsRobot(PIDMotor * motor1,
			PIDMotor * motor2, PIDMotor * motor3,
			Servo * servo,IRCamSimplePacketComsServer * IRCam,GetIMU * imu);
	/**
	 * Command status
	 *
	 * this is sent upstream to the Java GUI to notify it of current state
	 */
	ComStackStatusState myCommandsStatus = Picking_up;
	/**
	 * This is internal data representing the runtime status of the robot for use in its state machine
	 */
	RobotStateMachine status = StartupRobot;

	RobotStateMachine lastStatus = StartupRobot;

	// This is the status to run to after navigation. Initialize to Running
	RobotStateMachine statusAfterNav = Running;

	// State variables for the enumeration of different routines. Initialized to first case
	ParkingStates parkingStatus = SETTING_PARKING_GOAL;

	NavigatingStates navigationStatus = SETTING_NAV_GOAL;

	BinDeliveryStates binDeliveryStatus = SETTING_DELIVERY_LOCATION;

	BinReturnStates binReturnStatus = SETTING_RETURN_LOCATION;

	HomingLiftStates homeLiftState = STARTING_HOME;

	MovingLiftFromGUIStates moveLiftState = DONE_LIFTING;




	// Objects for different routines robot is capable of
	Navigation navigation;
	Parking parking;
	LiftControl Lift;
	BinHandling binHandler;

	// goal column and goal row for navigation from a UI command
	int goalColumn = -2;
	int goalRow = 2;
	int goalShelf = 2;



	/**
	 * pidLoop This functoion is called to let the StudentsRobot controll the running of the PID loop functions
	 *
	 * The loop function on all motors needs to be run when this function is called and return fast
	 */
	void pidLoop();
	/**
	 * updateStateMachine use the stub state machine as a starting point.
	 *
	 * the students state machine can be updated with this function
	 */
	void updateStateMachine();
};

#endif /* STUDENTSROBOT_H_ */
