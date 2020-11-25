/*
 * ParkingRoutine.h
 *
 *  Created on: Oct 27, 2020
 *      Author: gentov
 */

#include "DrivingChassis.h"
#include "LineFollower.h"

#ifndef PARKINGROUTINE_H_
#define PARKINGROUTINE_H_

/*
 * @enum ParkingRoutineStates
 * States used in the parking state machine, not the main robot state machine
*/
enum ParkingRoutineStates{
	INITIALIZE_PARKING = 0,
	TURNING_TO_SPACE = 1,
	BACKING_UP_TO_OUTER_EDGE = 2,
	BACKING_INTO_SPACE = 3,
	FINISHED_PARKING = 4,
	WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING = 5,
	TIMED_OUT_PARKING = 6,
};

/*
 * @enum ExitParkingRoutineStates
 * States used in the exit parking state machine, not the main robot state machine
*/
enum ExitParkingRoutineStates{
	EXIT_PARKING_SPOT = 0,
	DRIVE_UP_TO_OUTER_EDGE = 1,
	DRIVE_FORWARD = 2,
	FINISHED_EXIT_PARKING = 3,
	WAIT_FOR_MOTION_SETPOINT_REACHED_EXIT_PARKING = 4,
	TIMED_OUT_EXIT_PARKING = 5,
};

class Parking{
    public:
	   Parking(DrivingChassis* robotChassis);

		/**
		 * Checks the status of the parking routine. Called repeatedly in a loop.
		 *
		 * @return the current state in ParkingRoutineStates
		 * @note this function is fast-return and should not block
		 */
	   ParkingRoutineStates checkParkingStatus();

		/**
		 * Checks the status of the parking exit routine. Called repeatedly in a loop.
		 *
		 * @return the current state in ExitParkingRoutineStates
		 * @note this function is fast-return and should not block
		 */
	   ExitParkingRoutineStates getOutOfParkingStatus();

	   DrivingChassis* chassis = NULL;

	   ParkingRoutineStates parkingState = INITIALIZE_PARKING;

	   // This is the parkingState that occurs after a setpoint has been reached
	   ParkingRoutineStates parkingStateAfterMotionSetpointReached = INITIALIZE_PARKING;


	   ExitParkingRoutineStates exitParkingState = EXIT_PARKING_SPOT;

	   // This is the exitParkingState that occurs after a setpoint has been reached
	   ExitParkingRoutineStates exitParkingStateAfterMotionSetpointReached = EXIT_PARKING_SPOT;

    private:
};


#endif /* PARKINGROUTINE_H_ */
