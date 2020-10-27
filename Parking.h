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

enum ParkingRoutineStates{
	INITIALIZE_PARKING = 0,
	TURNING_TO_SPACE = 1,
	BACKING_UP_TO_OUTER_EDGE = 2,
	BACKING_INTO_SPACE = 3,
	FINISHED_PARKING = 4,
	WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING = 5,
};

class Parking{
    public:
	   Parking(DrivingChassis* robotChassis, LineFollower* lineFollower);
	   void initializeParkingRoutine(DrivingChassis* robotChassis, LineFollower* lineFollower);
	   ParkingRoutineStates checkParkingStatus();

	   DrivingChassis* drivingChassis;
	   LineFollower* lineSensor;

	   ParkingRoutineStates parkingState = INITIALIZE_PARKING;

	   // This is the parkingState that occurs after a setpoint has been reached
	   ParkingRoutineStates parkingStateAfterMotionSetpointReached;

    private:
};


#endif /* PARKINGROUTINE_H_ */
