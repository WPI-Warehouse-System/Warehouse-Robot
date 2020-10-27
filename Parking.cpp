/*
 * ParkingRoutine.cpp
 *
 *  Created on: Oct 27, 2020
 *      Author: gentov
 */
#include "Parking.h"

//TODO: ALLOW APPROACH FROM EITHER SIDE
Parking::Parking(DrivingChassis *robotChassis, LineFollower* lineFollower){
	drivingChassis = robotChassis;
	lineSensor = lineFollower;
}

void Parking::initializeParkingRoutine(DrivingChassis* chassis, LineFollower* lineFollowingSensor){
	drivingChassis = chassis;
	lineSensor = lineFollowingSensor;
}

ParkingRoutineStates Parking::checkParkingStatus(){
	switch(parkingState){
		case INITIALIZE_PARKING:
			parkingState = TURNING_TO_SPACE;
			break;
		case TURNING_TO_SPACE:
			Serial.println("TURNING TO SPACE");
			drivingChassis->turnToHeading(-90, 5000);
			parkingStateAfterMotionSetpointReached = BACKING_UP_TO_OUTER_EDGE;
			parkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING;
			break;
		case BACKING_UP_TO_OUTER_EDGE:
			Serial.println("BACKING UP TO OUTER EDGE");
			drivingChassis->driveStraight(-90, DRIVING_BACKWARDS);
            if(lineSensor -> onMarker()){
            	// get off the marker, so you can count the one that starts the parking space
            	drivingChassis->driveBackwards(30, 1000);
            	parkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING;
            	parkingStateAfterMotionSetpointReached = BACKING_INTO_SPACE;
            }
			break;
		case BACKING_INTO_SPACE:
			Serial.println("BACKING INTO SPACE");
			drivingChassis->driveStraight(-90, DRIVING_BACKWARDS);
            if(lineSensor -> onMarker()){
            	drivingChassis->stop();
            	parkingState = FINISHED_PARKING;
            }
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING:
			if(drivingChassis->statusOfChassisDriving() == REACHED_SETPOINT){
				parkingState = parkingStateAfterMotionSetpointReached;
			}
            break;
		case FINISHED_PARKING:
			Serial.println("PARKED");
			parkingState = INITIALIZE_PARKING;
		}
	return parkingState;
}



