/*
 * ParkingRoutine.cpp
 *
 *  Created on: Oct 27, 2020
 *      Author: gentov
 */
#include "Parking.h"

//TODO: ALLOW APPROACH FROM EITHER SIDE
Parking::Parking(DrivingChassis* robotChassis){
	chassis = robotChassis;
}

ParkingRoutineStates Parking::checkParkingStatus(){
	switch(parkingState){
		case INITIALIZE_PARKING:
			parkingState = TURNING_TO_SPACE;
			break;
		case TURNING_TO_SPACE:
			Serial.println("TURNING TO SPACE");
			chassis->turnToHeading(-90, 7500);
			parkingStateAfterMotionSetpointReached = BACKING_UP_TO_OUTER_EDGE;
			parkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING;
			break;
		case BACKING_UP_TO_OUTER_EDGE:
			Serial.println("BACKING UP TO OUTER EDGE");
			chassis->driveStraight(-90, DRIVING_BACKWARDS);
            if(chassis -> lineSensor.onMarker()){
            	// get off the marker, so you can count the one that starts the parking space
            	chassis->driveBackwards(30, 1000);
            	parkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING;
            	parkingStateAfterMotionSetpointReached = BACKING_INTO_SPACE;
            }
			break;
		case BACKING_INTO_SPACE:
			Serial.println("BACKING INTO SPACE");
			chassis->driveStraight(-90, DRIVING_BACKWARDS);
            if(chassis -> lineSensor.onMarker()){
            	chassis->stop();
            	parkingState = FINISHED_PARKING;
            }
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING:
			if(chassis->statusOfChassisDriving() == REACHED_SETPOINT){
				parkingState = parkingStateAfterMotionSetpointReached;
			}
            break;
		case FINISHED_PARKING:
			Serial.println("PARKED");
			parkingState = INITIALIZE_PARKING;
			break;
		}
	return parkingState;
}

ExitParkingRoutineStates Parking::getOutOfParkingStatus(){
	switch(exitParkingState){
		case EXIT_PARKING_SPOT:
			chassis->driveForward(30, 1000);
		    exitParkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_EXIT_PARKING;
			exitParkingStateAfterMotionSetpointReached = DRIVE_UP_TO_OUTER_EDGE;
		break;
		case DRIVE_UP_TO_OUTER_EDGE:
			chassis->driveStraight(-90, DRIVING_FORWARDS);
			if(chassis -> lineSensor.onMarker()){
				chassis->stop();
				exitParkingState = DRIVE_FORWARD;
		    }
			break;
		case DRIVE_FORWARD:
			chassis->driveForward(130, 2500);
		    exitParkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_EXIT_PARKING;
			exitParkingStateAfterMotionSetpointReached = FINISHED_EXIT_PARKING;
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_EXIT_PARKING:
			if(chassis->statusOfChassisDriving() == REACHED_SETPOINT){
				exitParkingState = exitParkingStateAfterMotionSetpointReached;
			}
            break;
		case FINISHED_EXIT_PARKING:
			Serial.println("EXITED PARKING_SPOT");
			exitParkingState = EXIT_PARKING_SPOT;
		}
	return exitParkingState;
}



