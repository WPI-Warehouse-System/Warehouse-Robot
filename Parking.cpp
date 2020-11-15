/*
 * ParkingRoutine.cpp
 *
 *  Created on: Oct 27, 2020
 *      Author: gentov
 */
#include "Parking.h"

Parking::Parking(DrivingChassis* robotChassis){
	chassis = robotChassis;
}

ParkingRoutineStates Parking::checkParkingStatus(){
	switch(parkingState){
		case INITIALIZE_PARKING:
			parkingState = TURNING_TO_SPACE;
			break;
		case TURNING_TO_SPACE:
			//Serial.println("TURNING TO SPACE");
			// TODO: in the future, outside the scope of this project, the parking needs to know if the space is on the right
			// or the left of the outer lane. For right now, in our current implementation, we assume there is only one lane of parking spots.

			// since this is a turnToHeading, not just a turn -90, this should work regardless of how you approach the spot
			// (assuming the spot is left of the line)
			if(chassis->myChassisPose.getOrientationToClosest90() != -90){
				chassis->turnToHeading(-90, 7500);
				parkingStateAfterMotionSetpointReached = BACKING_UP_TO_OUTER_EDGE;
				parkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING;
			}
			else{
				parkingState = BACKING_UP_TO_OUTER_EDGE;
			}
			break;
		case BACKING_UP_TO_OUTER_EDGE:
			//Serial.println("BACKING UP TO OUTER EDGE");
			chassis->driveStraight(-90, DRIVING_BACKWARDS);
            if(chassis -> lineSensor.onMarker()){
            	// this drive forward is used to get off the outer-edge marker
            	chassis->driveBackwards(30, 1000);
            	parkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_PARKING;
            	parkingStateAfterMotionSetpointReached = BACKING_INTO_SPACE;
            }
			break;
		case BACKING_INTO_SPACE:
			//Serial.println("BACKING INTO SPACE");
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
			//Serial.println("PARKED");
			parkingState = INITIALIZE_PARKING;
			break;
		}
	return parkingState;
}

ExitParkingRoutineStates Parking::getOutOfParkingStatus(){
	switch(exitParkingState){
		case EXIT_PARKING_SPOT:
			// this drive forward is used to get off the line indicating the edge of a parking spot
			chassis->driveForward(30, 1500);
		    exitParkingState = WAIT_FOR_MOTION_SETPOINT_REACHED_EXIT_PARKING;
			exitParkingStateAfterMotionSetpointReached = DRIVE_UP_TO_OUTER_EDGE;
		break;
		// drive up to col 0 (outer edge)
		case DRIVE_UP_TO_OUTER_EDGE:
			chassis->driveStraight(chassis->myChassisPose.currentHeading, DRIVING_FORWARDS);
			// have we hit the outer edge?
			if(chassis -> lineSensor.onMarker()){
				chassis->stop();
				exitParkingState = DRIVE_FORWARD;
		    }
			break;
		case DRIVE_FORWARD:
			// this drive forwards is so that we are in line with the world, wherever we chose to navigate to
			// 130 IS a magic number. Once measurements are confirmed for the new robot, this needs to change into
			// a variable.
			chassis->driveForward(DISTANCE_TO_LINE_SENSOR, 2500);
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



