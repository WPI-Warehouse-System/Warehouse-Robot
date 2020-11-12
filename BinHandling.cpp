/*
 * BinHandling.cpp
 *
 *  Created on: Nov 10, 2020
 *      Author: gentov
 */

#include "BinHandling.h"

BinHandling::BinHandling(DrivingChassis* robotChassis, LiftControl* robotLift){
	chassis = robotChassis;
	lift = robotLift;
}


void BinHandling::setBinHeight(int height){
    if(height == 1){
        binHeight = 14.3; // From middle of cleat in bottom lift position to middle of bin on first shelf
    }
    else{
    	binHeight = 131.65; // From middle of cleat in bottom lift position to middle of bin on second shelf
    }
}

BinProcurementRoutineStates BinHandling::checkBinProcurementStatus(){
	switch(binProcurementState){
		case TURN_TO_BIN:
			chassis->turnToHeading(0, 7500);
			binProcurementState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT;
			binProcurementStateAfterMotionSetpointReached = RAISE_LIFT_TO_SHELF;
			break;
		case RAISE_LIFT_TO_SHELF:
			lift->SetLiftHeight(binHeight);
			binProcurementState = WAIT_FOR_LIFT_SETPOINT_REACHED_PROCUREMENT;
			binProcurementStateAfterLiftSetpointReached = APPROACH_BIN;
			break;
		case APPROACH_BIN:
			// maybe we put in a timeout here? We can see how testing is going
			chassis->driveStraight(0, DRIVING_FORWARDS);
			if(!digitalRead(CLEAT_LIMIT_SWITCH)){
				chassis->stop();
				// drive forward another little bit (1 cm) to make sure we are pressed against the bin
			    chassis->driveForward(1, 1000);
				binProcurementState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT;
				binProcurementStateAfterMotionSetpointReached = GRAB_BIN;
			}
			break;
		case GRAB_BIN:
			lift->SetLiftHeight(binHeight + BIN_LIP_OFFSET);
			binProcurementState = WAIT_FOR_LIFT_SETPOINT_REACHED_PROCUREMENT;
			binProcurementStateAfterLiftSetpointReached = BACK_UP_TO_WORLD_PROCUREMENT;
			break;
		case BACK_UP_TO_WORLD_PROCUREMENT:
			chassis->driveStraight(0, DRIVING_BACKWARDS);
			if(chassis->lineSensor.onMarker()){
				// we backed up to the world
				chassis->stop();
				binProcurementState = LOWER_BIN;
			}
			break;
		case LOWER_LIFT:
			lift->SetLiftHeight(0);
			binProcurementState = WAIT_FOR_LIFT_SETPOINT_REACHED_PROCUREMENT;
			binProcurementStateAfterLiftSetpointReached = FINISHED_PROCUREMENT;
			break;
		case FINISHED_PROCUREMENT:
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT:
		    if(chassis->statusOfChassisDriving() == REACHED_SETPOINT){
			    binProcurementState = binProcurementStateAfterMotionSetpointReached;
		    }
            break;
	    case WAIT_FOR_LIFT_SETPOINT_REACHED_PROCUREMENT:
			 if(lift->CheckIfPositionReached()){
				    binProcurementState = binProcurementStateAfterLiftSetpointReached;
			  }
	          break;
		}
	return binProcurementState;
}

BinReturnRoutineStates BinHandling::checkBinReturnStatus(){
	switch(binReturnState){
		case TURN_TO_SHELF:
			chassis->turnToHeading(0, 7500);
			binReturnState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN;
			binReturnStateAfterMotionSetpointReached = RAISE_BIN_TO_SHELF;
			break;
		case RAISE_BIN_TO_SHELF:
			lift->SetLiftHeight(binHeight + BIN_LIP_OFFSET);
			binReturnState = WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN;
			binReturnStateAfterLiftSetpointReached = APPROACH_SHELF;
			break;
		case APPROACH_SHELF:
			// maybe we put in a timeout here? We can see how testing is going
			chassis->driveStraight(0, DRIVING_FORWARDS);
			if(chassis->lineSensor.onMarker()){
				chassis->stop();
				// drive forward another little bit (1 cm) to make sure we are in the shelf
			    chassis->driveForward(1, 1000);
				binReturnState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN;
				binReturnStateAfterMotionSetpointReached = PLACE_BIN_ON_SHELF;
			}
			break;
		case PLACE_BIN_ON_SHELF:
			lift->SetLiftHeight(binHeight);
			binReturnState = WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN;
			binReturnStateAfterLiftSetpointReached = BACK_UP_TO_WORLD_RETURN;
			break;
		case BACK_UP_TO_WORLD_RETURN:
			chassis->driveStraight(0, DRIVING_BACKWARDS);
			if(chassis->lineSensor.onMarker()){
				// we backed up to the world
				chassis->stop();
				binReturnState = LOWER_LIFT;
			}
			break;
		case LOWER_LIFT:
			lift->SetLiftHeight(0);
			binReturnState = WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN;
			binReturnStateAfterLiftSetpointReached = FINISHED_RETURN;
			break;
		case FINISHED_RETURN:
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN:
		    if(chassis->statusOfChassisDriving() == REACHED_SETPOINT){
			    binReturnState = binReturnStateAfterMotionSetpointReached;
		    }
            break;
	    case WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN:
			 if(lift->CheckIfPositionReached()){
				    binReturnState = binReturnStateAfterLiftSetpointReached;
			  }
	          break;
		}
	return binReturnState;
}





