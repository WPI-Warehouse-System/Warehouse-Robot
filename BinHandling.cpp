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
        binHeight = 16.5; // From middle of cleat in bottom lift position to middle of bin on first shelf
    }
    else{
    	binHeight = 131.65; // From middle of cleat in bottom lift position to middle of bin on second shelf
    }
}

BinProcurementRoutineStates BinHandling::checkBinProcurementStatus(){
	switch(binProcurementState){
		case ALIGN_WITH_BIN:
			chassis->driveForward(56, 1000);
			binProcurementState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT;
			binProcurementStateAfterMotionSetpointReached = TURN_TO_BIN;
			break;
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
				binProcurementState = GRAB_BIN;
			}
			break;
		case GRAB_BIN:
			lift->SetLiftHeight(binHeight + BIN_LIP_OFFSET);
			binProcurementState = WAIT_FOR_LIFT_SETPOINT_REACHED_PROCUREMENT;
			binProcurementStateAfterLiftSetpointReached = BACK_AWAY_FROM_SHELF_PROCUREMENT;
			break;
		case BACK_AWAY_FROM_SHELF_PROCUREMENT:
			chassis->driveBackwards(35, 1000);
			binProcurementState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT;
			binProcurementStateAfterMotionSetpointReached = BACK_UP_TO_WORLD_PROCUREMENT;
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
			binProcurementState = ALIGN_WITH_BIN;
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
		case ALIGN_WITH_SHELF:
			Serial.println("ALIGNING WITH SHELF");
			chassis->driveForward(56, 2000); /// TODO: don't use magic number. Line sensor to COR distance
			binReturnState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN;
			binReturnStateAfterMotionSetpointReached = TURN_TO_SHELF;
			break;
		case TURN_TO_SHELF:
			Serial.println("TURNING TO SHELF");
			chassis->turnToHeading(0, 7500);
			binReturnState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN;
			binReturnStateAfterMotionSetpointReached = RAISE_BIN_TO_SHELF;
			break;
		case RAISE_BIN_TO_SHELF:
			Serial.println("RAISING LIFT");
			lift->SetLiftHeight(binHeight + BIN_LIP_OFFSET);
			binReturnState = WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN;
			binReturnStateAfterLiftSetpointReached = APPROACH_SHELF;
			break;
		case APPROACH_SHELF:
			Serial.println("APPROACHING SHELF");
				// maybe we put in a timeout here? We can see how testing is going
			chassis->driveStraight(0, DRIVING_FORWARDS);
			if(chassis->lineSensor.onMarker()){
				chassis->stop();
				binReturnState = PLACE_BIN_ON_SHELF;
			}
			break;
		case PLACE_BIN_ON_SHELF:
			Serial.println("PLACING BIN ON SHELF");
			lift->SetLiftHeight(binHeight);
			binReturnState = WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN;
			binReturnStateAfterLiftSetpointReached = BACK_AWAY_FROM_SHELF_RETURN;
			break;
		case BACK_AWAY_FROM_SHELF_RETURN:
			Serial.println("BACKING AWAY");
			chassis->driveBackwards(35, 1000);
			binReturnState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN;
			binReturnStateAfterMotionSetpointReached = BACK_UP_TO_WORLD_RETURN;
			break;
		case BACK_UP_TO_WORLD_RETURN:
			Serial.println("GOING TO WORLD");
			chassis->driveStraight(0, DRIVING_BACKWARDS);
			if(chassis->lineSensor.onMarker()){
				// we backed up to the world
				chassis->stop();
//				chassis->driveForward(76, 2000); /// TODO: don't use magic number. Line sensor to COR distance
//				binReturnState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN;
//				binReturnStateAfterMotionSetpointReached = LOWER_LIFT;
				binReturnState = LOWER_LIFT;
			}
			break;
		case LOWER_LIFT:
			lift->SetLiftHeight(0);
			binReturnState = WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN;
			binReturnStateAfterLiftSetpointReached = FINISHED_RETURN;
			break;
		case FINISHED_RETURN:
			binReturnState = ALIGN_WITH_SHELF;
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





