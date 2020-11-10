/*
 * BinHandling.cpp
 *
 *  Created on: Nov 10, 2020
 *      Author: gentov
 */

#include "BinHandling.h"

BinHandling::BinHandling(DrivingChassis* robotChassis){
	chassis = robotChassis;
}

BinProcurementStates BinHandling::checkBinProcurementStatus(){
	switch(binProcurementState){
		case TURN_TO_BIN:
			chassis->turnToHeading(0, 7500);
			binProcurementState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT;
			binProcurementStateAfterMotionSetpointReached = RAISE_LIFT_PROCUREMENT;
			break;
		case RAISE_LIFT_PROCUREMENT:
            // TODO: WRITE CODE TO LIFT BIN TO CERTAIN HEIGHT
			// When it's done go to APPROACH_BIN
			break;
		case APPROACH_BIN:
			// we could have it move and command lift at the same time for cool multi-tasking visual,
			// which will remove a state as well

			// maybe we put in a timeout here? We can see how testing is going
			chassis->driveStraight(0, DRIVING_FORWARDS);

			// TODO: Wire limit switch
//			if(digitalRead(CLEAT_LIMIT_SWITCH)){
//				// drive forward another little bit (1.5 cm) to make sure we are pressed against the bin
//			    chassis->driveForward(15, 1000);
//				binProcurementState = WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT;
//				binProcurementStateAfterMotionSetpointReached = LIFT_BIN;
//			}
			break;
		case LIFT_BIN:
			// TODO: WRITE CODE TO LIFT BIN TO A BIT HIGHER THAN CURRENT POSITION
			break;
		case BACK_UP_TO_WORLD_PROCUREMENT:
			chassis->driveStraight(0, DRIVING_BACKWARDS);
			if(chassis->lineSensor.onMarker()){
				// we backed up to the world
				chassis->stop();
				binProcurementState = LOWER_LIFT;
			}
			break;
		case LOWER_LIFT:
			// TODO: WRITE CODE TO LOWER BIN TO CERTAIN HEIGHT
			// when its done go to FINISHED_PROCUREMENT
			break;
		case FINISHED_PROCUREMENT:
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT:
		    if(chassis->statusOfChassisDriving() == REACHED_SETPOINT){
			    binProcurementState = binProcurementStateAfterMotionSetpointReached;
		    }
            break;
		}
	return binProcurementState;
}





