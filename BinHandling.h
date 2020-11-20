/*
 * BinHandling.h
 *
 *  Created on: Nov 10, 2020
 *      Author: gentov
 */

#include "DrivingChassis.h"
#include "LineFollower.h"
#include "LiftControl.h"

#ifndef BINHANDLING_H_
#define BINHANDLING_H_

#define BIN_LIP_OFFSET 18 // 1.8 cm
#define MAX_RETRIES_PROCUREMENT 5
/*
 * @enum BinProcurementStates
 * States used in the Bin procurement state machine, not the main robot state machine
*/

// TODO: Need to align with bin before turning
enum BinProcurementRoutineStates{
	TURN_TO_BIN,
	RAISE_LIFT_TO_SHELF,
	APPROACH_BIN,
	GRAB_BIN,
	BACK_AWAY_FROM_SHELF_PROCUREMENT,
	BACK_UP_TO_WORLD_PROCUREMENT,
	LOWER_BIN,
	FINISHED_PROCUREMENT,
	WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT,
	WAIT_FOR_LIFT_SETPOINT_REACHED_PROCUREMENT,
	TIMED_OUT_PROCUREMENT,

};

/*
 * @enum BinReturnStates
 * States used in the bin return state machine, not the main robot state machine
*/
enum BinReturnRoutineStates{
	TURN_TO_SHELF,
	RAISE_BIN_TO_SHELF,
	APPROACH_SHELF,
	PLACE_BIN_ON_SHELF,
	BACK_AWAY_FROM_SHELF_RETURN,
	BACK_UP_TO_WORLD_RETURN,
	LOWER_LIFT,
	FINISHED_RETURN,
	WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN,
	WAIT_FOR_LIFT_SETPOINT_REACHED_RETURN,
	TIMED_OUT_RETURN,
};

class BinHandling{
    public:
	   BinHandling(DrivingChassis* robotChassis, LiftControl* robotLift);

		/**
		 * Checks the status of the procurement routine. Called repeatedly in a loop.
		 *
		 * @return the current state in the part procurement process
		 * @note this function is fast-return and should not block
		 */
	   BinProcurementRoutineStates checkBinProcurementStatus();

		/**
		 * Checks the status of the return routine. Called repeatedly in a loop.
		 *
		 * @return the current state in returning process
		 * @note this function is fast-return and should not block
		 */
	   BinReturnRoutineStates checkBinReturnStatus();

	   void setBinHeight(int height);

	   DrivingChassis* chassis = NULL;
	   LiftControl* lift = NULL;

	   BinProcurementRoutineStates binProcurementState = TURN_TO_BIN;

	   // This is the binProcurementState that occurs after a setpoint has been reached
	   BinProcurementRoutineStates binProcurementStateAfterMotionSetpointReached = TURN_TO_BIN;

	   // This is the binProcurementState that occurs after a lift setpoint has been reached
	   BinProcurementRoutineStates binProcurementStateAfterLiftSetpointReached = TURN_TO_BIN;


	   BinReturnRoutineStates binReturnState = TURN_TO_SHELF;

	   // This is the binReturnState that occurs after a setpoint has been reached
	   BinReturnRoutineStates binReturnStateAfterMotionSetpointReached = TURN_TO_SHELF;

	   // This is the binProcurementState that occurs after a lift setpoint has been reached
	   BinReturnRoutineStates binReturnStateAfterLiftSetpointReached = TURN_TO_SHELF;

	   float binHeight = 0;
    private:
};

#endif /* BINHANDLING_H_ */
