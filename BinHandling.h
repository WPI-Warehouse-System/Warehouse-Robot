/*
 * BinHandling.h
 *
 *  Created on: Nov 10, 2020
 *      Author: gentov
 */

#include "DrivingChassis.h"
#include "LineFollower.h"

#ifndef BINHANDLING_H_
#define BINHANDLING_H_

/*
 * @enum BinProcurementStates
 * States used in the Bin procurement state machine, not the main robot state machine
*/
enum BinProcurementStates{
	TURN_TO_BIN,
	RAISE_LIFT_PROCUREMENT,
	APPROACH_BIN,
	LIFT_BIN,
	BACK_UP_TO_WORLD_PROCUREMENT,
	LOWER_LIFT,
	FINISHED_PROCUREMENT,
	WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_PROCUREMENT,
};

/*
 * @enum BinReturnStates
 * States used in the bin return state machine, not the main robot state machine
*/
enum BinReturnStates{
	TURN_TO_SHELF,
	RAISE_LIFT_RETURN,
	APPROACH_SHELF,
	LOWER_BIN,
	// MAYBE ANOTHER STATE TO MAKE SURE ITS PUSHED IN DEEP ENOUGH
	BACK_UP_TO_WORLD_RETURN,
	FINISHED_RETURN,
	WAIT_FOR_MOTION_SETPOINT_REACHED_BIN_RETURN,
};

class BinHandling{
    public:
	   BinHandling(DrivingChassis* robotChassis);

		/**
		 * Checks the status of the procurement routine. Called repeatedly in a loop.
		 *
		 * @return the current state in the part procurement process
		 * @note this function is fast-return and should not block
		 */
	   BinProcurementStates checkBinProcurementStatus();

		/**
		 * Checks the status of the return routine. Called repeatedly in a loop.
		 *
		 * @return the current state in returning process
		 * @note this function is fast-return and should not block
		 */
	   BinReturnStates checkBinReturnStatus();

	   DrivingChassis* chassis = NULL;

	   BinProcurementStates binProcurementState;

	   // This is the binProcurementState that occurs after a setpoint has been reached
	   BinProcurementStates binProcurementStateAfterMotionSetpointReached;


	   BinReturnStates binReturnState;

	   // This is the binReturnState that occurs after a setpoint has been reached
	   BinReturnStates binReturnStateAfterMotionSetpointReached;

    private:
};

#endif /* BINHANDLING_H_ */
