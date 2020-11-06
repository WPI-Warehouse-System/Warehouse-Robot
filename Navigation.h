/*
 * NavigationRoutine.h
 *
 *  Created on: Oct 22, 2020
 *      Author: gentov
 */

#include "DrivingChassis.h"
#include "LineFollower.h"

#ifndef NAVIGATIONROUTINE_H_
#define NAVIGATIONROUTINE_H_
/*
 * @enum NavigationStates
 * States used in the navigation state machine, not the main robot state machine
*/
enum NavigationStates{
	INITIALIZE_NAVIGATION = 0,
	TURN_TOWARDS_CORRECT_COLUMN = 1,
	FINDING_OUTER_EDGE = 2,
	FINDING_ROW = 3,
	TURN_TOWARDS_CORRECT_ROW = 4,
	FINDING_COLUMN = 5,
	WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION = 6,
	FINISHED_NAVIGATION = 7,
};

class Navigation{
    public:
	   Navigation(DrivingChassis* robotChassis);
	   // current row and current column will be updated by the line sensor lineCount
	   int currentRow = 0;
	   int currentColumn = 0;

	   // current heading will be updated by the IMU during turnToHeading.
	   float heading = 0;

	   // because multiple row/column markers exist, row and column count are not 1:1 with
	   // current row/column
	   int rowCount = 0;
	   int colCount = 0;

	   DrivingChassis* chassis = NULL;
	   int goalRow = 0;
	   int goalCol = 0;
	   NavigationStates navState = INITIALIZE_NAVIGATION;

	   // This is the navState that occurs after a setpoint has been reached
	   // Example: set the robot to turn to -90. After we turn we want to go to state: FINDING_COLUMN
	   // However, before we can set the state to FINDING_COLUMN, we must wait for the turn to finish. Therefore,
	   // after setting the turn, our state becomes WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION, before becoming
	   // FINDING_COLUMN. navStateAfterMotionSetpointReached becomes FINDING_COLUMN in this case
	   NavigationStates navStateAfterMotionSetpointReached = INITIALIZE_NAVIGATION;

		/**
		 * Sets a new navigation goal
		 *
		 * @param row is the goal row
		 * @param col is the goal col
		 */
	   void setNavGoal(int row, int col);

		/**
		 * Checks the status of the navigation routine. Called repeatedly in a loop.
		 *
		 * @return the current state in NavigationStates
		 * @note this function is fast-return and should not block
		 */
	   NavigationStates checkNavStatus();

    private:
};



#endif /* NAVIGATIONROUTINE_H_ */
