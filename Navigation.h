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
	   Navigation(DrivingChassis* robotChassis, LineFollower* lineFollower);
	   // current row and current column will be updated by the line sensor lineCount
	   int currentRow = 0;
	   int currentColumn = 0;
	   // current heading will be updated by the IMU during turnToHeading.
	   float heading = 0;
	   int rowCount = 0;
	   int colCount = 0;

	   DrivingChassis* drivingChassis;
	   LineFollower* lineSensor;
	   int goalRow;
	   int goalCol;
	   NavigationStates navState = INITIALIZE_NAVIGATION;

	   // This is the navState that occurs after a setpoint has been reached
	   NavigationStates navStateAfterMotionSetpointReached;
	   void setNavGoal(int row, int col);

	   NavigationStates checkNavStatus();

    private:
};



#endif /* NAVIGATIONROUTINE_H_ */
