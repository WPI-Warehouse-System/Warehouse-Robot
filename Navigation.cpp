/*
 * NavigationRoutine.cpp
 *
 *  Created on: Oct 22, 2020
 *      Author: gentov
 */
#include "Navigation.h"

Navigation::Navigation(DrivingChassis* robotChassis){
	chassis = robotChassis;
}

void Navigation::setNavGoal(int row, int col){
	goalRow = row;
	goalCol = col;
}

NavigationStates Navigation::checkNavStatus(){
	switch(navState){
		case INITIALIZE_NAVIGATION:
			Serial.println("INIT NAV");
			// if we're in the outerlane, then there really isn't a need to find the outerlane
			if(chassis->myChassisPose.currentColumn == 0){
				navState = TURN_TOWARDS_CORRECT_ROW;
			}
			else{
				navState = TURN_TOWARDS_CORRECT_COLUMN;
			}
			break;
		case TURN_TOWARDS_CORRECT_COLUMN:
			Serial.println("TURNING TOWARDS COLUMN");
			// determine what is our first state
			if(chassis->myChassisPose.currentRow != goalRow){
				// if our current row isn't our goal row, then we need to navigate to the outer edge
				// first
				/// TODO: check where we are, maybe our orientation is correct
				chassis->turnToHeading(90, 7500);
				navStateAfterMotionSetpointReached = FINDING_OUTER_EDGE;
				navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
			}
			else{
				// otherwise, we just need to find the right column
				if(chassis->myChassisPose.currentColumn > goalCol){
					chassis->turnToHeading(-90, 7500);
					navStateAfterMotionSetpointReached = FINDING_COLUMN;
					navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
				}
				else if(chassis->myChassisPose.currentColumn < goalCol){
					chassis->turnToHeading(90, 7500);
					navStateAfterMotionSetpointReached = FINDING_COLUMN;
					navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
				}
				// we're at the right column and row
				else{
					navState = FINISHED_NAVIGATION;
				}
			}
			break;
		case FINDING_OUTER_EDGE:
			//Serial.println("FINDING COL: 0, CURRENT COL: " + String(chassis->myChassisPose.currentColumn));
			// navigate until column == 0
			if(chassis->myChassisPose.currentColumn != 0){
				// if the column is wrong.
				chassis->lineFollowForwards();
			}
			else{
				chassis->stop();
				navState = TURN_TOWARDS_CORRECT_ROW;
			}
			break;
		case FINDING_ROW:
			//Serial.println("FINDING ROW: " + String(goalRow) +  "CURRENT ROW: " + String(chassis->myChassisPose.currentRow));
			if(chassis->myChassisPose.currentRow != goalRow){
				// if the row is wrong.
				chassis->lineFollowForwards();
			}
			else{
				chassis->stop();
				if(chassis->myChassisPose.currentColumn == goalCol){
					navState = FINISHED_NAVIGATION;
				}
				else if (goalCol != 0){
					navState = TURN_TOWARDS_CORRECT_COLUMN;
				}
			}
			break;
		case TURN_TOWARDS_CORRECT_ROW:
			//Serial.println("TURNING TOWARDS ROW");
			// otherwise, we just need to find the right column
			if(chassis->myChassisPose.currentRow > goalRow){
				chassis->turnToHeading(180, 7500);
				navStateAfterMotionSetpointReached = FINDING_ROW;
				navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
			}
			else if(chassis->myChassisPose.currentRow < goalRow){
				chassis->turnToHeading(0, 7500);
				navStateAfterMotionSetpointReached = FINDING_ROW;
				navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
			}
			// if we're at the right row
			else{
				navState = TURN_TOWARDS_CORRECT_COLUMN;
			}
			break;
		case FINDING_COLUMN:
			//Serial.println("FINDING COL: " + String(goalCol) +  "CURRENT COL: " + String(chassis->myChassisPose.currentColumn));
			if(chassis->myChassisPose.currentColumn != goalCol){
				// if the column is wrong.
				chassis->lineFollowForwards();
			}
			else{
			   chassis->stop();
			   if(goalRow == chassis->myChassisPose.currentRow){
			   					navState = FINISHED_NAVIGATION;
			   }
			}
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION:
		    if(chassis-> statusOfChassisDriving() == REACHED_SETPOINT){
			    navState = navStateAfterMotionSetpointReached;
		    }
            break;
		case FINISHED_NAVIGATION:
			navState = INITIALIZE_NAVIGATION;
			break;
		}
	return navState;
}



