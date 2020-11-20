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

/*
 * @enum NavigationStates
 * States used in the navigation state machine, not the main robot state machine
*/
void Navigation::setNavGoal(int row, int col){
	goalRow = row;
	goalCol = col;
}

NavigationStates Navigation::checkNavStatus(){
	switch(navState){
		case INITIALIZE_NAVIGATION:
			//Serial.println("INIT NAV");
			if((chassis->myChassisPose.currentColumn == goalCol) && (chassis->myChassisPose.currentRow == goalRow)){
				navState = FINISHED_NAVIGATION;
			}
			// are we in the outer lane?
			else if(chassis->myChassisPose.currentColumn == 0){
				navState = TURN_TOWARDS_CORRECT_ROW;
			}
			else{
				navState = TURN_TOWARDS_CORRECT_COLUMN;
			}
			break;
		case TURN_TOWARDS_CORRECT_COLUMN:
			//Serial.println("TURNING TOWARDS COLUMN");
			// determine if we are in the correct row or not
			if(chassis->myChassisPose.currentRow != goalRow){
				// if our current row isn't our goal row, then we need to navigate to the outer edge
				// first. We check to see if we are facing the correct row
			    if(chassis->myChassisPose.getOrientationToClosest90() != 90){
			    	// in our simplified world, there is only one outer edge, and its on the left side.
			    	// In the future, if there are more out edges, this would need to determine which edge is closest
			    	// and turn towards it.
				    chassis->turnToHeading(90, 7500);
					navStateAfterMotionSetpointReached = FINDING_OUTER_EDGE;
					navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
			    }
			    else{
			    	navState = FINDING_OUTER_EDGE;
			    }

			}
		    // if we're in the correct row, we need to find the column
			else{
				// check our orientation first in case we don't need to turn
				if(chassis->myChassisPose.currentColumn > goalCol){
				    if(chassis->myChassisPose.getOrientationToClosest90() != -90){
						chassis->turnToHeading(-90, 7500);
						navStateAfterMotionSetpointReached = FINDING_COLUMN;
						navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
				    }
				    else{
				    	navState = FINDING_COLUMN;
				    }
				}
				else if(chassis->myChassisPose.currentColumn < goalCol){
				    if(chassis->myChassisPose.getOrientationToClosest90() != 90){
						chassis->turnToHeading(90, 7500);
						navStateAfterMotionSetpointReached = FINDING_COLUMN;
						navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
				    }
				    else{
				    	navState = FINDING_COLUMN;
				    }
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
				chassis->lineFollowForwards();
			}
			else{
				chassis->stop();
				// if we're at the right column, stop
				if(chassis->myChassisPose.currentColumn == goalCol){
				    navState = FINISHED_NAVIGATION;
				}
				//otherwise turn towards the right column
				else if (goalCol != 0){
					navState = TURN_TOWARDS_CORRECT_COLUMN;
				}
			}
			break;
		case TURN_TOWARDS_CORRECT_ROW:
			//Serial.println("TURNING TOWARDS ROW");
			//If we're not in the right row, we need to find the right row
			if(chassis->myChassisPose.currentRow > goalRow){
			    if(chassis->myChassisPose.getOrientationToClosest90() != 180){
					chassis->turnToHeading(180, 7500);
					navStateAfterMotionSetpointReached = FINDING_ROW;
					navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
			    }
			    else{
			    	navState = FINDING_ROW;
			    }

			}
			else if(chassis->myChassisPose.currentRow < goalRow){
				if(chassis->myChassisPose.getOrientationToClosest90() != 0){
					chassis->turnToHeading(0, 7500);
					navStateAfterMotionSetpointReached = FINDING_ROW;
					navState = WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION;
				}
				else{
					navState = FINDING_ROW;
				}

			}
			// if we're at the right row
			else{
				navState = TURN_TOWARDS_CORRECT_COLUMN;;
			}
			break;
		case FINDING_COLUMN:
			//Serial.println("FINDING COL: " + String(goalCol) +  "CURRENT COL: " + String(chassis->myChassisPose.currentColumn));
			if(chassis->myChassisPose.currentColumn != goalCol){
				chassis->lineFollowForwards();
			}
			else{
			   chassis->stop();
			   if(goalRow == chassis->myChassisPose.currentRow){
					navState = FINISHED_NAVIGATION;
			   }
			}
			break;
		case WAIT_FOR_MOTION_SETPOINT_REACHED_NAVIGATION:{
			DrivingStatus motionStatus = chassis -> statusOfChassisDriving();

		    if(motionStatus == REACHED_SETPOINT){
			    navState = navStateAfterMotionSetpointReached;
		    }
		    else if(motionStatus == TIMED_OUT){
		    	navState = TIMED_OUT_NAVIGATION;
		    }
		}
            break;
		case FINISHED_NAVIGATION:
			navState = INITIALIZE_NAVIGATION;
			break;
		case TIMED_OUT_NAVIGATION:
			navState = INITIALIZE_NAVIGATION;
			break;
		}
	return navState;
}



