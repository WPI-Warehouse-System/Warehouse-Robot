/*
 * DrivingChassis.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: hephaestus
 */

#include "DrivingChassis.h"


/**
 * Compute a delta in wheel angle to traverse a specific distance
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param distance a distance for this wheel to travel in MM
 * @return the wheel angle delta in degrees
 */
float DrivingChassis::distanceToWheelAngle(float distance) {
	return 0;
}

/**
 * Compute the arch length distance the wheel needs to travel through to rotate the base
 * through a given number of degrees.
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param angle is the angle the base should be rotated by
 * @return is the linear distance the wheel needs to travel given the this CHassis's wheel track
 */
float DrivingChassis::chassisRotationToWheelDistance(float angle) {
	return 0;
}

DrivingChassis::~DrivingChassis() {
	// do nothing
}

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * @param left the left motor
 * @param right the right motor
 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
 * @param imu The object that is used to access the IMU data
 *
 */
DrivingChassis::DrivingChassis(PIDMotor * left, PIDMotor * right,
		float wheelTrackMM, float wheelRadiusMM,GetIMU * imu) {
	myleft = left;
	myright = right;
	mywheelTrackMM = wheelTrackMM;
	mywheelRadiusMM = wheelRadiusMM;
	IMU = imu;

}

/**
 * Start a drive forward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 * @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveForwardFromInterpolation(float mmDistanceFromCurrent, int msDuration) {
	// We should also have a "drive straight" method that uses the IMU to actually drive straight
	// Although we'll have a linefollow state so... idk maybe not

	myleft -> overrideCurrentPosition(0);
	myright -> overrideCurrentPosition(0);
	myleft -> startInterpolationDegrees(mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
    myright -> startInterpolationDegrees(-mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
}

void DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration){
	// if we're not performing an action
	// start a timer, reset encoders
    startTimeOfMovement_ms = millis();
	myleft -> overrideCurrentPosition(0);
	myright -> overrideCurrentPosition(0);
	motionSetpoint = mmDistanceFromCurrent;
	timeout_ms = msDuration;
	motionType = DRIVING_FORWARDS;
}

/**
	 * Start a drive backwards action
	 *
	 * @param mmDistanceFromCurrent is the distance the mobile base should drive backwards
	 * @param msDuration is the time in miliseconds that the drive action should take
	 *
	 * @note this function is fast-return and should not block
*/
void DrivingChassis::driveBackwardsFromInterpolation(float mmDistanceFromCurrent, int msDuration) {
	// We should also have a "drive straight" method that uses the IMU to actually drive straight
	// Although we'll have a linefollow state so... idk maybe not
	myleft -> overrideCurrentPosition(0);
	myright -> overrideCurrentPosition(0);

	myleft -> startInterpolationDegrees(-mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
    myright -> startInterpolationDegrees(mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
}

/**
 * Start a drive backward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive backward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 * @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveBackwards(float mmDistanceFromCurrent, int msDuration){
		// if we're not performing an action
		// start a timer, reset encoders
	    startTimeOfMovement_ms = millis();
		myleft -> overrideCurrentPosition(0);
		myright -> overrideCurrentPosition(0);
		motionSetpoint = mmDistanceFromCurrent;
		timeout_ms = msDuration;
		motionType = DRIVING_BACKWARDS;
}

/**
 * Start a turn action
 *
 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
 * Positive angles should rotate to the left
 *
 * This rotation is a positive rotation about the Z axis of the robot.
 *
 * @param degreesToRotateBase the number of degrees to rotate
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 *  @note this function is fast-return and should not block
 *  @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		  allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::turnDegreesFromInterpolation(float degreesToRotateBase, int msDuration) {
	   myleft -> overrideCurrentPosition(0);
	   myright -> overrideCurrentPosition(0);
	   myleft -> startInterpolationDegrees(degreesToRotateBase * WHEEL_DEGREES_TO_BODY_DEGREES, msDuration, LIN);
	   myright -> startInterpolationDegrees(degreesToRotateBase * WHEEL_DEGREES_TO_BODY_DEGREES, msDuration, LIN);
}

void DrivingChassis::turnToHeading(float degreesToRotateBase, int msDuration){
	motionSetpoint = degreesToRotateBase;
	timeout_ms = msDuration;
	startTimeOfMovement_ms = millis();
	motionType = TURNING;
}

/**
 * Check to see if the chassis is performing an action
 *
 * @return drivingStatus depending on where we are in the motion
 *
 * @note this function is fast-return and should not block
 */
DrivingStatus DrivingChassis::statusOfChassisDriving() {
	switch(motionType){

	    case TURNING:{
			 // check for timeout
			 if((millis() - startTimeOfMovement_ms) > timeout_ms){
					//timeout occured. Stop the robot
					Serial.println("Detected Timeout on Turn\r\n");
					//Serial.println("CURRENT HEADING: " + String(myChassisPose.currentHeading));
					stop();
					adjustedHeading = false;
					return TIMED_OUT;
			   }

			float currentHeading = myChassisPose.initialHeading - IMU->getWrappedAzimuth();
			// wrap-around for currentHeading
			if(fabs(currentHeading) > 360){
				if(currentHeading > 360){
					currentHeading -= 360;
				}
				else{
					currentHeading += 360;
				}
			}
			float headingError = motionSetpoint - currentHeading;

			// we don't want to to make long turns
			if(fabs(headingError) > 180){
				if(headingError > 180){
					headingError -= 360;
				}
				else{
					headingError += 360;
				}
			}
			float motorEffort = (turningMovementKp) * headingError;
			//myChassisPose.heading = -currentHeading; // - to account for what is considered a "positive" rotation
			myChassisPose.currentHeading = currentHeading;
			if(fabs(headingError) <= wheelMovementDeadband_deg)
			{
				//Serial.println("Reached Setpoint\r\n");
				stop();
				adjustedHeading = false;
				return REACHED_SETPOINT;
			}
			else{
					if(fabs(motorEffort) > MAX_MOTOR_EFFORT_DURING_TURN)
					{
						if(motorEffort < 0){
							motorEffort = -MAX_MOTOR_EFFORT_DURING_TURN;
						}
						else if(motorEffort > 0){
							motorEffort = MAX_MOTOR_EFFORT_DURING_TURN;
						}
					}
					   myleft->setVelocityDegreesPerSecond(motorEffort);
					   myright->setVelocityDegreesPerSecond(motorEffort);
			}
	    }
		    break;

	    case DRIVING_FORWARDS:{
	    		// check for timeout
	    		if((millis() - startTimeOfMovement_ms) > timeout_ms){
	    			//timeout occured. Stop the robot
	    			Serial.println("Detected Timeout on Forwards\r\n");
	    			stop();
	    			return TIMED_OUT;
	    		}

	    	    if(motionSetpoint != -1){
	    	    	float currentDistanceMovedRightWheel_mm = (myright -> getAngleDegrees())*WHEEL_DEGREES_TO_MM;
	    	    	float rightWheelError_mm = currentDistanceMovedRightWheel_mm - motionSetpoint;
	    	    	driveStraight(myChassisPose.currentHeading, DRIVING_FORWARDS);
	    	    	if((fabs(rightWheelError_mm) < wheelMovementDeadband_mm)){
						//Serial.println("Reached Setpoint \r\n");
						stop();
						return REACHED_SETPOINT;
					}
	    	    }

	    	    else{
	    	    	// sets speed to 20 cm per second
	    	    	driveStraight(myChassisPose.currentHeading, DRIVING_FORWARDS);
	    	    }
	        }
	        break;

	    case DRIVING_BACKWARDS:{
	    	// check for timeout
	    		if((millis() - startTimeOfMovement_ms) > timeout_ms){
	    			//timeout occured. Stop the robot
	    			Serial.println("Detected Timeout on Backwards\r\n");
	    			stop();
	    			return TIMED_OUT;
	    		}

	    	    if(motionSetpoint != -1){
	    	    	float currentDistanceMovedRightWheel_mm = (myright -> getAngleDegrees())*WHEEL_DEGREES_TO_MM;
	    	    	float rightWheelError_mm = - currentDistanceMovedRightWheel_mm - motionSetpoint;
	    	    	driveStraight(myChassisPose.currentHeading, DRIVING_BACKWARDS);
	    	    	if((fabs(rightWheelError_mm) < wheelMovementDeadband_mm)){
						//Serial.println("Reached Setpoint \r\n");
						stop();
						return REACHED_SETPOINT;
					}
	    	    }

	    	    else{
	    	    	// sets speed to 20 cm per second
	    	    	driveStraight(myChassisPose.currentHeading, DRIVING_BACKWARDS);
	    	    }
	        }
	        break;

	    default:
	    	 break;
	}
	return GOING_TO_SETPOINT;
}

/**
 * stop the robot
 */
void DrivingChassis::stop(){
	myleft -> stop();
	myright -> stop();
}

/**
 * drive straight, forward or backward
 *
 * @param targetHeading heading to maintain in degrees
 * @param direction is the direction we are moving, forward or backward
 *
 *  @note this function is fast-return and should not block
 *  @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		  allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveStraight(float targetHeading, MotionType direction){
	float currentHeading = myChassisPose.initialHeading - IMU->getWrappedAzimuth();
	if(fabs(currentHeading) > 360){
		if(currentHeading > 360){
			currentHeading -= 360;
		}
		else{
			currentHeading += 360;
		}
	}
	float headingError = targetHeading - currentHeading;
	if(fabs(headingError) > 180){

		if(headingError > 180){
			headingError -= 360;
		}
		else{
			headingError += 360;
		}
	}
	float motorEffort = (turningMovementKp) * .75 * headingError;

	if(direction == DRIVING_BACKWARDS){
		myleft->setVelocityDegreesPerSecond((MAX_SPEED_MM_PER_SEC + motorEffort)*MM_TO_WHEEL_DEGREES);
		myright->setVelocityDegreesPerSecond((-MAX_SPEED_MM_PER_SEC + motorEffort)*MM_TO_WHEEL_DEGREES);
	}

	else if(direction == DRIVING_FORWARDS){
		myright->setVelocityDegreesPerSecond((MAX_SPEED_MM_PER_SEC + motorEffort)*MM_TO_WHEEL_DEGREES);
	    myleft->setVelocityDegreesPerSecond((-MAX_SPEED_MM_PER_SEC + motorEffort)*MM_TO_WHEEL_DEGREES);
	}
}

/**
 * line follow in the backwards direction, if the line follower is mounted to the back of the robot
 */
void DrivingChassis::lineFollowBackwards(){
	  int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
	  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
	  float leftCorrection = 0;
	  float rightCorrection = 0;
	  if(leftSensorValue >= lineSensor.ON_BLACK && rightSensorValue>= lineSensor.ON_BLACK)
	  {
	   if(lineSensor.canCountLine){
		   lineSensor.lineCount++;
	      // Mathematically speaking, this should only increment one of the following. Either
	      // row or column. Since there are two markers for each row, we need to only count once every two markers.
		  int ordinalDirection_degrees = myChassisPose.getOrientationToClosest90();

		  // we need to count rows
		  if((ordinalDirection_degrees == 180) || (ordinalDirection_degrees == 0)){
			  myChassisPose.rowCount += 1;
		      if(myChassisPose.rowCount == 2){
		    	  if(ordinalDirection_degrees == 180)
		              myChassisPose.currentRow -= 1;
		    	  else
		    		  myChassisPose.currentRow += 1;
		          myChassisPose.rowCount = 0;
		      }
		  }

		  // we need to count columns
		  else if((ordinalDirection_degrees == 90) || (ordinalDirection_degrees == -90)){
			  myChassisPose.currentColumn += ordinalDirection_degrees/90;
		  }

	      lineSensor.canCountLine = false; // This is meant as a line "debouncing". We don't want to catch the same line twice.
	    }
	    //Serial.println("Line Count: " + String(lineCount));
	  }


	  else if(leftSensorValue >= lineSensor.ON_BLACK || rightSensorValue >= lineSensor.ON_BLACK){
			rightCorrection = (lineSensor.ON_BLACK - rightSensorValue)*lineSensor.lineFollowingKpForwards;
			leftCorrection =  (leftSensorValue - lineSensor.ON_BLACK)*lineSensor.lineFollowingKpForwards;
			lineSensor.canCountLine = true;
	  }
	  else{
		  lineSensor.canCountLine = true;
	  }
	  myleft -> setVelocityDegreesPerSecond(lineSensor.lineFollowingSpeedBackwards_mm_per_sec*MM_TO_WHEEL_DEGREES + leftCorrection);
      myright -> setVelocityDegreesPerSecond(-lineSensor.lineFollowingSpeedForwards_mm_per_sec*MM_TO_WHEEL_DEGREES + rightCorrection);
}

/**
 * line follow in the forwards direction, if the line follower is mounted to the front of the robot
 */
void DrivingChassis::lineFollowForwards(){
	  int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
	  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
	  float leftCorrection = 0;
	  float rightCorrection = 0;
	  if(leftSensorValue >= lineSensor.ON_BLACK && rightSensorValue>= lineSensor.ON_BLACK)
	  {
	   if(lineSensor.canCountLine){
		   lineSensor.lineCount++;
	      // Mathematically speaking, this should only increment one of the following. Either
	      // row or column. Since there are two markers for each row, we need to only count once every two markers.
		  int ordinalDirection_degrees = myChassisPose.getOrientationToClosest90();

		  // we need to count rows
		  if((ordinalDirection_degrees == 180) || (ordinalDirection_degrees == 0)){
			  myChassisPose.rowCount += 1;
		      if(myChassisPose.rowCount == 2){
		    	  if(ordinalDirection_degrees == 180)
		              myChassisPose.currentRow -= 1;
		    	  else
		    		  myChassisPose.currentRow += 1;
		          myChassisPose.rowCount = 0;
		      }
		  }

		  // we need to count columns
		  else if((ordinalDirection_degrees == 90) || (ordinalDirection_degrees == -90)){
			  myChassisPose.currentColumn += ordinalDirection_degrees/90;
		  }

	      lineSensor.canCountLine = false; // This is meant as a line "debouncing". We don't want to catch the same line twice.
	    }
	    //Serial.println("Line Count: " + String(lineCount));
	  }


	  else if(leftSensorValue >= lineSensor.ON_BLACK || rightSensorValue >= lineSensor.ON_BLACK){
			rightCorrection = (lineSensor.ON_BLACK - rightSensorValue)*lineSensor.lineFollowingKpForwards;
			leftCorrection =  (leftSensorValue - lineSensor.ON_BLACK)*lineSensor.lineFollowingKpForwards;
			lineSensor.canCountLine = true;
	  }
	  else{
		  lineSensor.canCountLine = true;
	  }
	  myleft -> setVelocityDegreesPerSecond(-lineSensor.lineFollowingSpeedForwards_mm_per_sec*MM_TO_WHEEL_DEGREES + leftCorrection);
      myright -> setVelocityDegreesPerSecond(lineSensor.lineFollowingSpeedForwards_mm_per_sec*MM_TO_WHEEL_DEGREES + rightCorrection);
}
/**
 *
 * loop()
 *
 * a fast loop function that will update states of the motors based on the information from the
 * imu.
 */
bool DrivingChassis::loop(){
return false;
}
