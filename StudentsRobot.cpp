/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */


#include "StudentsRobot.h"

uint32_t startTime = 0;

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servo, IRCamSimplePacketComsServer * IRCam,
		GetIMU * imu): robotChassis(motor2, motor1, 230, 30, imu),
				       navigation(&robotChassis), parking(&robotChassis), Lift(motor3), binHandler(&robotChassis, &Lift) {
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");

	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;
	IRCamera = IRCam;
	IMU = imu;

#if defined(USE_IMU)
	IMU->setXPosition(200);
	IMU->setYPosition(0);
	IMU->setZPosition(0);
#endif
	// Set the PID Clock gating rate. The PID must be 10 times slower than the motors update rate
	motor1->myPID.sampleRateMs = 5; //
	motor2->myPID.sampleRateMs = 5; //
	motor3->myPID.sampleRateMs = 5;  // 10khz H-Bridge, 0.1ms update, 1 ms PID

	// Set default P.I.D gains
	motor1->myPID.setpid(0.00015, 0, 0);
	motor2->myPID.setpid(0.00015, 0, 0);
	motor3->myPID.setpid(0.00015, 0, 0);

	motor1->velocityPID.setpid(0.1, 0, 0);
	motor2->velocityPID.setpid(0.1, 0, 0);
	motor3->velocityPID.setpid(0.1, 0, 0);
	// compute ratios and bounding
	double motorToWheel = 3;
	motor1->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor2->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150	// the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			64.0 * // Encoder CPR
					6.3 * // Motor Gear box ratio
					1.0 * // motor to arm stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			4800, // measured max degrees per second
			50 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	// Set up the Analog sensors
	pinMode(LEFT_LINE_SENSOR, ANALOG);
	pinMode(RIGHT_LINE_SENSOR, ANALOG);
	pinMode(LEFT_LINE_DETECT, ANALOG);
	pinMode(RIGHT_LINE_DETECT, ANALOG);
	// H-Bridge enable pin
	pinMode(H_BRIDGE_ENABLE, OUTPUT);
	// Stepper pins
	pinMode(BOTTOM_OPTICALSWITCH, INPUT);
	pinMode(TOP_OPTICALSWITCH, INPUT);
	pinMode(CLEAT_LIMIT_SWITCH, INPUT_PULLUP);
	// User button
	pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
	//Test IO
	//pinMode(WII_CONTROLLER_DETECT, OUTPUT);
}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */
void StudentsRobot::updateStateMachine() {
//	digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();
	//Serial.println("Limit Switches");
	//Serial.println(digitalRead(CLEAT_LIMIT_SWITCH));
	//Serial.println(digitalRead(BOTTOM_OPTICALSWITCH));
	//Serial.println(digitalRead(TOP_OPTICALSWITCH));
	//delay(100);

	if(lastStatus != status){
	    Serial.println("STATUS IS: " + String(StringStates[status]));
	    lastStatus = status;
	}

	switch (status) {
	case StartupRobot:
		//Do this once at startup
		status = StartRunning;
		Serial.println("StudentsRobot::updateStateMachine StartupRobot here ");
		break;
	case StartRunning:
		Serial.println("Start Running");

		digitalWrite(H_BRIDGE_ENABLE, 1);
		// Start an interpolation of the motors
		motor1->startInterpolationDegrees(motor1->getAngleDegrees(), 1000, SIN);
		motor2->startInterpolationDegrees(motor2->getAngleDegrees(), 1000, SIN);
		motor3->startInterpolationDegrees(motor3->getAngleDegrees(), 1000, SIN);
		status = WAIT_FOR_MOTORS_TO_FINNISH; // set the state machine to wait for the motors to finish
		nextStatus = Running; //HomingLift; // the next status to move to when the motors finish
		startTime = now + 1000; // the motors should be done in 1000 ms
		nextTime = startTime + 1000; // the next timer loop should be 1000ms after the motors stop
		break;
	case Running:
		// Set up a non-blocking 1000 ms delay
//		status = WAIT_FOR_TIME;
//		nextTime = nextTime + 1; // ensure no timer drift by incremeting the target
//		// After 1000 ms, come back to this state
//		nextStatus = Running;
		// Do something
		// On button press we go into testing state
		if (!digitalRead(BOOT_FLAG_PIN)) {
			Serial.println(
					" Running State Machine " + String((now - startTime)));
			//robotChassis.turnDegrees(-90, 5000);
			//robotChassis.driveForward(100, 5000);
			//robotChassis.driveBackwards(300, 5000);
#if defined(USE_IMU)
			IMU->print();
#endif
#if defined(USE_IR_CAM)
			IRCamera->print();
#endif
		// I put in this delay so that I have time to step back
		//status = WAIT_FOR_TIME;
		//nextTime = millis() + 3000; // ensure no timer drift by incremeting the target
		// After 1000 ms, come back to this state
		//nextStatus = TestingBasicMovement;
	    startTime = millis();
		status = Testing;
		}
		break;
	case WAIT_FOR_TIME:
		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
		break;
	case WAIT_FOR_MOTORS_TO_FINNISH:
		if (motor1->isInterpolationDone() && motor2->isInterpolationDone()
				&& motor3->isInterpolationDone()) {
			status = nextStatus;
		}
		break;
	case Halting:
		// save state and enter safe mode
		Serial.println("Halting State machine");
		digitalWrite(H_BRIDGE_ENABLE, 0);
		motor3->stop();
		motor2->stop();
		motor1->stop();

		status = Halt;
		break;
	case Halt:
		// in safe mode
		break;

	case Navigating:
		switch(navigationStatus){
		    case SETTING_NAV_GOAL:
		    	Serial.println("SETTING NAV GOAL TO: " + String(goalRow) + " " + String(goalColumn));
			    navigation.setNavGoal(goalRow, goalColumn);
			    navigationStatus = CHECKING_IF_PARKED;
			    break;

			case CHECKING_IF_PARKED:
				Serial.println("CHECKING IF PARKED");
				if(robotParked){
				    Serial.println("PARKED");
					navigationStatus = LEAVING_PARKING_SPOT;
				}
				else{
					Serial.println("NOT PARKED");
					navigationStatus = NAVIGATING;
				}
				break;

			case LEAVING_PARKING_SPOT:
				if(parking.getOutOfParkingStatus() == FINISHED_EXIT_PARKING){
					navigationStatus = NAVIGATING;
					robotParked = false;
					Serial.println("LEFT PARKING SPOT");
					//Serial.println("NAVIGATING");
				}
				break;

			case NAVIGATING:
				if(navigation.checkNavStatus() == FINISHED_NAVIGATION){
					navigationStatus = SETTING_NAV_GOAL;
					Serial.println("FINISHED NAVIGATION");
					status = statusAfterNav;
				}
				break;
			}
//		if(navigation.checkNavStatus() == FINISHED_NAVIGATION){
//			status = statusAfterNav;
//		}
		break;

	case ParkingRobot:
	    switch(parkingStatus){
	    case SETTING_PARKING_GOAL:
	    	parkingStatus = GOING_TO_PARKING_SPACE;
	    	navigation.setNavGoal(goalRow, goalColumn);
	    	break;
	    case GOING_TO_PARKING_SPACE:
	    	status = Navigating;
	    	parkingStatus = PARKING;
	    	statusAfterNav = ParkingRobot;
	       break;
	    case PARKING:
	    	if(parking.checkParkingStatus() == FINISHED_PARKING){
		    	parkingStatus = SETTING_PARKING_GOAL;
		    	status = Running;
		    	statusAfterNav = Running;
                robotParked = true;
	    	}
	    	break;
	    }
		break;

	case DeliveringBin:
		switch(binDeliveryStatus){
			case SETTING_DELIVERY_LOCATION:
		    	binDeliveryStatus = GOING_TO_BIN;
		    	navigation.setNavGoal(goalRow, goalColumn);
		    	binHandler.setBinHeight(goalShelf);
				break;
			case GOING_TO_BIN:
		    	status = Navigating;
		    	binDeliveryStatus = PROCURING_BIN;
		    	statusAfterNav = DeliveringBin;
				break;
			case PROCURING_BIN:
				if(binHandler.checkBinProcurementStatus() == FINISHED_PROCUREMENT){
			    	binDeliveryStatus = GOING_TO_USER;
			    	navigation.setNavGoal(0, 0); // replace with coordinates of designated drop off
			    	status = Navigating;
			    	statusAfterNav = DeliveringBin;
				}
				break;
			case GOING_TO_USER:
				status = Running;
				// TODO: send communication to GUI that bin is delivered
				binDeliveryStatus = SETTING_DELIVERY_LOCATION;
				break;
		}
		break;

	case ReturningBin:
		switch(binReturnStatus){
			case SETTING_RETURN_LOCATION:
				Serial.println("GOT NEW RETURN COMMAND");
				binReturnStatus = GOING_TO_SHELF;
			    navigation.setNavGoal(goalRow, goalColumn);
			    binHandler.setBinHeight(goalShelf);
				break;
			case GOING_TO_SHELF:
		    	status = Navigating;
		    	binReturnStatus = RETURNING_BIN;
		    	statusAfterNav = ReturningBin;
				break;
			case RETURNING_BIN:
				if(binHandler.checkBinReturnStatus() == FINISHED_RETURN){
			    	binReturnStatus = SETTING_RETURN_LOCATION;
			    	// TODO: send communication to GUI that bin is returned
					Serial.println("FINISHED RETURN, GOING TO RUNNING");
			    	status = Running;
				}
				break;
		}
	break;

	case HomingLift:
		switch(homeLiftState){
		case STARTING_HOME:
			Lift.StartHomeDown();
			homeLiftState = MOVING_TO_LOWER_LIMIT;
			break;
		case MOVING_TO_LOWER_LIMIT:
			if(Lift.CheckIfAtBottom()){
				Lift.StartHomeUp();
				homeLiftState = MOVING_TO_UPPER_LIMIT;
			}
			break;
		case MOVING_TO_UPPER_LIMIT:
			if(Lift.CheckIfAtTop()){
				liftHeight = 0;
				moveLiftState = SET_LIFT_HEIGHT;
				homeLiftState = DONE_HOMING;
			}
			break;
		case DONE_HOMING:
			status = MovingLiftFromGUI;
			Serial.println("Done Homing");
			break;
		}
	break;

	case MovingLiftFromGUI:
		switch(moveLiftState){
		case SET_LIFT_HEIGHT:
			if(Lift.SetLiftHeight(liftHeight)){//if the lift is not homed this will not run
				moveLiftState = WAIT_FOR_HEIGHT_REACHED;
				Serial.println("Homed confirmed");
			}
			else{
				moveLiftState = DONE_LIFTING;
				Serial.println("Not Homed");
			}
		break;
		case WAIT_FOR_HEIGHT_REACHED:
			if(Lift.CheckIfPositionReached()){
				moveLiftState= DONE_LIFTING;
			}
		break;
		case DONE_LIFTING:
			Serial.println("Done Move");
			status = Running;
		break;
		}
	break;

	case Testing:
		//myCommandsStatus = Ready_for_new_task;
		//status = Running;
// LINE FOLLOWING
	    if((millis() - startTime) < 7000){
			//robotChassis.lineFollowForwards();
	    	robotChassis.lineSensor.calibrate();
		}
		else{
		   robotChassis.stop();
		   robotChassis.lineSensor.resetLineCount();
		   status = Running;
		}

// Line Centering
//	if(robotChassis.isCenteredOnLine()){
//		   robotChassis.stop();
//		   robotChassis.lineSensor.resetLineCount();
//		   status = Running;
//	}

// Bin Return
//		goalRow = 2;
//		goalColumn = -1;
//		goalShelf = 2;
//		status = ReturningBin;
// Navigation
//
//		static int myCase = 1;
//		static int myCaseAfterNav = 2;
//		switch(myCase){
//			case 1:
//				// set a waypoint
//				navigation.setNavGoal(2, 0);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 2;
//				// set the state
//				myCase = 10;
//				 break;
//			case 2:
//                // set a waypoint
//				navigation.setNavGoal(2, -1);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 3;
//				// set the state
//				myCase = 10;
//				 break;
//			case 3:
//                // set a waypoint
//				navigation.setNavGoal(2, -2);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 4;
//				// set the state
//				myCase = 10;
//				 break;
//			case 4:
//                // set a waypoint
//				navigation.setNavGoal(2, -1);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 5;
//				// set the state
//				myCase = 10;
//				break;
//			case 5:
//                // set a waypoint
//				navigation.setNavGoal(2, 0);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 6;
//				// set the state
//				myCase = 10;
//				break;
//			case 6:
//                // set a waypoint
//				navigation.setNavGoal(1, 0);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 7;
//				// set the state
//				myCase = 10;
//				break;
//			case 7:
//                // set a waypoint
//				navigation.setNavGoal(1, -1);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 8;
//				// set the state
//				myCase = 10;
//				break;
//			case 8:
//                // set a waypoint
//				navigation.setNavGoal(1, -2);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 9;
//				// set the state
//				myCase = 10;
//				break;
//			case 9:
//				 myCase = 1;
//				 status = Running;
//				 break;
//			case 10:
//				 if(navigation.checkNavStatus() == FINISHED_NAVIGATION){
//					 myCase = myCaseAfterNav;
//				 }
//				 break;
//		}
// PARKING

// working
//		static int myCase = 1;
//		static int myCaseAfterNav = 2;
//		switch(myCase){
//			case 1:
//				// set a waypoint
//				navigation.setNavGoal(2, 0);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 2;
//				// set the state
//				myCase = 5;
//				 break;
//			case 2:
//                 if(parking.checkParkingStatus() == FINISHED_PARKING){
//                	 myCase = 3;
//                 }
//				 break;
//			case 3:
//				if(parking.getOutOfParkingStatus() == FINISHED_EXIT_PARKING){
//				     myCase = 4;
//				 }
//				 break;
//			case 4:
//				// set a waypoint
//				navigation.setNavGoal(1, -2);
//				// set the state to go to after the waypoint is reached
//				myCaseAfterNav = 6;
//				// set the state
//				myCase = 5;
//				break;
//			case 5:
//				 if(navigation.checkNavStatus() == FINISHED_NAVIGATION){
//					 myCase = myCaseAfterNav;
//				 }
//				 break;
//			case 6:
//				myCase = 1;
//				status = Running;
//				break;
//		}

// BASIC MOTION
//
//	static int myCase = 1;
//	static int myCaseAfterMotion = 2;
//	switch(myCase){
//		case 1:
//			 robotChassis.driveBackwards(300, 5000);
//			 myCase = 2;
//			 myCaseAfterMotion = 3;
//			 break;
//		case 2:
//			if(robotChassis.statusOfChassisDriving() == REACHED_SETPOINT){
//				myCase = myCaseAfterMotion;
//			}
//			break;
//		case 3:
//			 robotChassis.driveForward(300, 5000);
//			 myCase = 2;
//			 myCaseAfterMotion = 4;
//			 break;
//		case 4:
//			 robotChassis.turnToHeading(90, 5000);
//			 myCase = 2;
//			 myCaseAfterMotion = 5;
//			 break;
//		case 5:
//			 robotChassis.turnToHeading(-90, 5000);
//			 myCase = 2;
//			 myCaseAfterMotion = 6;
//			 break;
//		case 6:
//			 status = Running;
//			 break;
//	}
//
    break;

	}
//	digitalWrite(WII_CONTROLLER_DETECT, 0);
}


