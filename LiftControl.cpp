/*
 * LiftControl.cpp
 *
 *  Created on: Nov 10, 2020
 *      Author: nickj
 */

#include "LiftControl.h"

LiftControl::LiftControl(PIDMotor * lift) {
	ticksPerMM = 1;
	ticksOfMovement = 0;
	liftMotor = lift;


}

LiftControl::~LiftControl() {
	// TODO Auto-generated destructor stub
}
void LiftControl::StartHomeDown(){
	liftMotor->velocityPID.setpid(1, 0, 0);
	liftMotor->setVelocityDegreesPerSecond(DegreesPerMM*-12);//-2mm/sec for testing
}
void LiftControl::StartHomeUp(){
	liftMotor->velocityPID.setpid(1, 0, 0);
	liftMotor->setVelocityDegreesPerSecond(DegreesPerMM*12);//2mm/sec for testing
}

bool LiftControl::CheckIfAtBottom(){
	if(digitalRead(BOTTOM_OPTICALSWITCH)==1){
		liftMotor->overrideCurrentPosition(0);
		liftMotor->setVelocityDegreesPerSecond(0);
		return true;
	}
	else{
		return false;
	}
}
bool LiftControl::CheckIfAtTop(){
	if(digitalRead(TOP_OPTICALSWITCH)==1){
		liftMotor->setVelocityDegreesPerSecond(0);
		ticksOfMovement = liftMotor->getPosition();
		Serial.println("pos");
		Serial.println(ticksOfMovement);
		ticksPerMM = ticksOfMovement/mmOfMovement;
		Homed = true;
		return true;
	}
	else{
		return false;
	}
}

bool LiftControl::SetLiftHeight(float mm){
	liftMotor->SetTunings(0.2, 0.1, 0.1);
	float tickToMoveTo = 0;
	if(Homed==false){
		return false;//Dont move if not homed
	}
	else{
		if(mm<0){
			tickToMoveTo = 0;
		}
		else if(mm>mmOfMovement){
			tickToMoveTo = ticksOfMovement;
		}
		else{
			tickToMoveTo = mm*ticksPerMM;
		}
	}
	targetLiftHeightTicks = (int32_t)tickToMoveTo;
	liftMotor->setSetpoint(targetLiftHeightTicks);
	return true;
}

bool LiftControl::CheckIfPositionReached(){
	// this will return true if lift is not homed, since targetLiftHeightTicks is 0, therefore condition is met
	if(liftMotor->getPosition()<targetLiftHeightTicks+2 && liftMotor->getPosition()>targetLiftHeightTicks-2){
		liftMotor->SetTunings(0.0005, 0.0, 0.0);
		return true;
	}
	else{
		return false;
	}
}

