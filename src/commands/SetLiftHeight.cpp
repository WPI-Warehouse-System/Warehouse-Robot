/*
 * SetLiftHeight.cpp
 *
 *  Created on: Nov 11, 2020
 *      Author: nickj
 */

#include "SetLiftHeight.h"

SetLiftHeight::SetLiftHeight(StudentsRobot* robot) :
	PacketEventAbstract(2020) {
	robotPointer = robot;
}

SetLiftHeight::~SetLiftHeight() {
	// TODO Auto-generated destructor stub
}

void SetLiftHeight::event(float * buffer) {
	Serial.println("Recieved Moving Command");
	robotPointer->liftHeight = buffer[0];
	robotPointer -> status = MovingLift;
	robotPointer -> moveLiftState = SETLIFTHEIGHT;
}

