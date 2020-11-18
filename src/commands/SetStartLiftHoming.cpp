/*
 * SetStartLiftHoming.cpp
 *
 *  Created on: Nov 11, 2020
 *      Author: nickj
 */

#include "SetStartLiftHoming.h"

SetStartLiftHoming::SetStartLiftHoming(StudentsRobot* robot) :
	PacketEventAbstract(2077) {
	robotPointer = robot;

}

SetStartLiftHoming::~SetStartLiftHoming() {
	// TODO Auto-generated destructor stub
}
void SetStartLiftHoming::event(float * buffer) {
	Serial.println("Recieved Homing Command");
	robotPointer -> status = HomingLift;
	robotPointer -> homeLiftState = STARTING_HOME;

}
