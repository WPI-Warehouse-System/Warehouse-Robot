/*
 * SetNavGoal.cpp
 *
 *  Created on: Oct 29, 2020
 *      Author: gentov
 */

#include "SetNavGoal.h"
#include <Arduino.h>
SetNavGoal::SetNavGoal(StudentsRobot* robot) :
		PacketEventAbstract(1966) {
	    robotPointer = robot;

}

void SetNavGoal::event(float * buffer) {
	float row = buffer[0];
	float column = buffer[1];

	robotPointer -> goalRow = (int) row;
	robotPointer -> goalColumn = (int) column;
	//robotPointer -> navigation.setNavGoal(row, column);
	robotPointer -> status = Navigating;
	robotPointer -> statusAfterNav = Running;

}


