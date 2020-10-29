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

	robotPointer->navigation.setNavGoal((int) row, (int) column);
	robotPointer -> status = Navigating;
}


