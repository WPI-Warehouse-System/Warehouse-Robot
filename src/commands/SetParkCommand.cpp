/*
 * SetParkCommand.cpp
 *
 *  Created on: Oct 29, 2020
 *      Author: Gabe's PC
 */

#include "SetParkCommand.h"
#include <Arduino.h>
SetParkCommand::SetParkCommand(StudentsRobot* robot) :
		PacketEventAbstract(1945) {
	    robotPointer = robot;

}

void SetParkCommand::event(float * buffer) {
	float row = buffer[0];
	float column = buffer[1];

	robotPointer -> goalRow = (int) row;
	robotPointer -> goalColumn = (int) column;
	robotPointer -> status = ParkingRobot;

}


