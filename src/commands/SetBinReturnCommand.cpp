/*
 * SetBinReturnCommand.cpp
 *
 *  Created on: Nov 12, 2020
 *      Author: Gabe's PC
 */

#include "SetBinReturnCommand.h"
#include <Arduino.h>
SetBinReturnCommand::SetBinReturnCommand(StudentsRobot* robot) :
		PacketEventAbstract(1912) {
	    robotPointer = robot;

}

void SetBinReturnCommand::event(float * buffer) {
	float row = buffer[0];
	float column = buffer[1];
    float shelf = buffer[2];

	robotPointer -> goalRow = (int) row;
	robotPointer -> goalColumn = (int) column;
	robotPointer -> goalShelf = (int) shelf;
	robotPointer -> status = ReturningBin;

}



