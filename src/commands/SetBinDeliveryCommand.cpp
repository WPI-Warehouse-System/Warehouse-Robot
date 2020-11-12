/*
 * SetBinDeliveryCommand.cpp
 *
 *  Created on: Nov 12, 2020
 *      Author: gentov
 */

#include "SetBinDeliveryCommand.h"
#include <Arduino.h>
SetBinDeliveryCommand::SetBinDeliveryCommand(StudentsRobot* robot) :
		PacketEventAbstract(1908) {
	    robotPointer = robot;

}

void SetBinDeliveryCommand::event(float * buffer) {
	float row = buffer[0];
	float column = buffer[1];
    float shelf = buffer[2];

	robotPointer -> goalRow = (int) row;
	robotPointer -> goalColumn = (int) column;
	robotPointer -> goalShelf = (int) shelf;
	robotPointer -> status = DeliveringBin;

}


