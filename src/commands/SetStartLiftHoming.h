/*
 * SetStartLiftHoming.h
 *
 *  Created on: Nov 11, 2020
 *      Author: nickj
 */

#ifndef SRC_COMMANDS_SETSTARTLIFTHOMING_H_
#define SRC_COMMANDS_SETSTARTLIFTHOMING_H_

#include <SimplePacketComs.h>
#include "../../StudentsRobot.h"

class SetStartLiftHoming: public PacketEventAbstract{
	StudentsRobot* robotPointer;
public:
	SetStartLiftHoming(StudentsRobot* robot);
	virtual ~SetStartLiftHoming();
	void event(float * buffer);
};

#endif /* SRC_COMMANDS_SETSTARTLIFTHOMING_H_ */
