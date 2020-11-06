/*
 * SetParkCommand.h
 *
 *  Created on: Oct 29, 2020
 *      Author: gentov
 */

#ifndef SRC_COMMANDS_SETPARKCOMMAND_H_
#define SRC_COMMANDS_SETPARKCOMMAND_H_

#include <SimplePacketComs.h>
#include "../../StudentsRobot.h"
class SetParkCommand: public PacketEventAbstract {
	StudentsRobot* robotPointer;
public:
	SetParkCommand(StudentsRobot* robot);
	virtual ~SetParkCommand(){}
	void event(float * buffer);
};

#endif /* SRC_COMMANDS_SETPARKCOMMAND_H_ */
