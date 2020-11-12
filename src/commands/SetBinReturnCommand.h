/*
 * SetBinReturnCommand.h
 *
 *  Created on: Nov 12, 2020
 *      Author: gentov
 */

#ifndef SRC_COMMANDS_SETBINRETURNCOMMAND_H_
#define SRC_COMMANDS_SETBINRETURNCOMMAND_H_


#include <SimplePacketComs.h>
#include "../../StudentsRobot.h"

class SetBinReturnCommand: public PacketEventAbstract {
	StudentsRobot* robotPointer;
public:
	SetBinReturnCommand(StudentsRobot* robot);
	virtual ~SetBinReturnCommand(){}
	void event(float * buffer);
};


#endif /* SRC_COMMANDS_SETBINRETURNCOMMAND_H_ */
