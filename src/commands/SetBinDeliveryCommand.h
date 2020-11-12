/*
 * SetBinDeliveryCommand.h
 *
 *  Created on: Nov 12, 2020
 *      Author: gentov
 */

#ifndef SRC_COMMANDS_SETBINDELIVERYCOMMAND_H_
#define SRC_COMMANDS_SETBINDELIVERYCOMMAND_H_

#include <SimplePacketComs.h>
#include "../../StudentsRobot.h"

class SetBinDeliveryCommand: public PacketEventAbstract {
	StudentsRobot* robotPointer;
public:
	SetBinDeliveryCommand(StudentsRobot* robot);
	virtual ~SetBinDeliveryCommand(){}
	void event(float * buffer);
};

#endif /* SRC_COMMANDS_SETBINDELIVERYCOMMAND_H_ */
