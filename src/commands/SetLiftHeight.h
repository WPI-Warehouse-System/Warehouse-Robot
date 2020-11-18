/*
 * SetLiftHeight.h
 *
 *  Created on: Nov 11, 2020
 *      Author: nickj
 */

#ifndef SRC_COMMANDS_SETLIFTHEIGHT_H_
#define SRC_COMMANDS_SETLIFTHEIGHT_H_

#include <SimplePacketComs.h>
#include "../../StudentsRobot.h"

class SetLiftHeight : public PacketEventAbstract{
	StudentsRobot* robotPointer;
public:
	SetLiftHeight(StudentsRobot* robot);
	virtual ~SetLiftHeight();
	void event(float * buffer);
};

#endif /* SRC_COMMANDS_SETLIFTHEIGHT_H_ */
