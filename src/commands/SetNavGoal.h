/*
 * SetNavGoal.h
 *
 *  Created on: Oct 29, 2020
 *      Author: gentov
 */

#ifndef SRC_COMMANDS_SETNAVGOAL_H_
#define SRC_COMMANDS_SETNAVGOAL_H_

#include <SimplePacketComs.h>
#include "../../StudentsRobot.h"
class SetNavGoal: public PacketEventAbstract {
	StudentsRobot* robotPointer;
public:
	SetNavGoal(StudentsRobot* robot);
	virtual ~SetNavGoal(){}
	void event(float * buffer);
};

#endif /* SRC_COMMANDS_SETNAVGOAL_H_ */
