/*
 * LiftControl.h
 *
 *  Created on: Nov 10, 2020
 *      Author: Nicholas Lanotte
 */
#include "src/pid/PIDMotor.h"
#include "src/commands/GetIMU.h"
#include "config.h"
#ifndef LIFTCONTROL_H_
#define LIFTCONTROL_H_

#define DegreesPerMM 180

class LiftControl {
public:
	LiftControl(PIDMotor * lift);
	virtual ~LiftControl();
	PIDMotor * liftMotor;

	void StartHomeDown();
	void StartHomeUp();
	bool CheckIfAtBottom();
	bool CheckIfAtTop();
	bool SetLiftHeight(float mm);
	bool CheckIfPositionReached();

private:
	bool Homed = false;
	float mmOfMovement = 244;//measured distance of travel
	float ticksOfMovement;
	float ticksPerMM;
	int64_t targetLiftHeightTicks = 0;
};

#endif /* LIFTCONTROL_H_ */
