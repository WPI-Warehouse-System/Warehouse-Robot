/*
 * LineFollower.cpp
 *
 *  Created on: Oct 15, 2020
 *      Author: gentov
 */

#include "LineFollower.h"
#include <math.h>

LineFollower::LineFollower(){
}

bool LineFollower::onMarker(){
	int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
    int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
    return (leftSensorValue >= ON_BLACK && rightSensorValue>= ON_BLACK);
}
void LineFollower::resetLineCount(){
    lineCount = 0;
}

