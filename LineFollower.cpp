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
	int leftLineDetectSensorValue = analogRead(LEFT_LINE_DETECT);
    int rightLineDetectSensorValue = analogRead(RIGHT_LINE_DETECT);
    return (leftLineDetectSensorValue >= ON_BLACK_DETECT && rightLineDetectSensorValue>= ON_BLACK_DETECT);
}


bool LineFollower::onMarkerFront(){
	int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
    int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
    return (leftSensorValue >= ON_GREY_FOLLOW && rightSensorValue>= ON_GREY_FOLLOW);
}

void LineFollower::calibrate(){
	int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
    int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
	int leftLineDetectSensorValue = analogRead(LEFT_LINE_DETECT);
    int rightLineDetectSensorValue = analogRead(RIGHT_LINE_DETECT);
    Serial.print("LEFT LINE SENSOR VALUE: " + String(leftSensorValue) + " ");
    Serial.print("RIGHT LINE SENSOR VALUE: " + String(rightSensorValue) + " ");
    Serial.print("LEFT DETECT SENSOR VALUE: " + String(leftLineDetectSensorValue) + " ");
    Serial.println("RIGHT DETECT SENSOR VALUE: " + String(rightLineDetectSensorValue) + " ");
}

void LineFollower::resetLineCount(){
    lineCount = 0;
}

