/*
 * Pose.cpp
 *
 *  Created on: Oct 21, 2020
 *      Author: gentov
 */

#include "Pose.h"
#include <math.h>

// Initialize robot pose.
Pose::Pose(){

}

int Pose::getOrientationToClosest90(){
	float headingInRadians = (currentHeading)*(PI/180.0);
    float sinOrientation = (round(sin(headingInRadians)));
    float cosOrientation = (round(cos(headingInRadians)));

    if(sinOrientation == 1){
    	return 90;
    }
    else if(sinOrientation == -1){
    	return -90;
    }
    else if(cosOrientation == 1){
    	return 0;
    }
    else if(cosOrientation == -1){
    	return 180;
    }

    // should never get here!
    return 0;
}
