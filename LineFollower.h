/*
 * LineFollower.h
 *
 *  Created on: Oct 15, 2020
 *      Author: gentov
 */

#include "DrivingChassis.h"
#include "config.h"
#ifndef LINEFOLLOWER_H_
#define LINEFOLLOWER_H_


class LineFollower{
    public:
	   LineFollower();
	   // on black line
	   const int ON_BLACK = 3692;//3750;
	   int lineFollowingSpeedForwards_mm_per_sec = 100; // was 150, 125 (& 1.6 below)
	   int lineFollowingSpeedBackwards_mm_per_sec = 175;
	   float lineFollowingKpBackwards = .002*lineFollowingSpeedBackwards_mm_per_sec;
	   float lineFollowingKpForwards = 1.6; //1.6, was 1.3
	   int lineCount = 0;
       bool canCountLine = false; // so we can start on a line

	   void resetLineCount();
	   bool onMarker();

    private:
};




#endif /* LINEFOLLOWER_H_ */
