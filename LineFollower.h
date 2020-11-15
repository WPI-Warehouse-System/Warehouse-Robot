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
	   const int ON_BLACK = 3600;//3750;
	   const int ON_GREY = 2850;
	   const int ON_WHITE = 2100;
	   int lineFollowingSpeedForwards_mm_per_sec = 175; // was 150, 125 (& 1.6 below), was 100, 175
	   int lineFollowingSpeedBackwards_mm_per_sec = 175;
	   float lineFollowingKpBackwards = .002*lineFollowingSpeedBackwards_mm_per_sec;
	   //NOTE: The closer we want to be to black, the higher this gain needs to be. Since we will be staying on white 90% of the
	   // time, I made it try to find grey, not wobble between black. This was an oversight on my part last time.
	   float lineFollowingKpForwards = .45; //1.6, was 1.3, 2.4, .4, .55
	   int lineCount = 0;
       bool canCountLine = false; // so we can start on a line

	   void resetLineCount();
	   bool onMarker();
	   void calibrate();

    private:
};




#endif /* LINEFOLLOWER_H_ */
