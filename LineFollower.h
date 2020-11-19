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

// AS OF 6:57 on 11/15/2020, the working values are:
/*
 *  const int ON_BLACK = 3600;//3750;
	const int ON_GREY = 2875;
	const int ON_WHITE = 2150;

   int lineFollowingSpeedForwards_mm_per_sec = 175;
   float lineFollowingKpForwards = .35;
   int lineCount = 0;
   bool canCountLine = false; // so we can start on a line

 */

class LineFollower{
    public:
	   LineFollower();
//	   //NOTE: The closer we want to be to black, the higher this gain needs to be. Since we will be staying on white 90% of the
//	   // time, I made it try to find grey, not wobble between black. This was an oversight on my part last time
//	   float lineFollowingKpForwards = .035; //1.6, was 1.3, 2.4, .4, .55
	   const int ON_BLACK_FOLLOW = 3600;//3750;
	   const int ON_GREY_FOLLOW = 3150;
	   const int ON_WHITE_FOLLOW = 2700;
	   const int ON_BLACK_DETECT = 3200;
	   const int ON_GREY_DETECT = 2500;
	   const int ON_WHITE_DETECT = 1800;
	   int lineFollowingSpeedForwards_mm_per_sec = 160; // was 150, 125 (& 1.6 below), was 100, 175
	   int lineFollowingSpeedBackwards_mm_per_sec = 175;
	   float lineFollowingKpBackwards = .002*lineFollowingSpeedBackwards_mm_per_sec;
	   //NOTE: The closer we want to be to black, the higher this gain needs to be. Since we will be staying on white 90% of the
	   // time, I made it try to find grey, not wobble between black. This was an oversight on my part last time
	   float lineFollowingKpForwards = .15; //1.6, was 1.3, 2.4, .4, .55
	   int lineCount = 0;
       bool canCountLine = false; // so we can start on a line
   	   int lineDebouncing = 75;

	   void resetLineCount();
	   bool onMarker();
	   bool onMarkerFront();
	   void calibrate();

    private:
};




#endif /* LINEFOLLOWER_H_ */
