/*
 * Pose.h
 *
 *  Created on: Oct 21, 2020
 *      Author: Gabe's PC
 */
#ifndef POSE_H_
#define POSE_H_

#define PI 3.14159265f

class Pose{
    public:
	   Pose();
	   // current row and current column will be updated by the line sensor lineCount
	   int currentRow = 2;
	   int currentColumn = 0;
	   // current heading will be updated by the IMU during turnToHeading.
	   float initialHeading = 0;
	   float currentHeading = initialHeading;
	   int rowCount = 0;
	   int colCount = 0;

	   /**
	    * gets the orientation to the closest ordinal direction (0,90,180,-90)
	    */
       int getOrientationToClosest90();
    private:
};



#endif /* POSE_H_ */
