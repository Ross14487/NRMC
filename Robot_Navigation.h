/** @file Robot_Navigation.h
  * Robot_Navigation uses Pixy Cameras+Laser Range Finders
  * to find the x, y coordinates and heading of the robot.
  * All distances are in meters.  All angles are in degrees.
  *
  * @author Ross Mitchell
  * @version 0.01
  * @date 4/22/15
  *********************************************************/

#ifndef ROBOT_NAVIGATION_H_
#define ROBOT_NAVIGATION_H_

#define _USE_MATH_DEFINES
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <math.h>

using namespace std;

/** Robot_Navigatoion - finds x, y coordinates and heading*/
class Robot_Navigation
{
public:
	// getX() will report the x-coordinate of the robot with 0, 0 in NE corner
	//  and x-coordinate will be positive as it moves to the east.
	double getX ();

	// getY() will report the y-coordinate of the robot with 0, 0 in NE corner
	//  and y-coordinate will be positive as it moves to the south.
	double getY ();

	// getHeading() will report the heading in degrees from 000 to 359.9 degrees
	//  with the shovel end being the forward end of the robot and North or 000
	//  being at the center of the dump bin.  The robot will usually be pointing
	//  in a southerly direction as it travels from the start zone to the mining
	//  area and will continue pointing southerly as it travels from the mining
	//  area back to the dump bin.
	double getHeading();

	// calculate_X_Posit() uses the input from the pixie servos and the laser range
	//  finder to calculate xPosit.  It needs heading information from
	//  calculateHeading() to figure out where the center of the robot is located.
	double calculate_X_Posit(bool portPixyAcquired, bool stbdPixyAcquired,
			double portSensorAngle, double  portSensorDist, double  stbdSensorAngle,
			double stbdSensorDist);

	// calculate_Y_Posit() uses the input from the pixie servos and the laser range
	//  finder to calculate yPosit.  It needs heading information from
	//  calculateHeading() to figure out where the center of the robot is located.
	double calculate_Y_Posit(bool portPixyAcquired, bool stbdPixyAcquired,
			double portSensorAngle, double  portSensorDist, double  stbdSensorAngle,
			double stbdSensorDist);

	// calculateHeading() uses the input from the pixie servos to calculate which
	//  way the shovel end of the robot is pointing in degrees from 000 to 359.9.
	//  North or 000 is at the center of the dump bin.
	double calculateHeading(bool portPixyAcquired, bool stbdPixyAcquired,
			double portSensorAngle, double  portSensorDist, double  stbdSensorAngle,
			double stbdSensorDist);

private:
	bool portPixyAcquired, stbdPixyAcquired;
	double xPosit, yPosit, heading, portSensorAngle, stbdSensorAngle,
		portSensorDist, stbdSensorDist;

	//Laser CL(center line) offset is distance from mount to center line.
	//Laser FA(forward aft) offset is distance from mount to center of robot.
	//Servo offset is pulsewidth fix for pointing approx 15 degree off center line.
	//East and West offset are from west side
	//Boundary buffer is like a warning strip in baseall
	double  PORTLASER_CL_OFFSET = -0.375, STBDLASER_CL_OFFSET = 0.375,
			PORTLASER_FA_OFFSET = 0.75, STBDLASER_FA_OFFSET = 0.75,
			PORT_SERVO_OFFSET = -21, STBD_SERVO_OFFSET = 21,
			W_BOUNDARY = 0.00, E_BOUNDARY = 3.78, S_BOUNDARY = 7.38,
			W_MARKER_OFFSET = 1.1025, E_MARKER_OFFSET = 2.6775,
			BOUNDARY__BUFFER = 0.125;



};//end Robot_Navigation




#endif /* ROBOT_NAVIGATION_H_ */
