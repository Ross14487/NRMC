/** @file Robot_Navigation.cpp
  * Robot_Navigation uses Pixy Cameras+Laser Range Finders
  * to find the x, y coordinates and heading of the robot.
  * All distances are in meters.  All angles are in degrees.
  *
  * @author Ross Mitchell
  * @version 0.01
  * @date 4/22/15
  *********************************************************/

#include "Robot_Navigation.h"


double Robot_Navigation::getX ()
{
	return xPosit;
}//end getX()

double Robot_Navigation::getY ()
{
	return yPosit;
}//end getY()

double Robot_Navigation::getHeading ()
{
	return heading;
}//end getHeading()

double Robot_Navigation::calculate_X_Posit(bool portPixyAcquired, bool stbdPixyAcquired,
		double portSensorAngle, double  portSensorDist, double  stbdSensorAngle,
		double stbdSensorDist)
{
	double xStbd, xPort;
	if(portPixyAcquired && stbdPixyAcquired)
	{

		if (portSensorDist > stbdSensorDist)
		{
			double xOffWestMarker = stbdSensorDist * sin((stbdSensorAngle + STBD_SERVO_OFFSET)*180.0/256.0);
			xStbd = W_MARKER_OFFSET -  xOffWestMarker + (atan(STBDLASER_FA_OFFSET/STBDLASER_CL_OFFSET) - (270.0 - heading));

		}
		if (stbdSensorDist > portSensorDist)
		{
			double xOffEastMarker = portSensorDist * sin((stbdSensorAngle + PORT_SERVO_OFFSET)*180.0/256.0);
			xPort = E_MARKER_OFFSET -  xOffEastMarker + (atan(PORTLASER_FA_OFFSET/PORTLASER_CL_OFFSET) - (heading-90.0));  //NEED TO FIX TO EAST

		}
	}

	xPosit = (xStbd + xPort)/2;
	return xPosit;
}//end calculate_X_Posit()


double Robot_Navigation::calculate_Y_Posit(bool portPixyAcquired, bool stbdPixyAcquired,
		double portSensorAngle, double  portSensorDist, double  stbdSensorAngle,
		double stbdSensorDist)
{
	if(portPixyAcquired && stbdPixyAcquired)
	{
		yPosit = (portSensorDist + stbdSensorDist)/2; //need to use heading ------

	}//end if both sensors acquired
		else if (portPixyAcquired)
		{
			yPosit = portSensorDist; //need to use heading and portSensorAngle to figure distance and offsets to center of robot
		}
		else
		{
			yPosit = stbdSensorDist; //need to use heading and portSensorAngle to figure distance and offsets to center of robot
		}
	return yPosit;
}//end calculate_Y_Posit()

double Robot_Navigation::calculateHeading(bool portPixyAcquired, bool stbdPixyAcquired,
		double portSensorAngle, double  portSensorDist, double  stbdSensorAngle,
		double stbdSensorDist)
{
	double heading;
	if(portPixyAcquired && stbdPixyAcquired)
	{
		if (portSensorDist > stbdSensorDist)
		{
			double slope = (portSensorDist - stbdSensorDist)/((0.75^2) - (portSensorDist - stbdSensorDist)^2);
			heading = 180.0 + slope;
		}
		else if (portSensorDist > stbdSensorDist)
		{
			double slope = (stbdSensorDist - portSensorDist)/((0.75^2) - (stbdSensorDist - portSensorDist)^2);
						heading = 180.0 - slope;
		}

	}//end if both sensors acquired
	else if (portPixyAcquired)
	{
		heading = (0 + ((portSensorAngle - PORT_SERVO_OFFSET)*180.0/256.0)) % 360.0;
	}
	else
	{
		heading = (360 - ((stbdSensorAngle + STBD_SERVO_OFFSET)*180.0/256.0)) % 360.0;
	}
	return heading;
}//end calculateHeading()
