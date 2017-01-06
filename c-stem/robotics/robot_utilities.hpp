#ifndef __ROBOT_UTILITIES_H__
#define __ROBOT_UTILITIES_H__

#ifndef M_PI
#define M_PI            3.14159265358979323846
#endif

#include <cmath>

namespace cstem {

/* sgn(num) returns -1, 0, +1 */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void getDriveInfoByRelativePosition(double x, double y, double &angle, double &distance)
{
	angle = atan(y/x)/M_PI*180; /* in degrees */
	distance = sqrt(y*y+x*x);

	if(angle<0) {
		angle += 90;
	} else {
		angle -= 90;
	}
}

void getDriveInfoByAbsolutePosition(double x0, double y0, double angle0, double x1, double y1, double &angle, double &distance)
{
	/*
	 * from: a vector describes current robot location
	 * to: a vector describes target location
	 * path: a vector that the robot will drive along
	 * face: a unit vector in the direction of the robot facing
	 */
	double from[2], to[2], path[2], face[2];

	/* init from vector */
	from[0] = x0;
	from[1] = y0;

	/* init to vector */
	to[0] = x1;
	to[1] = y1;

	if(to[0]==0 && to[1]==0) {
		/* if to origin, path = -from vector */
		path[0] = -from[0];
		path[1] = -from[1];
	}
	else if(from[0] == 0 && from[1] == 0) {
		/* if from origin, path = to vector */
		path[0] = to[0];
		path[1] = to[1];
	}
	else {
		/* otherwise, path = to - from */
		path[0] = to[0] - from[0];
		path[1] = to[1] - from[1];
	}

	/* init face vector */
	face[0] = cos(angle0*M_PI/180); /* angle in radius */
	face[1] = sin(angle0*M_PI/180); /* angle in radius */

	/* angle to turn */
	int direction = sgn(face[0]*path[1]-face[1]*path[0]);

	double dot = face[0]*path[0] + face[1]*path[1];
	double normface = sqrt(face[0]*face[0] + face[1]*face[1]); /* norm of face vector */
	double normpath = sqrt(path[0]*path[0] + path[1]*path[1]); /* norm of path vector */

	angle = roundf(acos(dot/normface/normpath)*180/M_PI*direction); /* in degree */

	/* distance to drive */
	distance = roundf(sqrt(path[0]*path[0] + path[1]*path[1])); /* norm of path vector */
}

};

#endif
