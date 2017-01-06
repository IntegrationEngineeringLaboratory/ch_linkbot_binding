#ifndef __CSTEM_ROBOT_POSITION_RECORDER_H__
#define __CSTEM_ROBOT_POSITION_RECORDER_H__

namespace cstem {
class Position;

class RobotPositionRecorder
{
	public:
		RobotPositionRecorder();
		~RobotPositionRecorder();

		void initPosition(double x, double y, double angle);
		void getPosition(double &x, double &y, double &angle);
		void getxy(double &x, double &y);

		void updatePosByAngle(double angle, double jointSpeed, double wheelSize);
		void updatePosByDist(double dist, double jointSpeed);
		void updatePosByTime(double seconds, double jointSpeed, double wheelSize);

		void updateAngle(double angle);

	private:
		Position * _pos;
};
}

#endif
