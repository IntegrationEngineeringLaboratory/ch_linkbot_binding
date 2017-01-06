#ifndef __POSITION_H__
#define __POSITION_H__

#ifndef M_PI
#define M_PI            3.14159265358979323846
#endif

#include <cmath>

namespace cstem {
class Position {
	public:
		Position() 
			: _x(0), _y(0), _angle(0)
		{
		}

		~Position() {}

		void init(double x, double y, double angle) {
			_x = x;
			_y = y;
			_angle = angle;
		}

		void updateByDistance(double d) {
			_y += d * sin(_angle/180*M_PI);
			_x += d * cos(_angle/180*M_PI);
		}
		
		void updateByAngle(double angle) {
			_angle += angle;

			if(_angle>360) _angle -= 360; 
			if(_angle<0) _angle += 360;
		}

		void updateByVector(double theta, double r) {
			_y += r * sin(theta/180*M_PI);
			_x += r * cos(theta/180*M_PI);
		}

		void get(double &x, double &y, double &angle) {
			getxy(x, y);
			angle = _angle;
		}

		void getxy(double &x, double &y) {
			x = _x;
			y = _y;
		}

	private:
		double _x;
		double _y;
		double _angle;
};
}

#endif
