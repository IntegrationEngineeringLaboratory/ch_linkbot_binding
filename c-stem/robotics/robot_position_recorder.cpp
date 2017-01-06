#include "robot_position_recorder.h"
#include "position.hpp"

using namespace cstem;

/*
 * public:
 */
RobotPositionRecorder::RobotPositionRecorder() : 
	_pos(new Position())
{
}

RobotPositionRecorder::~RobotPositionRecorder()
{
	if(_pos) delete _pos;
	_pos = 0;
}

void RobotPositionRecorder::initPosition(double x, double y, double angle)
{
	if(_pos) _pos->init(x, y, angle);
}

void RobotPositionRecorder::getPosition(double &x, double &y, double &angle)
{
	if(_pos) _pos->get(x, y, angle);
}

void RobotPositionRecorder::getxy(double &x, double &y)
{
	if(_pos) _pos->getxy(x, y);
}

void RobotPositionRecorder::updatePosByAngle(double angle, double jointSpeed, double wheelSize)
{
	double distance = M_PI*angle/180 * wheelSize;
	updatePosByDist(distance, jointSpeed);
}

void RobotPositionRecorder::updatePosByDist(double dist, double jointSpeed)
{
	if(_pos) _pos->updateByDistance(jointSpeed>0?dist:-dist);
}

void RobotPositionRecorder::updatePosByTime(double seconds, double jointSpeed, double wheelSize)
{
	double angle = jointSpeed*seconds;
	updatePosByAngle(angle, jointSpeed, wheelSize);
}

void RobotPositionRecorder::updateAngle(double angle)
{
	if(_pos) _pos->updateByAngle(angle);
}
