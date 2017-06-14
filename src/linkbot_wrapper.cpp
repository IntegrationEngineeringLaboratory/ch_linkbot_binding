#include "linkbot_wrapper.h"
#include "rgbhashtable.h"
#include "robot_position_recorder.h"
#include "robotAction.hpp"
#include "robot_utilities.hpp"

#include <cmath>
#include <thread>
#include <array>
#include <vector>
#include <cstring>

#ifdef _WIN32
#include <windows.h>
#elif defined __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#include <unistd.h>
#else
#include <unistd.h>
#endif

#define callLinkbotFunction(func, ...) \
  _linkbot->func(__VA_ARGS__);

#define callLinkbotIOnlyFunction(func, ...) \
  LinkbotFormFactor type; \
  _linkbot->getFormFactor(type); \
  if(type == LINKBOT_FORM_FACTOR_L) return; \
  barobo::CLinkbotI * linkboti = (barobo::CLinkbotI *) _linkbot; \
  linkboti->func(__VA_ARGS__);
  
#define callLinkbotLOnlyFunction(func, ...) \
  LinkbotFormFactor type; \
  _linkbot->getFormFactor(type); \
  if(type == LINKBOT_FORM_FACTOR_I) return; \
  barobo::CLinkbotI * linkboti = (barobo::CLinkbotI *) _linkbot; \
  linkboti->func(__VA_ARGS__);

typedef struct robotRecord_s {
  robotRecord_s() :
    userShiftData(false),
    userRadius(3.5),
    userDistOffset(0)
  {
  }

  ~robotRecord_s()
  {
  }

  void setRobotRecordData(robotJointId_t id, robotRecordData_t &time, robotRecordData_t &angle)
  {
    _timeData[(int)id-1] = &time;
    _angleData[(int)id-1] = &angle;
  }

  void copyData(std::vector<double> from, robotRecordData_t *to, int num, double shift = 0)
  {
    (*to) = new double[num];

    for(int i=0; i<from.size(); i++) {
      (*to)[i] = from.at(i) - shift;
    }

    for(int i=from.size(); i<num; i++) {
      /* if the size of from is not 0, the rest of (*to) will be set to the
       * last value of from. Otherwise, it will be filled with 0 */
      (*to)[i] = from.size()>0?from.back():0;
    }
  }

  void adjustData(robotRecordData_t *data, int num, double (*adjuster)(double))
  {
    for(int i=0; i<num; i++) {
      (*data)[i] = adjuster((*data)[i]);
    }
  }

  robotRecordData_t* _angleData[3];
  robotRecordData_t* _timeData[3];

  bool userShiftData;
  double userRadius;
  double userDistOffset;
} robotRecord_t;
 
LinkbotWrapper * newLinkbotIWrapper(const char * serialId)
{
  LinkbotFormFactor type;
  barobo::CLinkbot * l = new barobo::CLinkbotI(serialId);

  if(l == NULL) {
    printf("newLinkbotIWrapper(): null pointer!\n");
    exit(-1);
  }

  l->getFormFactor(type);
  if(type == LINKBOT_FORM_FACTOR_L) {
    printf("A Linkbot-L is connected, not a Linkbot-I.\nPlease connect a Linbot-I.\nExiting..\n");
    exit(-1);
  }

  return new LinkbotWrapper(l);
}

LinkbotWrapper * newLinkbotLWrapper(const char * serialId)
{
  LinkbotFormFactor type;
  barobo::CLinkbot * l = new barobo::CLinkbotL(serialId);

  if(l == NULL) {
    printf("newLinkbotLWrapper(): null pointer!\n");
    exit(-1);
  }

  l->getFormFactor(type);
  if(type == LINKBOT_FORM_FACTOR_I) {
    printf("A Linkbot-I is connected, not a Linkbot-L.\nPlease connect a Linbot-L.\nExiting..\n");
    exit(-1);
  }

  return new LinkbotWrapper(l);
}

LinkbotJoint linkbotJoint(robotJointId_t id)
{
  return LinkbotJoint(id);
}

LinkbotDirection linkbotDirection(robotDirection_t dir)
{
  return LinkbotDirection(dir);
}

/*
 * C Functions for multi-threads
 */
void robotDrivePolarImp(LinkbotWrapper *l, double angle, double distance, double radius, double trackwidth, cstem::RobotAction *action = NULL)
{
#ifdef DEBUG
  fprintf(stdout, "angle = %lf\n", angle);
#endif

  if(l) {
    if(angle < 0) {
      l->turnRight(-angle, radius, trackwidth);
    } else {
      l->turnLeft(angle, radius, trackwidth);
    }

    l->driveDistance(distance, radius);
  }

  if(action) action->stop();
}

void robotDrivexyToImp(LinkbotWrapper *l, double x, double y, double radius, double trackwidth, cstem::RobotAction *action=NULL)
{
  double x0, y0, angle0;

  if(l) {
    l->getPosition(x0, y0, angle0);

    double angle, distance;
    cstem::getDriveInfoByAbsolutePosition(x0, y0, angle0, x, y, angle, distance);

    /* drive robot using drivexy function */
    robotDrivePolarImp(l, angle, distance, radius, trackwidth);
  }

  if(action) action->stop();
}

void robotDrivexyToArrayImp(LinkbotWrapper *l, double *px, double *py, int num, double radius, double trackwidth, cstem::RobotAction *action=NULL)
{
  if(l) {
    for(int i=0; i<num; i++) {
      robotDrivexyToImp(l, px[i], py[i], radius, trackwidth);
    }
  }

  if(action) action->stop();
}

void robotPlayNotesImp(LinkbotWrapper *l, int *frequency, double *duration, int num, cstem::RobotAction *action=NULL)
{
  if(l) {
    for(int i=0; i<num; i++) {
      l->setBuzzerFrequency(frequency[i], duration[i]);
    }
  }

  if(action) action->stop();
}

/*
 * LinkbotWrapper Implementation
 */
LinkbotWrapper::LinkbotWrapper(barobo::CLinkbot * linkbot)
: _linkbot(linkbot),
  _record(new robotRecord_t())
{
}

LinkbotWrapper::~LinkbotWrapper()
{
  if(_linkbot) delete _linkbot;
  if(_record) delete _record;
}

void LinkbotWrapper::moveForeverNB()
{
  callLinkbotFunction(
      setMovementStateNB, 
      linkbotDirection(linkbot::POSITIVE), 
      linkbotDirection(linkbot::POSITIVE), 
      linkbotDirection(linkbot::NEGATIVE)
      );
}

void LinkbotWrapper::moveJoint(robotJointId_t id, double angle)
{
  moveJointNB(id, angle);
  moveJointWait(id);
}

void LinkbotWrapper::moveJointNB(robotJointId_t id, double angle)
{
  double speed;
  getJointSpeed(id, speed);

  if (speed < 0) {
    angle = -angle;
  }
  
  callLinkbotFunction(moveJointNB, linkbotJoint(id), angle);
}

void LinkbotWrapper::moveJointTo(robotJointId_t id, double angle)
{
    moveJointToNB(id, angle);
    moveWait();
}

void LinkbotWrapper::moveJointToNB(robotJointId_t id, double angle)
{
  /* TODO */
}

void LinkbotWrapper::moveJointForeverNB(robotJointId_t id)
{
  /* TODO */
}

void LinkbotWrapper::moveJointTime(robotJointId_t id, double time)
{
  moveJointTimeNB(id, time);
  moveJointWait(id);
}

void LinkbotWrapper::moveJointTimeNB(robotJointId_t id, double time)
{
  /* TODO */
}

void LinkbotWrapper::moveJointWait(robotJointId_t id)
{
  _linkbot->moveJointWait(linkbotJoint(id));
}

void LinkbotWrapper::move(double j1, double j2, double j3)
{
  callLinkbotFunction(move, j1, j2, j3);
}

void LinkbotWrapper::moveNB(double j1, double j2, double j3)
{
  callLinkbotFunction(moveNB, j1, j2, j3);
}

void LinkbotWrapper::moveWait(int mask)
{
  callLinkbotFunction(moveWait, mask);
}

void LinkbotWrapper::moveTo(double angle1, double angle2, double angle3)
{
  callLinkbotFunction(moveTo, angle1, angle2, angle3);
}

void LinkbotWrapper::moveToNB(double angle1, double angle2, double angle3)
{
  callLinkbotFunction(moveToNB, angle1, angle2, angle3);
}

void LinkbotWrapper::moveJointToByTrackPos(robotJointId_t id, double angle)
{
  /* TODO */
}

void LinkbotWrapper::moveJointToByTrackPosNB(robotJointId_t id, double angle)
{
  /* TODO */
}

void LinkbotWrapper::moveToByTrackPos(double angle1, double angle2, double angle3)
{
  moveToByTrackPosNB(angle1, angle2, angle3);
  moveWait();
}

void LinkbotWrapper::moveToByTrackPosNB(double angle1, double angle2, double angle3)
{
  /* TODO */
}

void LinkbotWrapper::moveToZero()
{
  /* TODO */
}

void LinkbotWrapper::moveToZeroNB()
{
  /* TODO */
}

void LinkbotWrapper::moveTime(double time)
{
  /* TODO */
}

void LinkbotWrapper::moveTimeNB(double time)
{
  /* TODO */
}

void LinkbotWrapper::resetToZero()
{
  callLinkbotFunction(resetToZero);
}

void LinkbotWrapper::resetToZeroNB()
{
  callLinkbotFunction(resetToZeroNB);
}

void LinkbotWrapper::stop(int mask)
{
  callLinkbotFunction(stop, mask);
}

void LinkbotWrapper::driveAngle(double angle)
{
  callLinkbotIOnlyFunction(driveAngle, angle);
}

void LinkbotWrapper::driveAngleNB(double angle)
{
  callLinkbotIOnlyFunction(driveAngleNB, angle);
}

void LinkbotWrapper::driveDistance(double distance, double radius)
{
  callLinkbotIOnlyFunction(driveDistance, distance, radius);
}

void LinkbotWrapper::driveDistanceNB(double distance, double radius)
{
  callLinkbotIOnlyFunction(driveDistanceNB, distance, radius);
}

void LinkbotWrapper::driveForeverNB()
{
  callLinkbotIOnlyFunction(driveForeverNB);
}

void LinkbotWrapper::driveTime(double time)
{
  callLinkbotIOnlyFunction(driveTime, time);
}

void LinkbotWrapper::driveTimeNB(double time)
{
  callLinkbotIOnlyFunction(driveTimeNB, time);
}

void LinkbotWrapper::driveForward(double angle)
{
  driveAngle(angle);
}

void LinkbotWrapper::driveForwardNB(double angle)
{
  driveAngleNB(angle);
}

void LinkbotWrapper::driveAccelJointTimeNB(double radius, double acceleration, double time)
{
  callLinkbotFunction(driveAccelJointTimeNB, radius, acceleration, time);
}
void LinkbotWrapper::accelJointAngleNB(robotJointId_t id, double acceleration, double angle)
{
  callLinkbotFunction(accelJointAngleNB, linkbotJoint(id), acceleration, angle);
}

void LinkbotWrapper::accelJointTimeNB(robotJointId_t id, double acceleration, double time)
{
  callLinkbotFunction(accelJointTimeNB, linkbotJoint(id), acceleration, time);
}

void LinkbotWrapper::accelJointToVelocityNB(robotJointId_t id, double acceleration, double speed)
{
  callLinkbotFunction(accelJointToVelocityNB, linkbotJoint(id), acceleration, speed);
}

void LinkbotWrapper::accelJointToMaxSpeedNB(robotJointId_t id, double acceleration)
{
  callLinkbotFunction(accelJointToMaxSpeedNB, linkbotJoint(id), acceleration);
}

void LinkbotWrapper::driveAccelToVelocityNB(double radius, double acceleration, double velocity)
{
  callLinkbotFunction(driveAccelToVelocityNB, radius, acceleration, velocity);
}

void LinkbotWrapper::driveAccelToMaxSpeedNB(double radius, double acceleration)
{
  callLinkbotFunction(driveAccelToMaxSpeedNB, radius, acceleration);
}

void LinkbotWrapper::driveAccelDistanceNB(double radius, double acceleration, double distance)
{
  callLinkbotFunction(driveAccelDistanceNB, radius, acceleration, distance);
}

void LinkbotWrapper::turnLeft(double angle, double radius, double tracklength)
{
  callLinkbotIOnlyFunction(turnLeft, angle, radius, tracklength);
}

void LinkbotWrapper::turnLeftNB(double angle, double radius, double tracklength)
{
  callLinkbotIOnlyFunction(turnLeftNB, angle, radius, tracklength);
}

void LinkbotWrapper::turnRight(double angle, double radius, double tracklength)
{
  callLinkbotIOnlyFunction(turnRight, angle, radius, tracklength);
}

void LinkbotWrapper::turnRightNB(double angle, double radius, double tracklength)
{
  callLinkbotIOnlyFunction(turnRightNB, angle, radius, tracklength);
}

void LinkbotWrapper::openGripper(double angle)
{
  move(-angle/2.0, 0, -angle/2.0);
}

void LinkbotWrapper::openGripperNB(double angle)
{
  moveNB(-angle/2.0, 0, -angle/2.0);  
}

void LinkbotWrapper::closeGripper()
{
  /* TODO */
}

void LinkbotWrapper::closeGripperNB()
{
}

void LinkbotWrapper::drivePolar(
    double angle, 
    double distance, 
    double radius, 
    double trackwidth, 
    bool NB)
{
  if(NB) {
    _drivexyAction->start();
    std::thread driveThread(robotDrivePolarImp, this, angle, distance, radius, trackwidth, _drivexyAction);
    driveThread.detach();
  } else {
    robotDrivePolarImp(this, angle, distance, radius, trackwidth, _drivexyAction);
  }
}

void LinkbotWrapper::drivexy(
    double x, 
    double y, 
    double radius, 
    double trackwidth, 
    bool NB)
{
  double angle, distance;
  cstem::getDriveInfoByRelativePosition(x, y, angle, distance);

  drivePolar(angle, distance, radius, trackwidth, NB);
}

void LinkbotWrapper::drivexyTo(
    double x, 
    double y, 
    double radius, 
    double trackwidth, 
    bool NB)
{
  if(NB) {
    _drivexyAction->start();
    std::thread driveThread(robotDrivexyToImp, this, x, y, radius, trackwidth, _drivexyAction);
    driveThread.detach();
  } else {
    robotDrivexyToImp(this, x, y, radius, trackwidth, _drivexyAction);
  }
}

void LinkbotWrapper::drivexyToArray(
    double *px, 
    double *py, 
    int num, 
    double radius, 
    double trackwidth, 
    bool NB)
{
  if(NB) {
    _drivexyAction->start();
    std::thread driveThread(robotDrivexyToArrayImp, this, px, py, num, radius, trackwidth, _drivexyAction);
    driveThread.detach();
  } else {
    robotDrivexyToArrayImp(this, px, py, num, radius, trackwidth, _drivexyAction);
  }
}

void LinkbotWrapper::drivexyWait()
{
  while(_drivexyAction->isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(int(200)));
  }
}

void LinkbotWrapper::holdJoint(robotJointId_t id)
{
}

void LinkbotWrapper::holdJoints()
{
}

void LinkbotWrapper::holdJointsAtExit()
{
}

void LinkbotWrapper::relaxJoint(robotJointId_t id)
{
}

void LinkbotWrapper::relaxJoints()
{
}

/* SETTERS */
void LinkbotWrapper::setJointSpeed(robotJointId_t id, double speed)
{
  _linkbot->setJointSpeed(linkbotJoint(id), speed);
}

void LinkbotWrapper::setJointSpeeds(double speed1, double speed2, double speed3)
{
  _linkbot->setJointSpeeds(speed1, speed2, speed3);
}

void LinkbotWrapper::setJointSpeedRatio(robotJointId_t id, double ratio)
{
  _linkbot->setJointSpeedRatio(linkbotJoint(id), ratio);
}

void LinkbotWrapper::setJointSpeedRatios(double ratio1, double ratio2, double ratio3)
{
  _linkbot->setJointSpeedRatios(ratio1, ratio2, ratio3);
}

void LinkbotWrapper::setSpeed(double speed, double radius)
{
  double omega;
  omega = speed/radius; // in rad/s
  omega = (omega*180.0)/M_PI; // in deg/s
  if (omega > 200) {
    std::cout << "Warning: cannot set joint speeds to "
              << omega << " degrees/second." << std::endl;
    std::cout << "It is beyond the limit of 200 degrees/second. "
              << "Joints speeds will be set to 200 degrees/second."
              <<std::endl;
    omega = 200.0;
  }
  else if (omega < -200){
    std::cout << "Warning: cannot set joint speeds to " 
              << omega << " degrees/second." << std::endl;
    std::cout << "It is beyond the limit of -200 degrees/second. "
              << "Joints speeds will be set to -200 degrees/second." 
              << std::endl;
    omega = -200.0;
  }
  setJointSpeeds(omega, 0, omega);
}

void LinkbotWrapper::setBuzzerFrequencyOn(int frequency)
{
  callLinkbotFunction(setBuzzerFrequencyOn, frequency);
}

void LinkbotWrapper::setBuzzerFrequencyOff()
{
  callLinkbotFunction(setBuzzerFrequencyOff);
}

void LinkbotWrapper::setBuzzerFrequency(int frequency, double time)
{
  callLinkbotFunction(setBuzzerFrequency, frequency, time);
}

void LinkbotWrapper::setLEDColorRGB(int r, int g, int b)
{
  callLinkbotFunction(setLEDColorRGB, r, g, b);
}

void LinkbotWrapper::setLEDColor(char *color)
{
  int htRetval;
  int getRGB[3];
  rgbHashTable * rgbTable = HT_Create();

  htRetval = HT_Get(rgbTable, color, getRGB);
  HT_Destroy(rgbTable);

  setLEDColorRGB(getRGB[0], getRGB[1], getRGB[2]);
}

void LinkbotWrapper::setJointPower(robotJointId_t id, double power)
{
  callLinkbotFunction(setJointPower, linkbotJoint(id), power);
}

void LinkbotWrapper::setMotorPowers(double p1, double p2, double p3)
{
  callLinkbotFunction(setMotorPowers, p1, p2, p3);
}

void LinkbotWrapper::setJointMovementStateNB(robotJointId_t id, robotDirection_t dir)
{
  callLinkbotFunction(setJointMovementStateNB, linkbotJoint(id), linkbotDirection(dir));
}

void LinkbotWrapper::setJointMovementStateTime(robotJointId_t id, robotDirection_t dir, double seconds)
{
  callLinkbotFunction(setJointMovementStateTime, linkbotJoint(id), linkbotDirection(dir), seconds);
}

void LinkbotWrapper::setJointMovementStateTimeNB(robotJointId_t id, robotDirection_t dir, double seconds)
{
  callLinkbotFunction(setJointMovementStateTimeNB, linkbotJoint(id), linkbotDirection(dir), seconds);
}

void LinkbotWrapper::setMovementStateNB(robotDirection_t dir1, robotDirection_t dir2, robotDirection_t dir3)
{
  callLinkbotFunction(setMovementStateNB, linkbotDirection(dir1), linkbotDirection(dir2), linkbotDirection(dir3));
}

void LinkbotWrapper::setMovementStateTime(robotDirection_t dir1, robotDirection_t dir2, robotDirection_t dir3, double seconds)
{
  callLinkbotFunction(setMovementStateTime, linkbotDirection(dir1), linkbotDirection(dir2), linkbotDirection(dir3), seconds);
}

void LinkbotWrapper::setMovementStateTimeNB(robotDirection_t dir1, robotDirection_t dir2, robotDirection_t dir3, double seconds)
{
  callLinkbotFunction(setMovementStateTimeNB, linkbotDirection(dir1), linkbotDirection(dir2), linkbotDirection(dir3), seconds);
}

/* GETTERS */
void LinkbotWrapper::getJointAngle(robotJointId_t id, double &angle)
{
  double angles[3];
  static int numReadings = 10;

  angle = 0;
  for (int i=0; i<numReadings; i++)
  {
    getJointAnglesInstant(angles[0], angles[1], angles[2]);
    angle += angles[int(id)-1];
  }
  angle = angle/numReadings;
}

void LinkbotWrapper::getJointAngles(double &angle1, double &angle2, double &angle3)
{
  double angles[3];
  static int numReadings = 10;

  angle1 = 0;
  angle2 = 0;
  angle3 = 0;
  for (int i=0; i<numReadings; i++)
  {
    getJointAnglesInstant(angles[0], angles[1], angles[2]);
    angle1 += angles[0];
    angle2 += angles[1];
    angle3 += angles[2];
  }
  angle1 = angle1/numReadings;
  angle2 = angle2/numReadings;
  angle3 = angle3/numReadings;
}

void LinkbotWrapper::getJointAngleInstant(robotJointId_t id, double &angle)
{
  double angles[3];
  getJointAnglesInstant(angles[0], angles[1], angles[2]);
  angle = angles[int(id)-1];
}

void LinkbotWrapper::getJointAnglesInstant(double &angle1, double &angle2, double &angle3)
{
  _linkbot->getJointAngles(angle1, angle2, angle3);
}

void LinkbotWrapper::getJointSpeed(robotJointId_t id, double &speed)
{
  _linkbot->getJointSpeed(linkbotJoint(id), speed);
}

void LinkbotWrapper::getJointSpeedRatio(robotJointId_t id, double &ratio)
{
  _linkbot->getJointSpeedRatio(linkbotJoint(id), ratio);
}

void LinkbotWrapper::getJointSpeeds(double &speed1, double &speed2, double &speed3)
{
  _linkbot->getJointSpeeds(speed1, speed2, speed3);
}

void LinkbotWrapper::getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3)
{
  _linkbot->getJointSpeedRatios(ratio1, ratio2, ratio3);
}

void LinkbotWrapper::getLEDColorRGB(int &r, int &g, int &b)
{
  _linkbot->getLEDColorRGB(r, g, b);
}

void LinkbotWrapper::getLEDColor(char color[])
{
  int getRGB[3];
  int retval;
  rgbHashTable * rgbTable = NULL;

  getLEDColorRGB(getRGB[0], getRGB[1], getRGB[2]);

  rgbTable = HT_Create();
  retval = HT_GetKey(rgbTable, getRGB, color);
  HT_Destroy(rgbTable);
}

void LinkbotWrapper::getAccelerometerData(double &x, double &y, double &z)
{
  callLinkbotFunction(getAccelerometerData, x, y, z);
}

void LinkbotWrapper::getBatteryVoltage(double &voltage)
{
  callLinkbotFunction(getBatteryVoltage, voltage);
}

void LinkbotWrapper::getDistance(double &distance, double radius)
{
  double angle;
  getJointAngle(linkbot::JOINT_ONE, angle);
  distance = (angle*M_PI/180.0)*radius;
}

/* RECORDING FUNCTIONS */
void LinkbotWrapper::recordAngleBegin(
    robotJointId_t id,
    robotRecordData_t &time,
    robotRecordData_t &angle,
    double seconds)
{
  _record->_timeData[(int)id-1] = &time;
  _record->_angleData[(int)id-1] = &angle;

  callLinkbotFunction(recordAnglesBegin);
}

void LinkbotWrapper::recordAngleEnd(robotJointId_t id, int &num)
{
  int timestamp_index = 2*((int)id-1);
  int angle_index = timestamp_index + 1;

  auto data = callLinkbotFunction(recordAnglesEnd);

  num = data[angle_index].size();
  _record->copyData(data[timestamp_index], _record->_timeData[(int)id-1], num, data[timestamp_index].at(0));
  _record->copyData(data[angle_index], _record->_angleData[(int)id-1], num);

  _record->adjustData(_record->_timeData[(int)id-1], num, [](double value){ return value/1000.0; });
}

void LinkbotWrapper::recordAnglesBegin(
    robotRecordData_t &time,
    robotRecordData_t &angle1,
    robotRecordData_t &angle2,
    robotRecordData_t &angle3,
    double seconds)
{
  _record->_timeData[0] = &time;
  _record->_angleData[0] = &angle1;
  _record->_angleData[1] = &angle2;
  _record->_angleData[2] = &angle3;

  callLinkbotFunction(recordAnglesBegin);
}

void LinkbotWrapper::recordAnglesEnd(int &num)
{
  auto data = callLinkbotFunction(recordAnglesEnd);

  /* find the index for time data set that has the max points
   * Since in Ch, the function should return same size of data
   * for time, angle1, angle2 and angle3. However, the library
   * records angles asynchronizely so that they have different
   * sizes. Hence, we need to find the set of data has maximum
   * number of points and make all data in the same size. The
   * copy function will fill the extra data points with the
   * final value. */
  int index = data[0].size()>data[2].size()?0:2;
  index = data[4].size()>data[index].size()?4:index;

  num = data[index].size();

  _record->copyData(data[index], _record->_timeData[0], num, data[index].at(0));
  _record->copyData(data[1], _record->_angleData[0], num);
  _record->copyData(data[3], _record->_angleData[1], num);
  _record->copyData(data[5], _record->_angleData[2], num);

  _record->adjustData(_record->_timeData[0], num, [](double value){ return value/1000.0; });
}

void LinkbotWrapper::recordDistanceBegin(
    robotRecordData_t &time,
    robotRecordData_t &distance,
    double radius,
    double seconds)
{
  _record->userRadius = radius;
  recordAngleBegin((robotJointId_t) 1, time, distance, seconds);
}

void LinkbotWrapper::recordDistanceEnd(int &num)
{
  recordAngleEnd((robotJointId_t) 1, num);

  for(int i=0; i<num; i++) {
    (*_record->_angleData[0])[i] = (*_record->_angleData[0])[i]/180*M_PI*_record->userRadius;
  }
  //_record->adjustData(_record->_angleData[0], num, [this](double value)->double { return value/180*M_PI*(*this)._record->userRadius; });
}

void LinkbotWrapper::enableRecordDataShift()
{
}

void LinkbotWrapper::disableRecordDataShift()
{
}

void LinkbotWrapper::recordNoDataShift()
{
}

void LinkbotWrapper::recordDistanceOffset(double distance)
{
}

void LinkbotWrapper::recordDataShift()
{
}


/* MISC FUNCTIONS */
void LinkbotWrapper::delaySeconds(int seconds)
{
  std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

void LinkbotWrapper::systemTime(double &time)
{
#ifdef _WIN32
  time = (GetTickCount()/1000.0);
#elif defined __MACH__
  clock_serv_t cclock;
  mach_timespec_t mts;
  mach_timespec_t cur_time;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  cur_time.tv_nsec = mts.tv_nsec;
  time = mts.tv_sec;
  time += (mts.tv_nsec / 1000000000.0);
#else
  struct timespec cur_time;
  clock_gettime(CLOCK_REALTIME, &cur_time);
  time = cur_time.tv_sec;
  time += (cur_time.tv_nsec/1000000000.0);
#endif
}

void LinkbotWrapper::blinkLED(double delay, int numBlinks)
{
  std::thread blinkThread {
    [this, numBlinks, delay] {
      int r,g,b;
      getLEDColorRGB(r,g,b);
      for(int i = 0; i < numBlinks; i++) {
        setLEDColorRGB(0,0,0);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay*1000)));
        setLEDColorRGB(r,g,b);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(delay*1000)));
      }
    }
  };
  blinkThread.detach();
}

/* Melody APIs */
void LinkbotWrapper::playNotes(int *frequency, double *duration, int num, bool NB)
{
  if(NB) {
    _playNotesAction->start();
    std::thread playNotesThread(robotPlayNotesImp, this, frequency, duration, num, _playNotesAction);
    playNotesThread.detach();
  }
  else {
    robotPlayNotesImp(this, frequency, duration, num);
  }
}

void LinkbotWrapper::playNotesWait()
{
  while(_playNotesAction->isRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(int(200)));
  }
}

/* Position APIs */
void LinkbotWrapper::initPosition(double x, double y, double angle)
{
  _posRecorder->initPosition(x, y, angle);
}

void LinkbotWrapper::getPosition(double &x, double &y, double &angle)
{
  _posRecorder->getPosition(x, y, angle);
}

void LinkbotWrapper::getxy(double &x, double &y)
{
  _posRecorder->getxy(x, y);
}
