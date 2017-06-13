#ifndef LINKBOT_WRAPPER_H
#define LINKBOT_WRAPPER_H

#include <linkbot/linkbot.hpp>

namespace linkbot {
  enum Button {
    BUTTON_POWER=LINKBOT_BUTTON_POWER,
    BUTTON_A=LINKBOT_BUTTON_A,
    BUTTON_B=LINKBOT_BUTTON_B
  };

  enum ButtonState {
    BUTTON_STATE_UP=LINKBOT_BUTTON_STATE_UP,
    BUTTON_STATE_DOWN=LINKBOT_BUTTON_STATE_DOWN
  };

  enum Direction {
    BACKWARD=LINKBOT_BACKWARD,
    NEUTRAL=LINKBOT_NEUTRAL,
    FORWARD=LINKBOT_FORWARD,
    POSITIVE=LINKBOT_POSITIVE,
    NEGATIVE=LINKBOT_NEGATIVE
  };

  enum FormFactor {
    FORM_FACTOR_I=LINKBOT_FORM_FACTOR_I,
    FORM_FACTOR_L=LINKBOT_FORM_FACTOR_L,
    FORM_FACTOR_T=LINKBOT_FORM_FACTOR_T
  };

  enum Joint {
    JOINT_ONE=LINKBOT_JOINT_ONE,
    JOINT_TWO=LINKBOT_JOINT_TWO,
    JOINT_THREE=LINKBOT_JOINT_THREE
  };

  enum JointState {
    JOINT_STATE_COAST=LINKBOT_JOINT_STATE_COAST,
    JOINT_STATE_HOLD=LINKBOT_JOINT_STATE_HOLD,
    JOINT_STATE_MOVING=LINKBOT_JOINT_STATE_MOVING,
    JOINT_STATE_FAILURE=LINKBOT_JOINT_STATE_FAILURE
  };
}

typedef linkbot::Joint robotJointId_t;
typedef linkbot::JointState robotJointState_t;
typedef linkbot::Direction robotDirection_t;
typedef double* robotRecordData_t;

namespace cstem {
  class RobotPositionRecorder;
  class RobotAction;
  class RobotAction;
}

class LinkbotWrapper {
  public:
    LinkbotWrapper(barobo::CLinkbot * linkbot);
    ~LinkbotWrapper();

    /* MOVE FUNCTIONS */
    void moveForeverNB();
    void moveJoint(robotJointId_t id, double angle);
    void moveJointNB(robotJointId_t id, double angle);
    void moveJointTo(robotJointId_t id, double angle);
    void moveJointToNB(robotJointId_t id, double angle);
    void moveJointToByTrackPos(robotJointId_t id, double angle);
    void moveJointToByTrackPosNB(robotJointId_t id, double angle);
    void moveJointForeverNB(robotJointId_t id);
		void moveJointTime(robotJointId_t id, double time);
		void moveJointTimeNB(robotJointId_t id, double time);
		void moveJointWait(robotJointId_t id);
    void move(double j1, double j2, double j3);
    void moveNB(double j1, double j2, double j3);
    void moveWait(int mask = 0x07);
    void moveTo(double angle1, double angle2, double angle3);
    void moveToNB(double angle1, double angle2, double angle3);
    void moveToByTrackPos(double angle1, double angle2, double angle3);
    void moveToByTrackPosNB(double angle1, double angle2, double angle3);
    void moveToZero();
    void moveToZeroNB();
    void moveTime(double time);
    void moveTimeNB(double time);

    void driveAngle(double angle);
    void driveAngleNB(double angle);
    void driveDistance(double distance, double radius);
    void driveDistanceNB(double distance, double radius);
    void driveTime(double time);
    void driveTimeNB(double time);
    void driveForward(double angle);
    void driveForwardNB(double angle);

    void accelJointAngleNB(robotJointId_t id, double acceleration, double angle);
    void accelJointTimeNB(robotJointId_t id, double acceleration, double time);
    void accelJointToVelocityNB(robotJointId_t id, double acceleration, double speed);
    void accelJointToMaxSpeedNB(robotJointId_t id, double acceleration);
    void driveAccelJointTimeNB(double radius, double acceleration, double time);
    void driveAccelToVelocityNB(double radius, double acceleration, double velocity);
    void driveAccelToMaxSpeedNB(double radius, double acceleration);
    void driveAccelDistanceNB(double radius, double acceleration, double distance);

    void turnLeft(double angle, double radius, double tracklength);
    void turnLeftNB(double angle, double radius, double tracklength);
    void turnRight(double angle, double radius, double tracklength);
    void turnRightNB(double angle, double radius, double tracklength);
    void openGripper(double angle);
    void openGripperNB(double angle);
    void closeGripper();
    void closeGripperNB();
    void driveForeverNB();
    void drivePolar(double angle, double distance, double radius, double trackwidth, bool NB=false);
    void drivexy(double x, double y, double radius, double trackwidth, bool NB=false);
    void drivexyTo(double x, double y, double radius, double trackwidth, bool NB=false);
    void drivexyToArray(double *px, double *py, int num, double radius, double trackwidth, bool NB=false);
    void drivexyWait();
    void holdJoint(robotJointId_t id);
    void holdJoints();
    void holdJointsAtExit();
    void relaxJoint(robotJointId_t id);
    void relaxJoints();
    void resetToZero();
    void resetToZeroNB();
    void stop(int mask = 0x07);

    /* SETTERS */
    void setJointSpeed(robotJointId_t id, double speed);
    void setJointSpeeds(double speed1, double speed2, double speed3);
    void setJointSpeedRatio(robotJointId_t id, double ratio);
    void setJointSpeedRatios(double ratio1, double ratio2, double ratio3);
    void setSpeed(double speed, double radius);
    void setBuzzerFrequency(int frequency, double time);
    void setBuzzerFrequencyOn(int frequency);
    void setBuzzerFrequencyOff();
    void setLEDColorRGB(int r, int g, int b);
    void setLEDColor(char *color);
    void setJointPower(robotJointId_t id, double power);
    void setMotorPowers(double p1, double p2, double p3);
    void setJointMovementStateNB(robotJointId_t id, robotDirection_t dir);
    void setJointMovementStateTime(robotJointId_t id, robotDirection_t dir, double seconds);
    void setJointMovementStateTimeNB(robotJointId_t id, robotDirection_t dir, double seconds);
    void setMovementStateNB(robotDirection_t dir1, robotDirection_t dir2, robotDirection_t dir3);
    void setMovementStateTime(robotDirection_t dir1, robotDirection_t dir2, robotDirection_t dir3, double seconds);
    void setMovementStateTimeNB(robotDirection_t dir1, robotDirection_t dir2, robotDirection_t dir3, double seconds);

    /* GETTERS */
    void getJointAngle(robotJointId_t id, double &angle);
    void getJointAngles(double &angle1, double &angle2, double &angle3);
    void getJointAngleInstant(robotJointId_t id, double &angle);
    void getJointAnglesInstant(double &angle1, double &angle2, double &angle3);
/*
    void getJointSafetyAngle(double &angle);
    void getJointSafetyAngleTimeout(double &timeout);
*/
    void getJointSpeed(robotJointId_t id, double &speed);
    void getJointSpeedRatio(robotJointId_t id, double &ratio);
    void getJointSpeeds(double &speed1, double &speed2, double &speed3);
    void getJointSpeedRatios(double &ratio1, double &ratio2, double &ratio3);
    void getLEDColorRGB(int &r, int &g, int &b);
    void getLEDColor(char color[]);
    void getAccelerometerData(double &x, double &y, double &z);
    void getBatteryVoltage(double &voltage);
    void getDistance(double &distance, double radius);

    /* RECORDING FUNCTIONS */
    /* TODO */

    /* MISC FUNCTIONS */
    void delaySeconds(int seconds);
    void systemTime(double &time);
    void blinkLED(double delay, int numBlinks);

    /* Melody APIs */
    void playNotes(int *frequency, double *duration, int num, bool NB=false);
    void playNotesWait();

    /* Position APIs */
    void initPosition(double x, double y, double angle);
    void getPosition(double &x, double &y, double &angle);
    void getxy(double &x, double &y);

  private:
    barobo::CLinkbot * _linkbot;
    cstem::RobotPositionRecorder * _posRecorder;
    cstem::RobotAction * _drivexyAction;
    cstem::RobotAction * _playNotesAction;
};

LinkbotWrapper * newLinkbotIWrapper(const char * serialId = NULL);
LinkbotWrapper * newLinkbotLWrapper(const char * serialId = NULL);

#endif
