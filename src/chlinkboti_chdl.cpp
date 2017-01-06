#include<ch.h>
#include <math.h>
#include<stdio.h>
#include<functional>

#include "linkbot_wrapper.h"

#define unimplemented() \
  fprintf(stderr, "Function %s is currently unimplemented.\n", __func__); \
exit(-1)

/*class creator*/
EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  const char* serialId;
  class LinkbotWrapper * l;

  Ch_VaStart(interp, ap, varg);  
  if (Ch_VaCount(interp, ap) == 0){
    // l= new barobo::CLinkbotI();
    //Ch_CppChangeThisPointer(interp, l, sizeof(barobo::CLinkbotI));
    printf("Wrong number of argument passed\n");
  }
  else if (Ch_VaCount(interp, ap) == 1){
    serialId = Ch_VaArg(interp, ap, const char *);
    l = newLinkbotIWrapper(serialId);
    Ch_CppChangeThisPointer(interp, l, sizeof(LinkbotWrapper));
  }
  else {
    printf("Wrong number of argument passed\n");
  }

  Ch_VaEnd(interp, ap);
  return;
}

/* class destructor*/   
EXPORTCH void CLinkbotI_dCLinkbotI_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);
  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  if(Ch_CppIsArrayElement(interp)){
    l->~LinkbotWrapper();
  }
  else{
    delete l;
  }
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot connectWithSerialID*/
EXPORTCH int CLinkbotI_connectWithSerialID_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  const char *id;
  int type;
  int rc;
  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id = Ch_VaArg(interp, ap, const char*);
  rc = l->connectWithSerialID(id);
  Ch_VaEnd(interp, ap);
  l->getFormFactor(type);
  if (type == 1)
  {
    printf("A Linkbot-L is connected, not a Linkbot-I.\nPlease connect a Linbot-I.\nExiting..\n");
    exit(-1);
  }

  return rc;
#endif
}

/*linkbot connect*/
EXPORTCH int CLinkbotI_connect_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int type;
  int rc;
  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  rc = l->connect();
  Ch_VaEnd(interp, ap);
  l->getFormFactor(type);
  if (type == 1)
  {
    printf("A Linkbot-L is connected, not a Linkbot-I.\nPlease connect a Linbot-I.\nExiting..\n");
    exit(-1);
  }

  return rc;
#endif
}

/*linkbot disconnect*/
EXPORTCH void CLinkbotI_disconnect_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->disconnect();
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*MOVEMENT FUNCTIONS*/

/*linkbot accelJointAngleNB*/
EXPORTCH void CLinkbotI_accelJointAngleNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double acceleration;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  acceleration=Ch_VaArg(interp, ap, double);
  angle=Ch_VaArg(interp, ap, double);
  l->accelJointAngleNB(id, acceleration, angle);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot accelJointTimeNB*/
EXPORTCH void CLinkbotI_accelJointTimeNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double acceleration;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  acceleration=Ch_VaArg(interp, ap, double);
  time=Ch_VaArg(interp, ap, double);
  l->accelJointTimeNB(id, acceleration, time);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot accelJointToVelocityNB*/
EXPORTCH void CLinkbotI_accelJointToVelocityNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double acceleration;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  acceleration=Ch_VaArg(interp, ap, double);
  time=Ch_VaArg(interp, ap, double);
  l->accelJointToVelocityNB(id, acceleration, time);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot accelJointToMaxSpeedNB*/
EXPORTCH void CLinkbotI_accelJointToMaxSpeedNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double acceleration;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  acceleration=Ch_VaArg(interp, ap, double);
  l->accelJointToMaxSpeedNB(id, acceleration);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot driveAccelJointTimeNB*/
EXPORTCH void CLinkbotI_driveAccelJointTimeNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double radius;
  double acceleration;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  radius=Ch_VaArg(interp, ap, double);
  acceleration=Ch_VaArg(interp, ap, double);
  time=Ch_VaArg(interp, ap, double);
  l->driveAccelJointTimeNB(radius, acceleration, time);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot driveAccelToVelocityNB*/
EXPORTCH void CLinkbotI_driveAccelToVelocityNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double radius;
  double acceleration;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  radius=Ch_VaArg(interp, ap, double);
  acceleration=Ch_VaArg(interp, ap, double);
  time=Ch_VaArg(interp, ap, double);
  l->driveAccelToVelocityNB(radius, acceleration, time);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot driveAccelToMaxSpeedNB*/
EXPORTCH void CLinkbotI_driveAccelToMaxSpeedNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double radius;
  double acceleration;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  radius=Ch_VaArg(interp, ap, double);
  acceleration=Ch_VaArg(interp, ap, double);
  l->driveAccelToMaxSpeedNB(radius, acceleration);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot driveAccelDistanceNB*/
EXPORTCH void CLinkbotI_driveAccelDistanceNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double radius;
  double acceleration;
  double distance;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  radius=Ch_VaArg(interp, ap, double);
  acceleration=Ch_VaArg(interp, ap, double);
  distance=Ch_VaArg(interp, ap, double);
  l->driveAccelDistanceNB(radius, acceleration, distance);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot move*/
EXPORTCH void CLinkbotI_move_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double j1;
  double j2;
  double j3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  j1=Ch_VaArg(interp, ap, double);
  j2=Ch_VaArg(interp, ap, double);
  j3=Ch_VaArg(interp, ap, double);
  l->move(j1, j2, j3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveNB*/
EXPORTCH void CLinkbotI_moveNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double j1;
  double j2;
  double j3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  j1=Ch_VaArg(interp, ap, double);
  j2=Ch_VaArg(interp, ap, double);
  j3=Ch_VaArg(interp, ap, double);
  l->moveNB(j1, j2, j3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveToByTrackPos*/
EXPORTCH void CLinkbotI_moveToByTrackPos_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  l->moveToByTrackPos(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}


/*linkbot moveToByTrackPosNB*/
EXPORTCH void CLinkbotI_moveToByTrackPosNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  l->moveToByTrackPosNB(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointToByTrackPos*/
EXPORTCH void CLinkbotI_moveJointToByTrackPos_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  l->moveJointToByTrackPos(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointToByTrackPosNB*/
EXPORTCH void CLinkbotI_moveJointToByTrackPosNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  l->moveJointToByTrackPosNB(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveAngle*/
EXPORTCH void CLinkbotI_driveAngle_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle = Ch_VaArg(interp, ap, double);
  l->driveAngle(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveAngleNB*/
EXPORTCH void CLinkbotI_driveAngleNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle = Ch_VaArg(interp, ap, double);
  l->driveAngleNB(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveDistance*/
EXPORTCH void CLinkbotI_driveDistance_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  distance=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  l->driveDistance(distance, radius);
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbotDriveDistanceNB*/
EXPORTCH void CLinkbotI_driveDistanceNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  distance=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  l->driveDistanceNB(distance, radius);
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbotDriveForeverNB*/
EXPORTCH void CLinkbotI_driveForeverNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->driveForeverNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveTime*/
EXPORTCH void CLinkbotI_driveTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  seconds=Ch_VaArg(interp, ap, double);
  l->driveTime(seconds);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveTimeNB*/
EXPORTCH void CLinkbotI_driveTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  seconds=Ch_VaArg(interp, ap, double);
  l->driveTimeNB(seconds);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveWait*/
EXPORTCH void CLinkbotI_moveWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int mask;    

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  mask=Ch_VaArg(interp, ap, int);
  l->moveWait(mask);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointWait*/
EXPORTCH void CLinkbotI_moveJointWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;    

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  l->moveJointWait(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot isMoving*/
EXPORTCH int CLinkbotI_isMoving_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int mask;  
  int retval;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  mask=Ch_VaArg(interp, ap, int);
  retval=l->isMoving(mask);
  Ch_VaEnd(interp, ap);
  return retval;
#endif
}

/*linkbot isConnected*/
EXPORTCH int CLinkbotI_isConnected_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int retval;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  retval = l->isConnected();
  Ch_VaEnd(interp, ap);
  return retval;
#endif
}

/*linkbot stop*/
EXPORTCH void CLinkbotI_stop_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->stop();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot stopOneJoint*/
EXPORTCH void CLinkbotI_stopOneJoint_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  l->stopOneJoint(id);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot turnLeft*/
EXPORTCH void CLinkbotI_turnLeft_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  l->turnLeft(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbot turnLeftNB*/
EXPORTCH void CLinkbotI_turnLeftNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  l->turnLeftNB(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot turnRight*/
EXPORTCH void CLinkbotI_turnRight_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  l->turnRight(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot turnRightNB*/
EXPORTCH void CLinkbotI_turnRightNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  l->turnRightNB(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot holdJoint*/
EXPORTCH void CLinkbotI_holdJoint_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  l->holdJoint(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot holdJoints*/
EXPORTCH void CLinkbotI_holdJoints_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->holdJoints();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot holdJointsAtExit*/
EXPORTCH void CLinkbotI_holdJointsAtExit_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->holdJointsAtExit();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot relaxJoint*/
EXPORTCH void CLinkbotI_relaxJoint_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  l->relaxJoint(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot relaxJoints*/
EXPORTCH void CLinkbotI_relaxJoints_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->relaxJoints();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveForeverNB*/
EXPORTCH void CLinkbotI_moveForeverNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->moveForeverNB();
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbot moveJoint*/
EXPORTCH void CLinkbotI_moveJoint_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  l->moveJoint(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbot moveJointNB*/
EXPORTCH void CLinkbotI_moveJointNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  l->moveJointNB(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointForeverNB*/
EXPORTCH void CLinkbotI_moveJointForeverNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  l->moveJointForeverNB(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveTime*/
EXPORTCH void CLinkbotI_moveTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  time=Ch_VaArg(interp, ap, double);
  l->moveTime(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveTimeNB*/
EXPORTCH void CLinkbotI_moveTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  time=Ch_VaArg(interp, ap, double);
  l->moveTimeNB(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointTime*/
EXPORTCH void CLinkbotI_moveJointTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  time=Ch_VaArg(interp, ap, double);
  l->moveJointTime(id, time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointTimeNB*/
EXPORTCH void CLinkbotI_moveJointTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  time=Ch_VaArg(interp, ap, double);
  l->moveJointTimeNB(id, time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot openGripper*/
EXPORTCH void CLinkbotI_openGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle=Ch_VaArg(interp, ap, double);
  l->openGripper(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot openGripperNB*/
EXPORTCH void CLinkbotI_openGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle=Ch_VaArg(interp, ap, double);
  l->openGripperNB(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointTo*/
EXPORTCH void CLinkbotI_moveJointTo_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  l->moveJointTo(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveJointToNB*/
EXPORTCH void CLinkbotI_moveJointToNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  l->moveJointToNB(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveTo*/
EXPORTCH void CLinkbotI_moveTo_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  l->moveTo(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveToNB*/
EXPORTCH void CLinkbotI_moveToNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  l->moveToNB(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveToZero*/
EXPORTCH void CLinkbotI_moveToZero_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->moveToZero();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot moveToZeroNB*/
EXPORTCH void CLinkbotI_moveToZeroNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->moveToZeroNB();
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbot closeGripper*/
EXPORTCH void CLinkbotI_closeGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->closeGripper();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot closeGripperNB*/
EXPORTCH void CLinkbotI_closeGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->closeGripperNB();
  Ch_VaEnd(interp, ap);
  return;
}

/* linkbot drivexy */
EXPORTCH void CLinkbotI_drivexy_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double x, y, radius, trackwidth;
  bool nb = false;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  x=Ch_VaArg(interp, ap, double);
  y=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  trackwidth=Ch_VaArg(interp, ap, double);
  nb = Ch_VaArg(interp, ap, bool);
  l->drivexy(x, y, radius, trackwidth, nb);
  Ch_VaEnd(interp, ap);

  return;
}

/* linkbot drivexyTo */
EXPORTCH void CLinkbotI_drivexyTo_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double x, y, radius, trackwidth;
  bool nb = false;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  x=Ch_VaArg(interp, ap, double);
  y=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  trackwidth=Ch_VaArg(interp, ap, double);
  nb = Ch_VaArg(interp, ap, int);
  l->drivexyTo(x, y, radius, trackwidth, nb);
  Ch_VaEnd(interp, ap);

  return;
}

/* linkbot drivexyToArray */
EXPORTCH void CLinkbotI_drivexyToArray_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *px, *py, radius, trackwidth;
  int num, nb = 0;

  Ch_VaStart(interp, ap, varg);
  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  px = Ch_VaArg(interp, ap, double*);
  py = Ch_VaArg(interp, ap, double*);
  num = Ch_VaArg(interp, ap, int);
  radius = Ch_VaArg(interp, ap, double);
  trackwidth = Ch_VaArg(interp, ap, double);
  nb = Ch_VaArg(interp, ap, int);
  l->drivexyToArray(px, py, num, radius, trackwidth, nb);
  Ch_VaEnd(interp, ap);

  return;
}

/* linkbot drivexyWait */
EXPORTCH void CLinkbotI_drivexyWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);
  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->drivexyWait();
  Ch_VaEnd(interp, ap);

  return;
}
/*END MOVEMENT FUNCTIONS*/
/*GET FUNCTIONS*/

/*linkbot getAccelerometerData*/
EXPORTCH void CLinkbotI_getAccelerometerData_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *x;
  double *y;
  double *z;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  x=Ch_VaArg(interp, ap, double *);
  y=Ch_VaArg(interp, ap, double *);
  z=Ch_VaArg(interp, ap, double *);
  l->getAccelerometerData(*x, *y, *z);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getBatteryVoltage*/
EXPORTCH void CLinkbotI_getBatteryVoltage_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *voltage;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  voltage = Ch_VaArg(interp, ap, double *);
  l->getBatteryVoltage(*voltage);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getLEDColorRGB*/
EXPORTCH void CLinkbotI_getLEDColorRGB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int *r;
  int *g;
  int *b;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  r=Ch_VaArg(interp, ap, int *);
  g=Ch_VaArg(interp, ap, int *);
  b=Ch_VaArg(interp, ap, int *);
  l->getLEDColorRGB(*r, *g, *b);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getLEDColor*/
EXPORTCH void CLinkbotI_getLEDColor_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  char * color;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  color=Ch_VaArg(interp, ap, char *);
  l->getLEDColor(color);
  Ch_VaEnd(interp, ap);
  return;
}


/*linkbot getDistance*/
EXPORTCH void CLinkbotI_getDistance_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *distance;
  double radius;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  distance=Ch_VaArg(interp, ap, double *);
  radius=Ch_VaArg(interp, ap, double);
  l->getDistance(*distance, radius);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointAngleInstant*/
EXPORTCH void CLinkbotI_getJointAngleInstant_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *angle;
  robotJointId_t id;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double *);
  l->getJointAngleInstant(id, *angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointAnglesInstant*/
EXPORTCH void CLinkbotI_getJointAnglesInstant_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *angle1;
  double *angle2;
  double *angle3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle1=Ch_VaArg(interp, ap, double *);
  angle2=Ch_VaArg(interp, ap, double *);
  angle3=Ch_VaArg(interp, ap, double *);
  l->getJointAnglesInstant(*angle1, *angle2, *angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointAngle*/
EXPORTCH void CLinkbotI_getJointAngle_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *angle;
  robotJointId_t id;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double *);
  l->getJointAngle(id, *angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointAngles*/
EXPORTCH void CLinkbotI_getJointAngles_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *angle1;
  double *angle2;
  double *angle3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle1=Ch_VaArg(interp, ap, double *);
  angle2=Ch_VaArg(interp, ap, double *);
  angle3=Ch_VaArg(interp, ap, double *);
  l->getJointAngles(*angle1, *angle2, *angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointSpeed*/
EXPORTCH void CLinkbotI_getJointSpeed_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *speed;
  robotJointId_t id;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  speed=Ch_VaArg(interp, ap, double *);
  l->getJointSpeed(id, *speed);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointSpeedRatio*/
EXPORTCH void CLinkbotI_getJointSpeedRatio_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *ratio;
  robotJointId_t id;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  ratio=Ch_VaArg(interp, ap, double *);
  l->getJointSpeedRatio(id, *ratio);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointSpeeds*/
EXPORTCH void CLinkbotI_getJointSpeeds_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *speed1;
  double *speed2;
  double *speed3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  speed1=Ch_VaArg(interp, ap, double *);
  speed2=Ch_VaArg(interp, ap, double *);
  speed3=Ch_VaArg(interp, ap, double *);
  l->getJointSpeeds(*speed1, *speed2, *speed3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointSpeedRatios*/
EXPORTCH void CLinkbotI_getJointSpeedRatios_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *ratio1;
  double *ratio2;
  double *ratio3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  ratio1=Ch_VaArg(interp, ap, double *);
  ratio2=Ch_VaArg(interp, ap, double *);
  ratio3=Ch_VaArg(interp, ap, double *);
  l->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getJointSafetyAngle*/
EXPORTCH void CLinkbotI_getJointSafetyAngle_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle = Ch_VaArg(interp, ap, double *);
  l->getJointSafetyAngle(*angle);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot getJointSafetyAngleTimeout*/
EXPORTCH void CLinkbotI_getJointSafetyAngleTimeout_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *timeout;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  timeout = Ch_VaArg(interp, ap, double *);
  l->getJointSafetyAngleTimeout(*timeout);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*END GET FUNCTIONS*/
/*SET FUNCTIONS*/

/*linkbot setJointMovementStateNB*/
EXPORTCH void CLinkbotI_setJointMovementStateNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  robotJointState_t dir;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  dir=Ch_VaArg(interp, ap, robotJointState_t);
  l->setJointMovementStateNB(id, dir);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot setJointMovementStateTime*/
EXPORTCH void CLinkbotI_setJointMovementStateTime_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  robotJointState_t dir;
  double seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  dir=Ch_VaArg(interp, ap, robotJointState_t);
  seconds=Ch_VaArg(interp, ap, double);
  l->setJointMovementStateTime(id, dir, seconds);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot setJointSpeed*/
EXPORTCH void CLinkbotI_setJointSpeed_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double speed;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  speed=Ch_VaArg(interp, ap, double);
  l->setJointSpeed(id, speed);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setJointSpeeds*/
EXPORTCH void CLinkbotI_setJointSpeeds_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double speed1;
  double speed2;
  double speed3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  speed1=Ch_VaArg(interp, ap, double);
  speed2=Ch_VaArg(interp, ap, double);
  speed3=Ch_VaArg(interp, ap, double);
  l->setJointSpeeds(speed1, speed2, speed3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setJointSpeedRatio*/
EXPORTCH void CLinkbotI_setJointSpeedRatio_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double ratio;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  ratio=Ch_VaArg(interp, ap, double);
  l->setJointSpeedRatio(id, ratio);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setJointSpeedRatios*/
EXPORTCH void CLinkbotI_setJointSpeedRatios_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double ratio1;
  double ratio2;
  double ratio3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  ratio1=Ch_VaArg(interp, ap, double);
  ratio2=Ch_VaArg(interp, ap, double);
  ratio3=Ch_VaArg(interp, ap, double);
  l->setJointSpeedRatios(ratio1, ratio2, ratio3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setJointPower*/
EXPORTCH void CLinkbotI_setJointPower_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  int power;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  power=Ch_VaArg(interp, ap, int);
  l->setJointPower(id, power);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setMotorPowers*/
EXPORTCH void CLinkbotI_setMotorPowers_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double p1;
  double p2;
  double p3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  p1=Ch_VaArg(interp, ap, double);
  p2=Ch_VaArg(interp, ap, double);
  p3=Ch_VaArg(interp, ap, double);
  l->setMotorPowers(p1, p2, p3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setMovementStateNB*/
EXPORTCH void CLinkbotI_setMovementStateNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointState_t dir1;
  robotJointState_t dir2;
  robotJointState_t dir3;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  dir1=Ch_VaArg(interp, ap, robotJointState_t);
  dir2=Ch_VaArg(interp, ap, robotJointState_t);
  dir3=Ch_VaArg(interp, ap, robotJointState_t);
  l->setMovementStateNB(dir1, dir2, dir3);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot setMovementStateTime*/
EXPORTCH void CLinkbotI_setMovementStateTime_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointState_t dir1;
  robotJointState_t dir2;
  robotJointState_t dir3;
  double seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  dir1=Ch_VaArg(interp, ap, robotJointState_t);
  dir2=Ch_VaArg(interp, ap, robotJointState_t);
  dir3=Ch_VaArg(interp, ap, robotJointState_t);
  seconds=Ch_VaArg(interp, ap, double);
  l->setMovementStateTime(dir1, dir2, dir3, seconds);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot setMovementStateTime*/
EXPORTCH void CLinkbotI_setMovementStateTimeNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointState_t dir1;
  robotJointState_t dir2;
  robotJointState_t dir3;
  double seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  dir1=Ch_VaArg(interp, ap, robotJointState_t);
  dir2=Ch_VaArg(interp, ap, robotJointState_t);
  dir3=Ch_VaArg(interp, ap, robotJointState_t);
  seconds=Ch_VaArg(interp, ap, double);
  l->setMovementStateTimeNB(dir1, dir2, dir3, seconds);
  Ch_VaEnd(interp, ap);
  return;
#endif
}


/*linkbot setBuzzerFrequnencyOn*/
EXPORTCH void CLinkbotI_setBuzzerFrequencyOn_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int frequency;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  frequency=Ch_VaArg(interp, ap, int);
  l->setBuzzerFrequencyOn(frequency);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setBuzzerFrequnencyOff*/
EXPORTCH void CLinkbotI_setBuzzerFrequencyOff_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->setBuzzerFrequencyOff();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setBuzzerFrequnency*/
EXPORTCH void CLinkbotI_setBuzzerFrequency_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int frequency;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  frequency=Ch_VaArg(interp, ap, int);
  time=Ch_VaArg(interp, ap, double);
  l->setBuzzerFrequency(frequency, time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setSpeed*/
EXPORTCH void CLinkbotI_setSpeed_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double speed;
  double radius;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  speed=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  l->setSpeed(speed, radius);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setJointSafetyAngle*/
EXPORTCH void CLinkbotI_setJointSafetyAngle_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle = Ch_VaArg(interp, ap, double);
  l->setJointSafetyAngle(angle);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot setJointSafetyAngleTimeout*/
EXPORTCH void CLinkbotI_setJointSafetyAngleTimeout_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double timeout;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  timeout = Ch_VaArg(interp, ap, double);
  l->setJointSafetyAngleTimeout(timeout);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot setLEDColorRGB*/
EXPORTCH void CLinkbotI_setLEDColorRGB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int r;
  int g;
  int b;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  r=Ch_VaArg(interp, ap, int);
  g=Ch_VaArg(interp, ap, int);
  b=Ch_VaArg(interp, ap, int);
  l->setLEDColorRGB(r, g, b);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot setLEDColor*/
EXPORTCH void CLinkbotI_setLEDColor_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  char *color;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  color=Ch_VaArg(interp, ap, char*);
  l->setLEDColor(color);
  Ch_VaEnd(interp, ap);
  return;
}


/*END SET FUNCTIONS*/
/*MISCELLANEOUS FUNCTIONS*/

/*linkbot enableButtonCallback*/
EXPORTCH void CLinkbotI_enableButtonCallback_chdl(void *varg) {
  return; 
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  void *data;
  void (*cb)(void*, int, int);    

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  data=Ch_VaArg(interp, ap, void*);
  cb=(void(*)(void*, int, int))Ch_VaArg(interp, ap, void*);
  l->enableButtonCallback(data, cb);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot disableButtonCallback*/
EXPORTCH void CLinkbotI_disableButtonCallback_chdl(void *varg) {
  return;
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->disableButtonCallback();
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot delaySeconds*/
EXPORTCH void CLinkbotI_delaySeconds_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  seconds=Ch_VaArg(interp, ap, int);
  l->delaySeconds(seconds);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot systemTime*/
EXPORTCH void CLinkbotI_systemTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  time=Ch_VaArg(interp, ap, double *);
  l->systemTime(*time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot resetToZeroNB*/
EXPORTCH void CLinkbotI_resetToZeroNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->resetToZeroNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot resetToZero*/
EXPORTCH void CLinkbotI_resetToZero_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->resetToZero();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot recordAngleBegin*/
EXPORTCH void CLinkbotI_recordAngleBegin_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double** time;
  double** angle;
  double* angle1;
  double* angle2;
  double* angle3;
  double seconds;
  int shiftData;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id = Ch_VaArg(interp, ap, robotJointId_t);
  time = Ch_VaArg(interp, ap, double**);
  angle = Ch_VaArg(interp, ap, double**);
  seconds = Ch_VaArg(interp, ap, double);
  shiftData = Ch_VaArg(interp, ap, int);

  if (id == ROBOT_JOINT1) {
    l->recordAnglesBegin(*time, *angle, angle2, angle3, seconds, 1 << (int(id) - 1), shiftData);
  }
  else if (id == ROBOT_JOINT2) {
    l->recordAnglesBegin(*time, angle1, *angle, angle3, seconds, 1 << (int(id) - 1), shiftData);
  }
  else {
    l->recordAnglesBegin(*time, angle1, angle2, *angle, seconds, 1 << (int(id) - 1), shiftData);
  }

  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot recordAngleEnd*/
EXPORTCH void CLinkbotI_recordAngleEnd_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  int *num;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id = Ch_VaArg(interp, ap, robotJointId_t);
  num = Ch_VaArg(interp, ap, int*);
  l->recordAnglesEnd(*num);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot recordAnglesBegin*/
EXPORTCH void CLinkbotI_recordAnglesBegin_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double** time;
  double** angle1;
  double* ignored = 0;
  double** angle3;
  double seconds;
  int shiftData;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  time = Ch_VaArg(interp, ap, double**);
  angle1 = Ch_VaArg(interp, ap, double**);
  angle3 = Ch_VaArg(interp, ap, double**);
  seconds = Ch_VaArg(interp, ap, double);
  shiftData = Ch_VaArg(interp, ap, int);

  const int mask = 1<<(ROBOT_JOINT1-1) | 1<<(ROBOT_JOINT3-1);
  l->recordAnglesBegin(*time, *angle1, ignored, *angle3, seconds, mask, shiftData);

  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot recordAnglesEnd*/
EXPORTCH void CLinkbotI_recordAnglesEnd_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int *num;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  num = Ch_VaArg(interp, ap, int*);
  l->recordAnglesEnd(*num);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot recordDistanceBegin*/
EXPORTCH void CLinkbotI_recordDistanceBegin_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  double** time;
  double**distance;
  double radius;
  double seconds;
  int shiftData;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id = Ch_VaArg(interp, ap, robotJointId_t);
  time = Ch_VaArg(interp, ap, double**);
  distance = Ch_VaArg(interp, ap, double**);
  radius = Ch_VaArg(interp, ap, double);
  seconds = Ch_VaArg(interp, ap, double);
  shiftData = Ch_VaArg(interp, ap, int);

  l->recordDistanceBegin(id, *time, *distance, radius, seconds, shiftData);

  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot recordDistanceEnd*/
EXPORTCH void CLinkbotI_recordDistanceEnd_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  int *num;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id = Ch_VaArg(interp, ap, robotJointId_t);
  num = Ch_VaArg(interp, ap, int*);
  l->recordDistanceEnd(id, *num);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot enableRecordDataShift*/
EXPORTCH void CLinkbotI_enableRecordDataShift_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->enableRecordDataShift();
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot disableRecordDataShift*/
EXPORTCH void CLinkbotI_disableRecordDataShift_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->disableRecordDataShift();
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot recordNoDataShift*/
EXPORTCH void CLinkbotI_recordNoDataShift_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->recordNoDataShift();
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot recordDistanceOffset*/
EXPORTCH void CLinkbotI_recordDistanceOffset_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double distance;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  distance = Ch_VaArg(interp, ap, double);
  l->recordDistanceOffset(distance);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot blinkLED*/
EXPORTCH void CLinkbotI_blinkLED_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double delay;
  int numBlinks;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  delay = Ch_VaArg(interp, ap, double);
  numBlinks = Ch_VaArg(interp, ap, int);
  l->blinkLED(delay, numBlinks);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot playNotes*/
EXPORTCH void CLinkbotI_playNotes_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  int *frequency, num;
  double *duration;
  bool nb = false;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  frequency=Ch_VaArg(interp, ap, int*);
  duration=Ch_VaArg(interp, ap, double *);
  num=Ch_VaArg(interp, ap, int);
  nb=Ch_VaArg(interp, ap, bool);
  l->playNotes(frequency, duration, num, nb);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot playNotesWait*/
EXPORTCH void CLinkbotI_playNotesWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);
  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  l->playNotesWait();
  Ch_VaEnd(interp, ap);

  return;
}

/*linkbot initPosition*/
EXPORTCH void CLinkbotI_initPosition_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double x, y, angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  x=Ch_VaArg(interp, ap, double);
  y=Ch_VaArg(interp, ap, double);
  angle=Ch_VaArg(interp, ap, double);
  l->initPosition(x, y, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getPosition*/
EXPORTCH void CLinkbotI_getPosition_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *x, *y, *angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  x=Ch_VaArg(interp, ap, double *);
  y=Ch_VaArg(interp, ap, double *);
  angle=Ch_VaArg(interp, ap, double *);
  l->getPosition(*x, *y, *angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getPosition*/
EXPORTCH void CLinkbotI_getxy_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double *x, *y;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  x=Ch_VaArg(interp, ap, double *);
  y=Ch_VaArg(interp, ap, double *);
  l->getxy(*x, *y);
  Ch_VaEnd(interp, ap);
  return;
}

#if 0
/* CLinkbotIGroup functions */

/*Constructor*/
EXPORTCH void CLinkbotIGroup_CLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *c=new LinkbotGroup();

  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(LinkbotGroup));
  Ch_VaEnd(interp, ap);
}

/*Destructor*/
EXPORTCH void CLinkbotIGroup_dCLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;


  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  if(Ch_CppIsArrayElement(interp)){
    g->~LinkbotGroup();

  }
  else {
    delete g;
  }
  Ch_VaEnd(interp, ap);
  return;

}

/*linkbotGroup addRobot*/
EXPORTCH void CLinkbotIGroup_addRobot_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  l=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  g->addRobot(*l);
  Ch_VaEnd(interp, ap);
}

/*linkbotGroup addRobots*/
EXPORTCH void CLinkbotIGroup_addRobots_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  class LinkbotWrapper *robots;
  int numRobots;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  robots=Ch_VaArg(interp, ap, class LinkbotWrapper *);
  numRobots=Ch_VaArg(interp, ap, int);
  g->addRobots(robots, numRobots);
  Ch_VaEnd(interp, ap);
#endif
}

/*linkbotGroup connect*/
EXPORTCH void CLinkbotIGroup_connect_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  int checkType;
  int type = 0;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->connect();
  Ch_VaEnd(interp, ap);

  checkType=g->checkFormFactor(type);
  if (checkType == -1){
    printf("WARNING: Not all the Linkbots in the group are Linkbot-Is.\nPlease check the Linkbots.\nExiting...\n");
    exit(-1);
  }
#endif
}

/*linkbotGroup driveDistanceNB*/
EXPORTCH void CLinkbotIGroup_driveDistanceNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  distance=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  g->driveDistanceNB(distance, radius);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveDistance*/
EXPORTCH void CLinkbotIGroup_driveDistance_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  distance=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  g->driveDistance(distance, radius);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveWait*/
EXPORTCH void CLinkbotIGroup_moveWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->moveWait();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointWait*/
EXPORTCH void CLinkbotIGroup_moveJointWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  g->moveJointWait(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveAngle*/
EXPORTCH void CLinkbotIGroup_driveAngle_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle = Ch_VaArg(interp, ap, double);
  g->driveAngle(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveAngleNB*/
EXPORTCH void CLinkbotIGroup_driveAngleNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle = Ch_VaArg(interp, ap, double);
  g->driveAngleNB(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveForeverNB*/
EXPORTCH void CLinkbotIGroup_driveForeverNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->driveForeverNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveTimeNB*/
EXPORTCH void CLinkbotIGroup_driveTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  time=Ch_VaArg(interp, ap, double);
  g->driveTimeNB(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveTime*/
EXPORTCH void CLinkbotIGroup_driveTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  time=Ch_VaArg(interp, ap, double);
  g->driveTime(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup holdJoint*/
EXPORTCH void CLinkbotIGroup_holdJoint_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  g->holdJoint(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup holdJoints*/
EXPORTCH void CLinkbotIGroup_holdJoints_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->holdJoints();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup holdJointsAtExit*/
EXPORTCH void CLinkbotIGroup_holdJointsAtExit_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->holdJointsAtExit();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup turnLeftNB*/
EXPORTCH void CLinkbotIGroup_turnLeftNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  g->turnLeftNB(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup turnLeft*/
EXPORTCH void CLinkbotIGroup_turnLeft_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  g->turnLeft(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbotGroup turnRightNB*/
EXPORTCH void CLinkbotIGroup_turnRightNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  g->turnRightNB(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}
/*linkbotGroup turnRight*/
EXPORTCH void CLinkbotIGroup_turnRight_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  g->turnRight(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup relaxJoint*/
EXPORTCH void CLinkbotIGroup_relaxJoint_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  g->relaxJoint(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup relaxJoints*/
EXPORTCH void CLinkbotIGroup_relaxJoints_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->relaxJoints();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup stop*/
EXPORTCH void CLinkbotIGroup_stop_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->stop();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveNB*/
EXPORTCH void CLinkbotIGroup_moveNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double j1;
  double j2;
  double j3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  j1=Ch_VaArg(interp, ap, double);
  j2=Ch_VaArg(interp, ap, double);
  j3=Ch_VaArg(interp, ap, double);
  g->moveNB(j1, j2, j3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup move*/
EXPORTCH void CLinkbotIGroup_move_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double j1;
  double j2;
  double j3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  j1=Ch_VaArg(interp, ap, double);
  j2=Ch_VaArg(interp, ap, double);
  j3=Ch_VaArg(interp, ap, double);
  g->move(j1, j2, j3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveForeverNB*/
EXPORTCH void CLinkbotIGroup_moveForeverNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->moveForeverNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveToNB*/
EXPORTCH void CLinkbotIGroup_moveToNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  g->moveToNB(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveTo*/
EXPORTCH void CLinkbotIGroup_moveTo_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  g->moveTo(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveToZeroNB*/
EXPORTCH void CLinkbotIGroup_moveToZeroNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->moveToZeroNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveToZero*/
EXPORTCH void CLinkbotIGroup_moveToZero_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->moveToZero();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveToByTrackPosNB*/
EXPORTCH void CLinkbotIGroup_moveToByTrackPosNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  g->moveToByTrackPosNB(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveToByTrackPos*/
EXPORTCH void CLinkbotIGroup_moveToByTrackPos_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle1;
  double angle2;
  double angle3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle1=Ch_VaArg(interp, ap, double);
  angle2=Ch_VaArg(interp, ap, double);
  angle3=Ch_VaArg(interp, ap, double);
  g->moveToByTrackPos(angle1, angle2, angle3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointNB*/
EXPORTCH void CLinkbotIGroup_moveJointNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  g->moveJointNB(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}


/*linkbotGroup moveJoint*/
EXPORTCH void CLinkbotIGroup_moveJoint_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  g->moveJoint(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}


/*linkbotGroup moveJointForeverNB*/
EXPORTCH void CLinkbotIGroup_moveJointForeverNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  g->moveJointForeverNB(id);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointToNB*/
EXPORTCH void CLinkbotIGroup_moveJointToNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  g->moveJointToNB(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointTo*/
EXPORTCH void CLinkbotIGroup_moveJointTo_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  g->moveJointTo(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointToByTrackPosNB*/
EXPORTCH void CLinkbotIGroup_moveJointToByTrackPosNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  g->moveJointToByTrackPosNB(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointToByTrackPos*/
EXPORTCH void CLinkbotIGroup_moveJointToByTrackPos_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  angle=Ch_VaArg(interp, ap, double);
  g->moveJointToByTrackPos(id, angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup openGripperNB*/
EXPORTCH void CLinkbotIGroup_openGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle=Ch_VaArg(interp, ap, double);
  g->openGripperNB(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup openGripper*/
EXPORTCH void CLinkbotIGroup_openGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  angle=Ch_VaArg(interp, ap, double);
  g->openGripper(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup setJointSpeed*/
EXPORTCH void CLinkbotIGroup_setJointSpeed_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double speed;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  speed=Ch_VaArg(interp, ap, double);
  g->setJointSpeed(id, speed);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup setJointSpeedRatio*/
EXPORTCH void CLinkbotIGroup_setJointSpeedRatio_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double ratio;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  ratio=Ch_VaArg(interp, ap, double);
  g->setJointSpeedRatio(id, ratio);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup setJointSpeeds*/
EXPORTCH void CLinkbotIGroup_setJointSpeeds_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double speed1;
  double speed2;
  double speed3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  speed1=Ch_VaArg(interp, ap, double);
  speed2=Ch_VaArg(interp, ap, double);
  speed3=Ch_VaArg(interp, ap, double);
  g->setJointSpeeds(speed1, speed2, speed3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup setJointSpeedRatios*/
EXPORTCH void CLinkbotIGroup_setJointSpeedRatios_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double ratio1;
  double ratio2;
  double ratio3;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  ratio1=Ch_VaArg(interp, ap, double);
  ratio2=Ch_VaArg(interp, ap, double);
  ratio3=Ch_VaArg(interp, ap, double);
  g->setJointSpeedRatios(ratio1, ratio2, ratio3);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup setSpeed*/
EXPORTCH void CLinkbotIGroup_setSpeed_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double speed;
  double radius;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  speed=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  g->setSpeed(speed, radius);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup setLEDColorRGB*/
EXPORTCH void CLinkbotIGroup_setLEDColorRGB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  int r;
  int gr;
  int b;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class LinkbotGroup *);
  r = Ch_VaArg(interp, ap, int);
  gr = Ch_VaArg(interp, ap, int);
  b = Ch_VaArg(interp, ap, int);
  g->setLEDColorRGB(r, gr, b);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup setLEDColorRGB*/
EXPORTCH void CLinkbotIGroup_setLEDColor_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  char* color;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class LinkbotGroup *);
  color = Ch_VaArg(interp, ap, char*);
  g->setLEDColor(color);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup resetToZero*/
EXPORTCH void CLinkbotIGroup_resetToZero_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->resetToZero();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup resetToZeroNB*/
EXPORTCH void CLinkbotIGroup_resetToZeroNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->resetToZeroNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup isMoving*/
EXPORTCH int CLinkbotIGroup_isMoving_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  int mask;
  int retval;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  mask=Ch_VaArg(interp, ap, int);
  retval=g->isMoving(mask);
  Ch_VaEnd(interp, ap);
  return retval;
}

/*linkbotGroup isConnected*/
EXPORTCH int CLinkbotIGroup_isConnected_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  int retval;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class LinkbotGroup *);
  retval = g->isConnected();
  Ch_VaEnd(interp, ap);
  return retval;
}

/*linkbotGroup closeGripperNB*/
EXPORTCH void CLinkbotIGroup_closeGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->closeGripperNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup closeGripper*/
EXPORTCH void CLinkbotIGroup_closeGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  g->closeGripper();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointTime*/
EXPORTCH void CLinkbotIGroup_moveJointTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  time=Ch_VaArg(interp, ap, double);
  g->moveJointTime(id, time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveJointTimeNB*/
EXPORTCH void CLinkbotIGroup_moveJointTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  robotJointId_t id;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  id=Ch_VaArg(interp, ap, robotJointId_t);
  time=Ch_VaArg(interp, ap, double);
  g->moveJointTimeNB(id, time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveTime*/
EXPORTCH void CLinkbotIGroup_moveTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  time=Ch_VaArg(interp, ap, double);
  g->moveTime(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup moveTimeNB*/
EXPORTCH void CLinkbotIGroup_moveTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  time=Ch_VaArg(interp, ap, double);
  g->moveTimeNB(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup delaySeconds*/
EXPORTCH void CLinkbotIGroup_delaySeconds_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotGroup *g;
  double seconds;

  Ch_VaStart(interp, ap, varg);
  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  seconds=Ch_VaArg(interp, ap, double);
  g->delaySeconds(seconds);
  Ch_VaEnd(interp, ap);

  return;
}
#endif
