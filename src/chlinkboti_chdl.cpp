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
  class barobo::CLinkbotI * l;

  Ch_VaStart(interp, ap, varg);  
  if (Ch_VaCount(interp, ap) == 0){
    // l= new barobo::CLinkbotI();
    //Ch_CppChangeThisPointer(interp, l, sizeof(barobo::CLinkbotI));
    printf("Wrong number of argument passed\n");
  }
  else if (Ch_VaCount(interp, ap) == 1){
    serialId = Ch_VaArg(interp, ap, const char *);
    l = new barobo::CLinkbotI(serialId);
    Ch_CppChangeThisPointer(interp, l, sizeof(barobo::CLinkbotI));
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
  class barobo::CLinkbotI *l;

  Ch_VaStart(interp, ap, varg);
  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  if(Ch_CppIsArrayElement(interp)){
    l->~CLinkbotI();
  }
  else{
    delete l;
  }
  Ch_VaEnd(interp, ap);
  return;
}

/*MOVEMENT FUNCTIONS*/

/*linkbot driveAccelJointTimeNB*/
EXPORTCH void CLinkbotI_driveAccelJointTimeNB_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double radius;
  double acceleration;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  double radius;
  double acceleration;
  double time;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  double radius;
  double acceleration;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  double radius;
  double acceleration;
  double distance;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  radius=Ch_VaArg(interp, ap, double);
  acceleration=Ch_VaArg(interp, ap, double);
  distance=Ch_VaArg(interp, ap, double);
  l->driveAccelDistanceNB(radius, acceleration, distance);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbotDriveAngle*/
EXPORTCH void CLinkbotI_driveAngle_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  angle = Ch_VaArg(interp, ap, double);
  l->driveAngle(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveAngleNB*/
EXPORTCH void CLinkbotI_driveAngleNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  angle = Ch_VaArg(interp, ap, double);
  l->driveAngleNB(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveDistance*/
EXPORTCH void CLinkbotI_driveDistance_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  l->driveForeverNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveTime*/
EXPORTCH void CLinkbotI_driveTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  seconds=Ch_VaArg(interp, ap, double);
  l->driveTime(seconds);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveTimeNB*/
EXPORTCH void CLinkbotI_driveTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double seconds;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  seconds=Ch_VaArg(interp, ap, double);
  l->driveTimeNB(seconds);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot turnLeft*/
EXPORTCH void CLinkbotI_turnLeft_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  l->turnRightNB(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot openGripper*/
EXPORTCH void CLinkbotI_openGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  angle=Ch_VaArg(interp, ap, double);
  unimplemented();
#if 0
  l->openGripper(angle);
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot openGripperNB*/
EXPORTCH void CLinkbotI_openGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  angle=Ch_VaArg(interp, ap, double);
  unimplemented();
#if 0
  l->openGripperNB(angle);
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot closeGripper*/
EXPORTCH void CLinkbotI_closeGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  unimplemented();
#if 0
  l->closeGripper();
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot closeGripperNB*/
EXPORTCH void CLinkbotI_closeGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  unimplemented();
#if 0
  l->closeGripperNB();
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/* linkbot drivexy */
EXPORTCH void CLinkbotI_drivexy_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double x, y, radius, trackwidth;
  bool nb = false;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  x=Ch_VaArg(interp, ap, double);
  y=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  trackwidth=Ch_VaArg(interp, ap, double);
  nb = Ch_VaArg(interp, ap, bool);
  unimplemented();
#if 0
  l->drivexy(x, y, radius, trackwidth, nb);
#endif
  Ch_VaEnd(interp, ap);

  return;
}

/* linkbot drivexyTo */
EXPORTCH void CLinkbotI_drivexyTo_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double x, y, radius, trackwidth;
  bool nb = false;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  x=Ch_VaArg(interp, ap, double);
  y=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  trackwidth=Ch_VaArg(interp, ap, double);
  nb = Ch_VaArg(interp, ap, int);
  unimplemented();
#if 0
  l->drivexyTo(x, y, radius, trackwidth, nb);
#endif
  Ch_VaEnd(interp, ap);

  return;
}

/* linkbot drivexyToArray */
EXPORTCH void CLinkbotI_drivexyToArray_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double *px, *py, radius, trackwidth;
  int num, nb = 0;

  Ch_VaStart(interp, ap, varg);
  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  px = Ch_VaArg(interp, ap, double*);
  py = Ch_VaArg(interp, ap, double*);
  num = Ch_VaArg(interp, ap, int);
  radius = Ch_VaArg(interp, ap, double);
  trackwidth = Ch_VaArg(interp, ap, double);
  nb = Ch_VaArg(interp, ap, int);
  unimplemented();
#if 0
  l->drivexyToArray(px, py, num, radius, trackwidth, nb);
#endif
  Ch_VaEnd(interp, ap);

  return;
}

/* linkbot drivexyWait */
EXPORTCH void CLinkbotI_drivexyWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;

  Ch_VaStart(interp, ap, varg);
  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  unimplemented();
#if 0
  l->drivexyWait();
#endif
  Ch_VaEnd(interp, ap);

  return;
}
/*END MOVEMENT FUNCTIONS*/
/*GET FUNCTIONS*/

/*linkbot getDistance*/
EXPORTCH void CLinkbotI_getDistance_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double *distance;
  double radius;


  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  distance=Ch_VaArg(interp, ap, double *);
  radius=Ch_VaArg(interp, ap, double);
  unimplemented();
#if 0
  l->getDistance(*distance, radius);
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/*END GET FUNCTIONS*/
/*SET FUNCTIONS*/

/*END SET FUNCTIONS*/
/*MISCELLANEOUS FUNCTIONS*/

/*linkbot recordDistanceBegin*/
EXPORTCH void CLinkbotI_recordDistanceBegin_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  robotJointId_t id;
  double** time;
  double**distance;
  double radius;
  double seconds;
  int shiftData;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *l;
  robotJointId_t id;
  int *num;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  id = Ch_VaArg(interp, ap, robotJointId_t);
  num = Ch_VaArg(interp, ap, int*);
  l->recordDistanceEnd(id, *num);
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
  class barobo::CLinkbotI *l;
  double distance;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  distance = Ch_VaArg(interp, ap, double);
  l->recordDistanceOffset(distance);
  Ch_VaEnd(interp, ap);
  return;
#endif
}

/*linkbot playNotes*/
EXPORTCH void CLinkbotI_playNotes_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  int *frequency, num;
  double *duration;
  bool nb = false;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  frequency=Ch_VaArg(interp, ap, int*);
  duration=Ch_VaArg(interp, ap, double *);
  num=Ch_VaArg(interp, ap, int);
  nb=Ch_VaArg(interp, ap, bool);
  unimplemented();
#if 0
  l->playNotes(frequency, duration, num, nb);
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot playNotesWait*/
EXPORTCH void CLinkbotI_playNotesWait_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;

  Ch_VaStart(interp, ap, varg);
  l = Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  unimplemented();
#if 0
  l->playNotesWait();
#endif
  Ch_VaEnd(interp, ap);

  return;
}

/*linkbot initPosition*/
EXPORTCH void CLinkbotI_initPosition_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double x, y, angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  x=Ch_VaArg(interp, ap, double);
  y=Ch_VaArg(interp, ap, double);
  angle=Ch_VaArg(interp, ap, double);
  unimplemented();
#if 0
  l->initPosition(x, y, angle);
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getPosition*/
EXPORTCH void CLinkbotI_getPosition_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double *x, *y, *angle;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  x=Ch_VaArg(interp, ap, double *);
  y=Ch_VaArg(interp, ap, double *);
  angle=Ch_VaArg(interp, ap, double *);
  unimplemented();
#if 0
  l->getPosition(*x, *y, *angle);
#endif
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot getPosition*/
EXPORTCH void CLinkbotI_getxy_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotI *l;
  double *x, *y;

  Ch_VaStart(interp, ap, varg);

  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
  x=Ch_VaArg(interp, ap, double *);
  y=Ch_VaArg(interp, ap, double *);
  unimplemented();
#if 0
  l->getxy(*x, *y);
#endif
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
  class barobo::CLinkbotI *l;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
  class barobo::CLinkbotI *robots;
  int numRobots;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class LinkbotGroup *);
  robots=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
