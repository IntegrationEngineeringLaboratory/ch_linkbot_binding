#include<ch.h>
#include <math.h>
#include<stdio.h>
#include <string.h>
#include<functional>
#include "linkbot_wrapper.h"

#ifdef _WIN32
#include <windows.h>
#include <Shlobj.h>
#endif

#define unimplemented() \
  fprintf(stderr, "Function %s is currently unimplemented.\n", __func__); \
exit(-1)

int linkbot_count = 0;

/*class creator*/
EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  const char* serialId;
  class LinkbotWrapper * l;

  Ch_VaStart(interp, ap, varg);  
  if (Ch_VaCount(interp, ap) == 0){
    l= newLinkbotIWrapper(nullptr);
    Ch_CppChangeThisPointer(interp, l, sizeof(LinkbotWrapper));
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

/*MOVEMENT FUNCTIONS*/

/*linkbot driveAccelJointTimeNB*/
EXPORTCH void CLinkbotI_driveAccelJointTimeNB_chdl(void *varg) {
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

/*linkbotDriveForward*/
EXPORTCH void CLinkbotI_driveForward_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle = Ch_VaArg(interp, ap, double);
  l->driveForward(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotDriveForward*/
EXPORTCH void CLinkbotI_driveForwardNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  double angle;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  angle = Ch_VaArg(interp, ap, double);
  l->driveForwardNB(angle);
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

/*END GET FUNCTIONS*/
/*SET FUNCTIONS*/

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

/*END SET FUNCTIONS*/
/*MISCELLANEOUS FUNCTIONS*/

/*linkbot recordDistanceBegin*/
EXPORTCH void CLinkbotI_recordDistanceBegin_chdl(void *varg) {
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

  l->recordDistanceBegin(*time, *distance, radius, seconds);

  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot recordDistanceEnd*/
EXPORTCH void CLinkbotI_recordDistanceEnd_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class LinkbotWrapper *l;
  robotJointId_t id;
  int *num;

  Ch_VaStart(interp, ap, varg);

  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
  id = Ch_VaArg(interp, ap, robotJointId_t);
  num = Ch_VaArg(interp, ap, int*);
  l->recordDistanceEnd(*num);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbot recordDistanceOffset*/
EXPORTCH void CLinkbotI_recordDistanceOffset_chdl(void *varg) {
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
  class LinkbotWrapper *l;

  Ch_VaStart(interp, ap, varg);
  l = Ch_VaArg(interp, ap, class LinkbotWrapper *);
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

/* barobo::CLinkbotIGroup functions */

/*Constructor*/
EXPORTCH void CLinkbotIGroup_CLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *c=new barobo::CLinkbotIGroup();

  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(barobo::CLinkbotIGroup));
  Ch_VaEnd(interp, ap);
}

/*Destructor*/
EXPORTCH void CLinkbotIGroup_dCLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;


  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  if(Ch_CppIsArrayElement(interp)){
    g->~CLinkbotIGroup();

  }
  else {
    delete g;
  }
  Ch_VaEnd(interp, ap);
  return;

}

/*linkbotGroup driveDistanceNB*/
EXPORTCH void CLinkbotIGroup_driveDistanceNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
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
  class barobo::CLinkbotIGroup *g;
  double distance;
  double radius;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  distance=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  g->driveDistance(distance, radius);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveAngle*/
EXPORTCH void CLinkbotIGroup_driveAngle_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  angle = Ch_VaArg(interp, ap, double);
  g->driveAngle(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveAngleNB*/
EXPORTCH void CLinkbotIGroup_driveAngleNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g = Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  angle = Ch_VaArg(interp, ap, double);
  g->driveAngleNB(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveForeverNB*/
EXPORTCH void CLinkbotIGroup_driveForeverNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  g->driveForeverNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveTimeNB*/
EXPORTCH void CLinkbotIGroup_driveTimeNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  time=Ch_VaArg(interp, ap, double);
  g->driveTimeNB(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup driveTime*/
EXPORTCH void CLinkbotIGroup_driveTime_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double time;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  time=Ch_VaArg(interp, ap, double);
  g->driveTime(time);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup turnLeftNB*/
EXPORTCH void CLinkbotIGroup_turnLeftNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
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
  class barobo::CLinkbotIGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
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
  class barobo::CLinkbotIGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
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
  class barobo::CLinkbotIGroup *g;
  double angle;
  double radius;
  double tracklength;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  angle=Ch_VaArg(interp, ap, double);
  radius=Ch_VaArg(interp, ap, double);
  tracklength=Ch_VaArg(interp, ap, double);
  g->turnRight(angle, radius, tracklength);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup openGripperNB*/
EXPORTCH void CLinkbotIGroup_openGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  angle=Ch_VaArg(interp, ap, double);
  g->openGripperNB(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup openGripper*/
EXPORTCH void CLinkbotIGroup_openGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double angle;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  angle=Ch_VaArg(interp, ap, double);
  g->openGripper(angle);
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup closeGripperNB*/
EXPORTCH void CLinkbotIGroup_closeGripperNB_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  g->closeGripperNB();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup closeGripper*/
EXPORTCH void CLinkbotIGroup_closeGripper_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;

  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  g->closeGripper();
  Ch_VaEnd(interp, ap);
  return;
}

/*linkbotGroup delaySeconds*/
EXPORTCH void CLinkbotIGroup_delaySeconds_chdl(void *varg) {
  unimplemented();
#if 0
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotIGroup *g;
  double seconds;

  Ch_VaStart(interp, ap, varg);
  g=Ch_VaArg(interp, ap, class barobo::CLinkbotIGroup *);
  seconds=Ch_VaArg(interp, ap, double);
  g->delaySeconds(seconds);
  Ch_VaEnd(interp, ap);

  return;
#endif
}
