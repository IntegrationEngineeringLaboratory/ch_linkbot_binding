#include<ch.h>
#include <math.h>
#include<stdio.h>
#include<functional>
#include<linkbot/linkbot.hpp>

#define unimplemented() \
fprintf(stderr, "Function %s is currently unimplemented.\n", __func__); \
exit(-1)

/*class creator*/
EXPORTCH void CLinkbotI_CLinkbotI_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
	const char* serialId;
	LinkbotFormFactor type;
	class barobo::CLinkbotI *l;
    
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
	l->getFormFactor(type);
	if (type == 1)
	{
		printf("A Linkbot-L is connected, not a Linkbot-I.\nPlease connect a Linbot-I.\nExiting..\n");
		exit(-1);
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

/*linkbot connectWithSerialID*/
EXPORTCH int CLinkbotI_connectWithSerialID_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotI *l;
    const char *id;
	int type;
    int rc;
    Ch_VaStart(interp, ap, varg);
   
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
    class barobo::CLinkbotI *l;
	int type;
    int rc;
    Ch_VaStart(interp, ap, varg);
   
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
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
    class barobo::CLinkbotI *l;
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
    l->disconnect();
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*MOVEMENT FUNCTIONS*/

/*linkbot driveAccelJointTimeNB*/
EXPORTCH void CLinkbotI_driveAccelJointTimeNB_chdl(void *varg) {
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
    unimplemented();
    // TODO
    //l->driveAccelJointTimeNB(radius, acceleration, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot driveAccelToVelocityNB*/
EXPORTCH void CLinkbotI_driveAccelToVelocityNB_chdl(void *varg) {
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
    unimplemented();
    // TODO
    //l->driveAccelToVelocityNB(radius, acceleration, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot driveAccelToMaxSpeedNB*/
EXPORTCH void CLinkbotI_driveAccelToMaxSpeedNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotI *l;
    double radius;
    double acceleration;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
    radius=Ch_VaArg(interp, ap, double);
    acceleration=Ch_VaArg(interp, ap, double);
    unimplemented();
    // TODO
    //l->driveAccelToMaxSpeedNB(radius, acceleration);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot driveAccelDistanceNB*/
EXPORTCH void CLinkbotI_driveAccelDistanceNB_chdl(void *varg) {
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
    unimplemented();
    // TODO
    //l->driveAccelDistanceNB(radius, acceleration, distance);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbotDriveBackward*/
EXPORTCH void CLinkbotI_driveBackward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotI *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveBackward(angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotDriveBackwardNB*/
EXPORTCH void CLinkbotI_driveBackwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotI *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotDriveForward*/
EXPORTCH void CLinkbotI_driveForward_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotI *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveForward(angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbotDriveForwardNB*/
EXPORTCH void CLinkbotI_driveForwardNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotI *l;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbotI *);
    angle=Ch_VaArg(interp, ap, double);
    l->driveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
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
    //l->openGripper(angle);
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
    //l->openGripperNB(angle);
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
    //l->closeGripper();
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
    //l->closeGripperNB();
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
    //l->getDistance(*distance, radius);
    Ch_VaEnd(interp, ap);
    return;
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

/* CLinkbotIGroup functions */

/*Constructor*/
EXPORTCH void CLinkbotIGroup_CLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotGroup *c=new barobo::CLinkbotGroup();
  
  Ch_VaStart(interp, ap, varg);
  Ch_CppChangeThisPointer(interp, c, sizeof(barobo::CLinkbotGroup));
  Ch_VaEnd(interp, ap);
}

/*Destructor*/
EXPORTCH void CLinkbotIGroup_dCLinkbotIGroup_chdl(void *varg) {
  ChInterp_t interp;
  ChVaList_t ap;
  class barobo::CLinkbotGroup *g;
  
  
  Ch_VaStart(interp, ap, varg);

  g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
  if(Ch_CppIsArrayElement(interp)){
    g->~CLinkbotGroup();
	
  }
  else {
    delete g;
  }
  Ch_VaEnd(interp, ap);
  return;

}

/*linkbotGroup driveDistanceNB*/
EXPORTCH void CLinkbotIGroup_driveDistanceNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double distance;
	double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    distance=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    g->driveDistanceNB(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup driveDistance*/
EXPORTCH void CLinkbotIGroup_driveDistance_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double distance;
	double radius;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    distance=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    g->driveDistance(distance, radius);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup driveBackwardNB*/
EXPORTCH void CLinkbotIGroup_driveBackwardNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveBackwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
#endif

}
/*linkbotGroup driveBackward*/
EXPORTCH void CLinkbotIGroup_driveBackward_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveBackward(angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup driveAngle*/
EXPORTCH void CLinkbotIGroup_driveAngle_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbotGroup *g;
	double angle;

	Ch_VaStart(interp, ap, varg);

	g = Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	g->driveAngle(angle);
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbotGroup driveAngleNB*/
EXPORTCH void CLinkbotIGroup_driveAngleNB_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbotGroup *g;
	double angle;

	Ch_VaStart(interp, ap, varg);

	g = Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle = Ch_VaArg(interp, ap, double);
	g->driveAngleNB(angle);
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbotGroup driveForeverNB*/
EXPORTCH void CLinkbotIGroup_driveForeverNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
   
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->driveForeverNB();
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup driveForwardNB*/
EXPORTCH void CLinkbotIGroup_driveForwardNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveForwardNB(angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}
/*linkbotGroup driveForward*/
EXPORTCH void CLinkbotIGroup_driveForward_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    angle=Ch_VaArg(interp, ap, double);
    g->driveForward(angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup driveTimeNB*/
EXPORTCH void CLinkbotIGroup_driveTimeNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double time;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    time=Ch_VaArg(interp, ap, double);
    g->driveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup driveTime*/
EXPORTCH void CLinkbotIGroup_driveTime_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    double time;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    time=Ch_VaArg(interp, ap, double);
    g->driveTime(time);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup turnLeftNB*/
EXPORTCH void CLinkbotIGroup_turnLeftNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnLeftNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup turnLeft*/
EXPORTCH void CLinkbotIGroup_turnLeft_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnLeft(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
#endif
}
/*linkbotGroup turnRightNB*/
EXPORTCH void CLinkbotIGroup_turnRightNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnRightNB(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
#endif
}
/*linkbotGroup turnRight*/
EXPORTCH void CLinkbotIGroup_turnRight_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle;
	double radius;
	double tracklength;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
	tracklength=Ch_VaArg(interp, ap, double);
    g->turnRight(angle, radius, tracklength);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup openGripperNB*/
EXPORTCH void CLinkbotIGroup_openGripperNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
    g->openGripperNB(angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup openGripper*/
EXPORTCH void CLinkbotIGroup_openGripper_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle=Ch_VaArg(interp, ap, double);
    g->openGripper(angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup closeGripperNB*/
EXPORTCH void CLinkbotIGroup_closeGripperNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->closeGripperNB();
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup closeGripper*/
EXPORTCH void CLinkbotIGroup_closeGripper_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->closeGripper();
    Ch_VaEnd(interp, ap);
    return;
#endif
}

