
#include<ch.h>
#include<math.h>
#include<stdio.h>
#include<functional>
#include<linkbot/linkbot.hpp>

#define unimplemented() \
fprintf(stderr, "Function %s is currently unimplemented.\n", __func__); \
exit(-1)

typedef enum robotJointState_e
{
    ROBOT_NEUTRAL = 0,
    ROBOT_FORWARD,
    ROBOT_BACKWARD,
    ROBOT_HOLD,
    ROBOT_POSITIVE,
    ROBOT_NEGATIVE,
    ROBOT_ACCEL,
} robotJointState_t;

LinkbotDirection robotJointState_t2LinkbotDirection(robotJointState_t dir)
{
    switch(dir)
    {
        case ROBOT_NEUTRAL:
            return LINKBOT_NEUTRAL;
        case ROBOT_FORWARD:
            return LINKBOT_FORWARD;
        case ROBOT_BACKWARD:
            return LINKBOT_BACKWARD;
        case ROBOT_HOLD:
            return LINKBOT_NEUTRAL;
        case ROBOT_POSITIVE:
            return LINKBOT_POSITIVE;
        case ROBOT_NEGATIVE:
            return LINKBOT_NEGATIVE;
    }
}


/*linkbot accelJointAngleNB*/
EXPORTCH void CLinkbot_accelJointAngleNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double acceleration;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    acceleration=Ch_VaArg(interp, ap, double);
    angle=Ch_VaArg(interp, ap, double);
#if 0
    l->setJointAccelI(7, acceleration, acceleration, acceleration);
    auto radians = angle * M_PI/180.0;
    auto alpha = acceleration * M_PI/180.0;
    auto timeout = sqrt( 2*radians / alpha);
    l->moveAccel(1<<id, 0x07, 
        0, timeout, LINKBOT_JOINT_STATE_HOLD,
        0, timeout, LINKBOT_JOINT_STATE_HOLD,
        0, timeout, LINKBOT_JOINT_STATE_HOLD);
#endif
    l->accelJointAngleNB(id, acceleration, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot accelJointTimeNB*/
EXPORTCH void CLinkbot_accelJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double acceleration;
    double time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    acceleration=Ch_VaArg(interp, ap, double);
    time=Ch_VaArg(interp, ap, double);
    l->accelJointTimeNB(id, acceleration, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot accelJointToVelocityNB*/
EXPORTCH void CLinkbot_accelJointToVelocityNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double acceleration;
    double time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    acceleration=Ch_VaArg(interp, ap, double);
    unimplemented();
    /* TODO
    l->setJointAccelI(7, acceleration, acceleration, acceleration);
    time=Ch_VaArg(interp, ap, double);
    //l->accelJointToVelocityNB(id, acceleration, time);
    l->moveAccel(1<<id, 0x07, 
        0, time, LINKBOT_JOINT_STATE_MOVING,
        0, time, LINKBOT_JOINT_STATE_MOVING,
        0, time, LINKBOT_JOINT_STATE_MOVING);
    */
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot accelJointToMaxSpeedNB*/
EXPORTCH void CLinkbot_accelJointToMaxSpeedNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double acceleration;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    acceleration=Ch_VaArg(interp, ap, double);
    l->accelJointToMaxSpeedNB(id, acceleration);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbot moveToByTrackPos*/
EXPORTCH void CLinkbot_moveToByTrackPos_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double angle1;
    double angle2;
    double angle3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    angle1=Ch_VaArg(interp, ap, double);
    angle2=Ch_VaArg(interp, ap, double);
    angle3=Ch_VaArg(interp, ap, double);
    unimplemented();
    // TODO
    //l->moveToByTrackPos(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbot moveToByTrackPosNB*/
EXPORTCH void CLinkbot_moveToByTrackPosNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double angle1;
    double angle2;
    double angle3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    angle1=Ch_VaArg(interp, ap, double);
    angle2=Ch_VaArg(interp, ap, double);
    angle3=Ch_VaArg(interp, ap, double);
    unimplemented();
    // TODO
    //l->moveToByTrackPosNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointToByTrackPos*/
EXPORTCH void CLinkbot_moveJointToByTrackPos_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    angle=Ch_VaArg(interp, ap, double);
    unimplemented();
    // TODO
    //l->moveJointToByTrackPos(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointToByTrackPosNB*/
EXPORTCH void CLinkbot_moveJointToByTrackPosNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    angle=Ch_VaArg(interp, ap, double);
    unimplemented();
    // TODO
    //l->moveJointToByTrackPosNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot move*/
EXPORTCH void CLinkbot_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double j1;
    double j2;
    double j3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    j1=Ch_VaArg(interp, ap, double);
    j2=Ch_VaArg(interp, ap, double);
    j3=Ch_VaArg(interp, ap, double);
    l->move(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveNB*/
EXPORTCH void CLinkbot_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double j1;
    double j2;
    double j3;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    j1=Ch_VaArg(interp, ap, double);
    j2=Ch_VaArg(interp, ap, double);
    j3=Ch_VaArg(interp, ap, double);
    l->moveNB(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveWait*/
EXPORTCH void CLinkbot_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    int mask;    
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    mask=Ch_VaArg(interp, ap, int);
    l->moveWait(mask);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointWait*/
EXPORTCH void CLinkbot_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;    
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    l->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot isMoving*/
EXPORTCH int CLinkbot_isMoving_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    int mask;  
	int retval;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    mask=Ch_VaArg(interp, ap, int);
    unimplemented();
    /* TODO
    retval=l->isMoving(mask);
    */
    retval = -1;
    Ch_VaEnd(interp, ap);
    return retval;
}

/*linkbot isConnected*/
EXPORTCH int CLinkbot_isConnected_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	int retval;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    unimplemented();
    /* TODO
	retval = l->isConnected();
    */
	Ch_VaEnd(interp, ap);
	return retval;
}

/*linkbot stop*/
EXPORTCH void CLinkbot_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->stop();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot stopOneJoint*/
EXPORTCH void CLinkbot_stopOneJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    //l->stopOneJoint(id);
    l->stop(1<<id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot holdJoint*/
EXPORTCH void CLinkbot_holdJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
    //l->holdJoint(id);
    l->moveJointNB(LinkbotJoint(id), 0);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot holdJoints*/
EXPORTCH void CLinkbot_holdJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    //l->holdJoints();
    l->moveNB(0, 0, 0);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot holdJointsAtExit*/
EXPORTCH void CLinkbot_holdJointsAtExit_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	LinkbotJoint id;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    unimplemented();
    // TODO
	//l->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot relaxJoint*/
EXPORTCH void CLinkbot_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
    //l->relaxJoint(id);
    l->stop(1<<id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot relaxJoints*/
EXPORTCH void CLinkbot_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    //l->relaxJoints();
    l->stop();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveForeverNB*/
EXPORTCH void CLinkbot_moveForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbot moveJoint*/
EXPORTCH void CLinkbot_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
	double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}
/*linkbot moveJointNB*/
EXPORTCH void CLinkbot_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
	double angle;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointForeverNB*/
EXPORTCH void CLinkbot_moveJointForeverNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
    unimplemented();
    //l->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveTime*/
EXPORTCH void CLinkbot_moveTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	double time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	time=Ch_VaArg(interp, ap, double);
    l->moveTime(time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveTimeNB*/
EXPORTCH void CLinkbot_moveTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	double time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	time=Ch_VaArg(interp, ap, double);
    l->moveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointTime*/
EXPORTCH void CLinkbot_moveJointTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
	double time;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	time=Ch_VaArg(interp, ap, double);
    l->moveJointTime(id, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointTimeNB*/
EXPORTCH void CLinkbot_moveJointTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
	double time;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	time=Ch_VaArg(interp, ap, double);
    l->moveJointTimeNB(id, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointTo*/
EXPORTCH void CLinkbot_moveJointTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	LinkbotJoint id;
	double angle;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveJointToNB*/
EXPORTCH void CLinkbot_moveJointToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	LinkbotJoint id;
	double angle;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    l->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveTo*/
EXPORTCH void CLinkbot_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	double angle1;
	double angle2;
	double angle3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    l->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveToNB*/
EXPORTCH void CLinkbot_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	double angle1;
	double angle2;
	double angle3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    l->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveToZero*/
EXPORTCH void CLinkbot_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->moveTo(0,0,0);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot moveToZeroNB*/
EXPORTCH void CLinkbot_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->moveToNB(0,0,0);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getAccelerometerData*/
EXPORTCH void CLinkbot_getAccelerometerData_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *x;
    double *y;
	double *z;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    x=Ch_VaArg(interp, ap, double *);
    y=Ch_VaArg(interp, ap, double *);
	z=Ch_VaArg(interp, ap, double *);
    l->getAccelerometerData(*x, *y, *z);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getBatteryVoltage*/
EXPORTCH void CLinkbot_getBatteryVoltage_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	double *voltage;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	voltage = Ch_VaArg(interp, ap, double *);
	l->getBatteryVoltage(*voltage);
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot getLEDColorRGB*/
EXPORTCH void CLinkbot_getLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    int *r;
    int *g;
	int *b;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    r=Ch_VaArg(interp, ap, int *);
    g=Ch_VaArg(interp, ap, int *);
	b=Ch_VaArg(interp, ap, int *);
    l->getLEDColorRGB(*r, *g, *b);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getLEDColor*/
EXPORTCH void CLinkbot_getLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    char * color;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    color=Ch_VaArg(interp, ap, char *);
    l->getLEDColor(color);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbot getJointAngleInstant*/
EXPORTCH void CLinkbot_getJointAngleInstant_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *angle;
    LinkbotJoint id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    angle=Ch_VaArg(interp, ap, double *);
    unimplemented();
    //l->getJointAngleInstant(id, *angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointAnglesInstant*/
EXPORTCH void CLinkbot_getJointAnglesInstant_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *angle1;
    double *angle2;
    double *angle3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    angle1=Ch_VaArg(interp, ap, double *);
    angle2=Ch_VaArg(interp, ap, double *);
    angle3=Ch_VaArg(interp, ap, double *);
    unimplemented();
    //l->getJointAnglesInstant(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointAngle*/
EXPORTCH void CLinkbot_getJointAngle_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *angle;
    LinkbotJoint id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    angle=Ch_VaArg(interp, ap, double *);
    l->getJointAngle(id, *angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointAngles*/
EXPORTCH void CLinkbot_getJointAngles_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *angle1;
    double *angle2;
    double *angle3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    angle1=Ch_VaArg(interp, ap, double *);
    angle2=Ch_VaArg(interp, ap, double *);
    angle3=Ch_VaArg(interp, ap, double *);
    l->getJointAngles(*angle1, *angle2, *angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeed*/
EXPORTCH void CLinkbot_getJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *speed;
    LinkbotJoint id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    speed=Ch_VaArg(interp, ap, double *);
    l->getJointSpeed(id, *speed);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeedRatio*/
EXPORTCH void CLinkbot_getJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *ratio;
    LinkbotJoint id;


    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    ratio=Ch_VaArg(interp, ap, double *);
    l->getJointSpeedRatio(id, *ratio);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeeds*/
EXPORTCH void CLinkbot_getJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *speed1;
    double *speed2;
    double *speed3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    speed1=Ch_VaArg(interp, ap, double *);
    speed2=Ch_VaArg(interp, ap, double *);
    speed3=Ch_VaArg(interp, ap, double *);
    l->getJointSpeeds(*speed1, *speed2, *speed3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSpeedRatios*/
EXPORTCH void CLinkbot_getJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double *ratio1;
    double *ratio2;
    double *ratio3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    ratio1=Ch_VaArg(interp, ap, double *);
    ratio2=Ch_VaArg(interp, ap, double *);
    ratio3=Ch_VaArg(interp, ap, double *);
    l->getJointSpeedRatios(*ratio1, *ratio2, *ratio3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot getJointSafetyAngle*/
EXPORTCH void CLinkbot_getJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	double *angle;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	angle = Ch_VaArg(interp, ap, double *);
    unimplemented();
	//l->getJointSafetyAngle(*angle);
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot getJointSafetyAngleTimeout*/
EXPORTCH void CLinkbot_getJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	double *timeout;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	timeout = Ch_VaArg(interp, ap, double *);
    unimplemented();
	//l->getJointSafetyAngleTimeout(*timeout);
	Ch_VaEnd(interp, ap);
	return;
}

/*END GET FUNCTIONS*/
/*SET FUNCTIONS*/

/*linkbot setJointMovementStateNB*/
EXPORTCH void CLinkbot_setJointMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    robotJointState_t dir;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    dir=Ch_VaArg(interp, ap, robotJointState_t);
    l->setJointMovementStateNB(id, robotJointState_t2LinkbotDirection(dir));
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointMovementStateTime*/
EXPORTCH void CLinkbot_setJointMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    robotJointState_t dir;
    double seconds;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    dir=Ch_VaArg(interp, ap, robotJointState_t);
    seconds=Ch_VaArg(interp, ap, double);
    l->setJointMovementStateTime(id, robotJointState_t2LinkbotDirection(dir), seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeed*/
EXPORTCH void CLinkbot_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double speed;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    speed=Ch_VaArg(interp, ap, double);
    l->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeeds*/
EXPORTCH void CLinkbot_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double speed1;
    double speed2;
    double speed3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    speed1=Ch_VaArg(interp, ap, double);
    speed2=Ch_VaArg(interp, ap, double);
    speed3=Ch_VaArg(interp, ap, double);
    l->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeedRatio*/
EXPORTCH void CLinkbot_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    double ratio;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    ratio=Ch_VaArg(interp, ap, double);
    l->setJointSpeedRatio(id, ratio);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSpeedRatios*/
EXPORTCH void CLinkbot_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double ratio1;
    double ratio2;
    double ratio3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    ratio1=Ch_VaArg(interp, ap, double);
    ratio2=Ch_VaArg(interp, ap, double);
    ratio3=Ch_VaArg(interp, ap, double);
    l->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointPower*/
EXPORTCH void CLinkbot_setJointPower_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    LinkbotJoint id;
    int power;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    power=Ch_VaArg(interp, ap, int);
    l->setJointPower(id, power);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setMotorPowers*/
EXPORTCH void CLinkbot_setMotorPowers_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double p1;
    double p2;
    double p3;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    p1=Ch_VaArg(interp, ap, double);
    p2=Ch_VaArg(interp, ap, double);
    p3=Ch_VaArg(interp, ap, double);
    l->setMotorPowers(p1, p2, p3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setMovementStateNB*/
EXPORTCH void CLinkbot_setMovementStateNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    LinkbotDirection dir[3];

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    int i = 0;
    for ( auto d : {dir1, dir2, dir3} ) {
        dir[i] = robotJointState_t2LinkbotDirection(d);
        i++;
    }
    l->setMovementStateNB(dir[0], dir[1], dir[2]);
    Ch_VaEnd(interp, ap);
}

/*linkbot setMovementStateTime*/
EXPORTCH void CLinkbot_setMovementStateTime_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;

    LinkbotDirection dir[3];
    double seconds;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    int i = 0;
    for ( auto d : {dir1, dir2, dir3} ) {
        dir[i] = robotJointState_t2LinkbotDirection(d);
        i++;
    }
    seconds=Ch_VaArg(interp, ap, double);
    l->setMovementStateTime(dir[0], dir[1], dir[2], seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setMovementStateTime*/
EXPORTCH void CLinkbot_setMovementStateTimeNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    robotJointState_t dir1;
    robotJointState_t dir2;
    robotJointState_t dir3;
    double seconds;
    LinkbotDirection dir[3];

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    dir1=Ch_VaArg(interp, ap, robotJointState_t);
    dir2=Ch_VaArg(interp, ap, robotJointState_t);
    dir3=Ch_VaArg(interp, ap, robotJointState_t);
    int i = 0;
    for ( auto d : {dir1, dir2, dir3} ) {
        dir[i] = robotJointState_t2LinkbotDirection(d);
        i++;
    }
    seconds=Ch_VaArg(interp, ap, double);
    l->setMovementStateTimeNB(dir[0], dir[1], dir[2], seconds);
    Ch_VaEnd(interp, ap);
    return;
}


/*linkbot setBuzzerFrequnencyOn*/
EXPORTCH void CLinkbot_setBuzzerFrequencyOn_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    int frequency;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    frequency=Ch_VaArg(interp, ap, int);
    l->setBuzzerFrequencyOn(frequency);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setBuzzerFrequnencyOff*/
EXPORTCH void CLinkbot_setBuzzerFrequencyOff_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->setBuzzerFrequencyOff();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setBuzzerFrequnency*/
EXPORTCH void CLinkbot_setBuzzerFrequency_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    int frequency;
	double time;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    frequency=Ch_VaArg(interp, ap, int);
	time=Ch_VaArg(interp, ap, double);
    l->setBuzzerFrequency(frequency, time);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setJointSafetyAngle*/
EXPORTCH void CLinkbot_setJointSafetyAngle_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	double angle;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	angle = Ch_VaArg(interp, ap, double);
    unimplemented();
	//l->setJointSafetyAngle(angle);
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot setJointSafetyAngleTimeout*/
EXPORTCH void CLinkbot_setJointSafetyAngleTimeout_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	double timeout;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	timeout = Ch_VaArg(interp, ap, double);
    unimplemented();
	//l->setJointSafetyAngleTimeout(timeout);
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbot setLEDColorRGB*/
EXPORTCH void CLinkbot_setLEDColorRGB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	int r;
	int g;
	int b;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    r=Ch_VaArg(interp, ap, int);
	g=Ch_VaArg(interp, ap, int);
	b=Ch_VaArg(interp, ap, int);
    l->setLEDColorRGB(r, g, b);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot setLEDColor*/
EXPORTCH void CLinkbot_setLEDColor_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	char *color;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    color=Ch_VaArg(interp, ap, char*);
    l->setLEDColor(color);
    Ch_VaEnd(interp, ap);
    return;
}


/*END SET FUNCTIONS*/
/*MISCELLANEOUS FUNCTIONS*/

/*linkbot enableButtonCallback*/
EXPORTCH void CLinkbot_enableButtonCallback_chdl(void *varg) {
    return; 
    #if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    void *data;
    void (*cb)(void*, int, int);    

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    data=Ch_VaArg(interp, ap, void*);
    cb=(void(*)(void*, int, int))Ch_VaArg(interp, ap, void*);
    l->enableButtonCallback(data, cb);
    Ch_VaEnd(interp, ap);
    return;
    #endif
}

/*linkbot disableButtonCallback*/
EXPORTCH void CLinkbot_disableButtonCallback_chdl(void *varg) {
    return;
    #if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;

    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->disableButtonCallback();
    Ch_VaEnd(interp, ap);
    return;
    #endif
}

/*linkbot delaySeconds*/
EXPORTCH void CLinkbot_delaySeconds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    int seconds;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	seconds=Ch_VaArg(interp, ap, int);
    l->delaySeconds(seconds);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot systemTime*/
EXPORTCH void CLinkbot_systemTime_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
	double *time;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	time=Ch_VaArg(interp, ap, double *);
    l->systemTime(*time);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbot resetToZeroNB*/
EXPORTCH void CLinkbot_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot resetToZero*/
EXPORTCH void CLinkbot_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    
    Ch_VaStart(interp, ap, varg);
    
    l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    l->resetToZero();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbot recordAngleBegin*/
EXPORTCH void CLinkbot_recordAngleBegin_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	LinkbotJoint id;
	double** time;
	double** angle;
	double* angle1;
	double* angle2;
	double* angle3;
	double seconds;
	int shiftData;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id = Ch_VaArg(interp, ap, LinkbotJoint);
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
EXPORTCH void CLinkbot_recordAngleEnd_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	LinkbotJoint id;
	int *num;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id = Ch_VaArg(interp, ap, LinkbotJoint);
	num = Ch_VaArg(interp, ap, int*);
	l->recordAnglesEnd(*num);
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbot recordAnglesBegin*/
EXPORTCH void CLinkbot_recordAnglesBegin_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    double** time;
    double** angle1;
    double* ignored = 0;
    double** angle3;
    double seconds;
    int shiftData;

    Ch_VaStart(interp, ap, varg);

    l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
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
EXPORTCH void CLinkbot_recordAnglesEnd_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbot *l;
    int *num;

    Ch_VaStart(interp, ap, varg);

    l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    num = Ch_VaArg(interp, ap, int*);
    l->recordAnglesEnd(*num);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbot recordDistanceBegin*/
EXPORTCH void CLinkbot_recordDistanceBegin_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	LinkbotJoint id;
	double** time;
	double**distance;
	double radius;
	double seconds;
	int shiftData;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id = Ch_VaArg(interp, ap, LinkbotJoint);
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
EXPORTCH void CLinkbot_recordDistanceEnd_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	LinkbotJoint id;
	int *num;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	id = Ch_VaArg(interp, ap, LinkbotJoint);
	num = Ch_VaArg(interp, ap, int*);
	l->recordDistanceEnd(id, *num);
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbot enableRecordDataShift*/
EXPORTCH void CLinkbot_enableRecordDataShift_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	l->enableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbot disableRecordDataShift*/
EXPORTCH void CLinkbot_disableRecordDataShift_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	l->disableRecordDataShift();
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbot recordNoDataShift*/
EXPORTCH void CLinkbot_recordNoDataShift_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	l->recordNoDataShift();
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbot blinkLED*/
EXPORTCH void CLinkbot_blinkLED_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbot *l;
	double delay;
	int numBlinks;

	Ch_VaStart(interp, ap, varg);

	l = Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	delay = Ch_VaArg(interp, ap, double);
	numBlinks = Ch_VaArg(interp, ap, int);
	l->blinkLED(delay, numBlinks);
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/* GROUP FUNCTIONS */


/*linkbotGroup addRobot*/
EXPORTCH void CLinkbotGroup_addRobot_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    class barobo::CLinkbot *l;

    Ch_VaStart(interp, ap, varg);
   
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	l=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
    g->addRobot(*l);
    Ch_VaEnd(interp, ap);
}

/*linkbotGroup addRobots*/
EXPORTCH void CLinkbotGroup_addRobots_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    class barobo::CLinkbot *robots;
	int numRobots;

    Ch_VaStart(interp, ap, varg);
   
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	robots=Ch_VaArg(interp, ap, class barobo::CLinkbot *);
	numRobots=Ch_VaArg(interp, ap, int);
    //g->addRobots(robots, numRobots);
    for(auto i = 0; i < numRobots; i++) {
        g->addRobot(robots[i]);
    }
    Ch_VaEnd(interp, ap);
}

/*linkbotGroup connect*/
EXPORTCH void CLinkbotGroup_connect_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	int checkType;
	int type = 0;

    Ch_VaStart(interp, ap, varg);

    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->connect();
    Ch_VaEnd(interp, ap);

	checkType=g->checkFormFactor(type);
	if (checkType == -1){
		printf("WARNING: Not all the Linkbots in the group are Linkbot-Is.\nPlease check the Linkbots.\nExiting...\n");
	    exit(-1);
	}
#endif
}

/*linkbotGroup moveWait*/
EXPORTCH void CLinkbotGroup_moveWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->moveWait();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointWait*/
EXPORTCH void CLinkbotGroup_moveJointWait_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
    g->moveJointWait(id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup holdJoint*/
EXPORTCH void CLinkbotGroup_holdJoint_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    LinkbotJoint id;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    id=Ch_VaArg(interp, ap, LinkbotJoint);
    g->holdJoint(id);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup holdJoints*/
EXPORTCH void CLinkbotGroup_holdJoints_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
    
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->holdJoints();
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup holdJointsAtExit*/
EXPORTCH void CLinkbotGroup_holdJointsAtExit_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbotGroup *g;

	Ch_VaStart(interp, ap, varg);

	g = Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	g->holdJointsAtExit();
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbotGroup relaxJoint*/
EXPORTCH void CLinkbotGroup_relaxJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;

    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
    //g->relaxJoint(id);
    g->stop(1<<id);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup relaxJoints*/
EXPORTCH void CLinkbotGroup_relaxJoints_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;

    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    //g->relaxJoints();
    g->stop();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup stop*/
EXPORTCH void CLinkbotGroup_stop_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;

    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->stop();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveNB*/
EXPORTCH void CLinkbotGroup_moveNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double j1;
	double j2;
	double j3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	j1=Ch_VaArg(interp, ap, double);
	j2=Ch_VaArg(interp, ap, double);
	j3=Ch_VaArg(interp, ap, double);
    g->moveNB(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup move*/
EXPORTCH void CLinkbotGroup_move_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double j1;
	double j2;
	double j3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	j1=Ch_VaArg(interp, ap, double);
	j2=Ch_VaArg(interp, ap, double);
	j3=Ch_VaArg(interp, ap, double);
    g->move(j1, j2, j3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveForeverNB*/
EXPORTCH void CLinkbotGroup_moveForeverNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->moveForeverNB();
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveToNB*/
EXPORTCH void CLinkbotGroup_moveToNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveToNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveTo*/
EXPORTCH void CLinkbotGroup_moveTo_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveTo(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToZeroNB*/
EXPORTCH void CLinkbotGroup_moveToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->moveToNB(0,0,0);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToZero*/
EXPORTCH void CLinkbotGroup_moveToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->moveTo(0,0,0);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveToByTrackPosNB*/
EXPORTCH void CLinkbotGroup_moveToByTrackPosNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveToByTrackPosNB(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveToByTrackPos*/
EXPORTCH void CLinkbotGroup_moveToByTrackPos_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double angle1;
	double angle2;
	double angle3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	angle1=Ch_VaArg(interp, ap, double);
	angle2=Ch_VaArg(interp, ap, double);
	angle3=Ch_VaArg(interp, ap, double);
    g->moveToByTrackPos(angle1, angle2, angle3);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveJointNB*/
EXPORTCH void CLinkbotGroup_moveJointNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJoint*/
EXPORTCH void CLinkbotGroup_moveJoint_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJoint(id, angle);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup moveJointForeverNB*/
EXPORTCH void CLinkbotGroup_moveJointForeverNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
    g->moveJointForeverNB(id);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveJointToNB*/
EXPORTCH void CLinkbotGroup_moveJointToNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointToNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveJointTo*/
EXPORTCH void CLinkbotGroup_moveJointTo_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointTo(id, angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveJointToByTrackPosNB*/
EXPORTCH void CLinkbotGroup_moveJointToByTrackPosNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointToByTrackPosNB(id, angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveJointToByTrackPos*/
EXPORTCH void CLinkbotGroup_moveJointToByTrackPos_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double angle;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	angle=Ch_VaArg(interp, ap, double);
    g->moveJointToByTrackPos(id, angle);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup setJointSpeed*/
EXPORTCH void CLinkbotGroup_setJointSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double speed;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	speed=Ch_VaArg(interp, ap, double);
    g->setJointSpeed(id, speed);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setJointSpeedRatio*/
EXPORTCH void CLinkbotGroup_setJointSpeedRatio_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double ratio;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	ratio=Ch_VaArg(interp, ap, double);
    g->setJointSpeedRatio(id, ratio);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setJointSpeeds*/
EXPORTCH void CLinkbotGroup_setJointSpeeds_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double speed1;
	double speed2;
	double speed3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	speed1=Ch_VaArg(interp, ap, double);
	speed2=Ch_VaArg(interp, ap, double);
	speed3=Ch_VaArg(interp, ap, double);
    g->setJointSpeeds(speed1, speed2, speed3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setJointSpeedRatios*/
EXPORTCH void CLinkbotGroup_setJointSpeedRatios_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double ratio1;
	double ratio2;
	double ratio3;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	ratio1=Ch_VaArg(interp, ap, double);
	ratio2=Ch_VaArg(interp, ap, double);
	ratio3=Ch_VaArg(interp, ap, double);
    g->setJointSpeedRatios(ratio1, ratio2, ratio3);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setSpeed*/
EXPORTCH void CLinkbotGroup_setSpeed_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double speed;
	double radius;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	speed=Ch_VaArg(interp, ap, double);
	radius=Ch_VaArg(interp, ap, double);
    g->setSpeed(speed, radius);
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup setLEDColorRGB*/
EXPORTCH void CLinkbotGroup_setLEDColorRGB_chdl(void *varg) {
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbotGroup *g;
	int r;
	int gr;
	int b;

	Ch_VaStart(interp, ap, varg);

	g = Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	r = Ch_VaArg(interp, ap, int);
	gr = Ch_VaArg(interp, ap, int);
	b = Ch_VaArg(interp, ap, int);
	g->setLEDColorRGB(r, gr, b);
	Ch_VaEnd(interp, ap);
	return;
}

/*linkbotGroup setLEDColorRGB*/
EXPORTCH void CLinkbotGroup_setLEDColor_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbotGroup *g;
	char* color;

	Ch_VaStart(interp, ap, varg);

	g = Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	color = Ch_VaArg(interp, ap, char*);
	g->setLEDColor(color);
	Ch_VaEnd(interp, ap);
	return;
#endif
}

/*linkbotGroup resetToZero*/
EXPORTCH void CLinkbotGroup_resetToZero_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->resetToZero();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup resetToZeroNB*/
EXPORTCH void CLinkbotGroup_resetToZeroNB_chdl(void *varg) {
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
    g->resetToZeroNB();
    Ch_VaEnd(interp, ap);
    return;
}

/*linkbotGroup isMoving*/
EXPORTCH int CLinkbotGroup_isMoving_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	int mask;
	int retval;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	mask=Ch_VaArg(interp, ap, int);
    retval=g->isMoving(mask);
    Ch_VaEnd(interp, ap);
    return retval;
#endif
}

/*linkbotGroup isConnected*/
EXPORTCH int CLinkbotGroup_isConnected_chdl(void *varg) {
    unimplemented();
#if 0
	ChInterp_t interp;
	ChVaList_t ap;
	class barobo::CLinkbotGroup *g;
	int retval;

	Ch_VaStart(interp, ap, varg);

	g = Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	retval = g->isConnected();
	Ch_VaEnd(interp, ap);
	return retval;
#endif
}

/*linkbotGroup moveJointTime*/
EXPORTCH void CLinkbotGroup_moveJointTime_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	time=Ch_VaArg(interp, ap, double);
    g->moveJointTime(id, time);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveJointTimeNB*/
EXPORTCH void CLinkbotGroup_moveJointTimeNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	LinkbotJoint id;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	id=Ch_VaArg(interp, ap, LinkbotJoint);
	time=Ch_VaArg(interp, ap, double);
    g->moveJointTimeNB(id, time);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveTime*/
EXPORTCH void CLinkbotGroup_moveTime_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	time=Ch_VaArg(interp, ap, double);
    g->moveTime(time);
    Ch_VaEnd(interp, ap);
    return;
#endif
}

/*linkbotGroup moveTimeNB*/
EXPORTCH void CLinkbotGroup_moveTimeNB_chdl(void *varg) {
    unimplemented();
#if 0
    ChInterp_t interp;
    ChVaList_t ap;
    class barobo::CLinkbotGroup *g;
	double time;
	
    Ch_VaStart(interp, ap, varg);
    
    g=Ch_VaArg(interp, ap, class barobo::CLinkbotGroup *);
	time=Ch_VaArg(interp, ap, double);
    g->moveTimeNB(time);
    Ch_VaEnd(interp, ap);
    return;
#endif
}


