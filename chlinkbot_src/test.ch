/* Sample code to test the Ch binding*/

#include"linkbot.h"

double radius = 1.75;
double seconds = 5;
double angle=90;
double distance=5;
double trackwidth = 3.69;
double t;
double angle1, angle2, angle3;
int r, g, b;
string_t color;

//CLinkbotI robot;
CLinkbotIGroup group;
printf("addRobot\n");
group.addRobot("SRS8");
group.addRobot("TP51");
printf("connect\n");
group.connect();
//group.driveForeverNB();
group.driveDistance(5, radius);
group.turnLeftNB(90, radius, trackwidth);
group.driveDistance(5, radius);
group.turnRightNB(90, radius, trackwidth);
group.moveWait();








