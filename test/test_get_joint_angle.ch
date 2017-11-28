
#include <linkbot.h>
char serialId[32];
printf("Enter robot serial ID: ");
scanf("%s", serialId);
CLinkbotI robot=CLinkbotI(serialId);
double angle;
robot.getJointAngle(1, angle);
printf("Angle: %lf\n", angle);
robot.getJointAngle(2, angle);
printf("Angle: %lf\n", angle);
robot.getJointAngle(3, angle);
printf("Angle: %lf\n", angle);
