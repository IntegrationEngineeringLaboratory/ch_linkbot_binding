#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Moving joint1 90 degrees...");
    robot.moveJoint(JOINT1, 90);
    printf("Moving joint2 90 degrees...");
    robot.moveJoint(JOINT2, 90);
    printf("Moving joint3 90 degrees...");
    robot.moveJoint(JOINT3, 90);
    robot.stop();

    return 0;    
}

