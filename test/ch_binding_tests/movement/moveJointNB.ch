#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Moving joint1 90 degrees...");
    robot.moveJointNB(JOINT1, 90);
    robot.moveWait();
    printf("Moving joint2 90 degrees...");
    robot.moveJointNB(JOINT2, 90);
    robot.moveWait();
    printf("Moving joint3 90 degrees...");
    robot.moveJointNB(JOINT3, 90);
    robot.moveWait();
    robot.stop();

    return 0;    
}

