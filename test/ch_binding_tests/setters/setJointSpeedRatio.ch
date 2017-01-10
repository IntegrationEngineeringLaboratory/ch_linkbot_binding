#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting Joint speed ratio to 0.5...\n");
    robot.setJointSpeedRatio(JOINT1, 0.5);
    robot.moveJoint(JOINT1, 90);

    return 0;
}

