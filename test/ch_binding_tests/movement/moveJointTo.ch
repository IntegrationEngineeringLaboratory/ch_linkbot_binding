#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Resetting to zero...\n");
    robot.resetToZero();
    printf("Moving joint 1 to 90 degrees...\n");
    robot.moveJointTo(JOINT1, 90);
    printf("Done.\n");
    robot.stop();

    return 0;    
}

