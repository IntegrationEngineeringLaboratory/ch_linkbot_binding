#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Resetting to zero...\n");
    robot.resetToZero();
    robot.moveJointToNB(JOINT1, 90);
    printf("Moving joint 1 to 90 degrees...\n");
    robot.moveWait();
    printf("Done.\n");
    robot.stop();

    return 0;    
}

