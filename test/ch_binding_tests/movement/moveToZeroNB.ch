#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Moving to zero...\n");
    robot.resetToZero();
    printf("Moving all joints to 90 degrees...\n");
    robot.moveTo(90, 90, 90);
    robot.moveToZeroNB();
    printf("Moving all joints back to zero...\n");
    robot.moveWait();
    printf("Done.\n");
    robot.stop();

    return 0;    
}

