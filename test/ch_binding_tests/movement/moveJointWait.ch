#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Waiting for joint1 to stop moving...");
    robot.move(90, 30, 30);
    robot.moveJointWait(JOINT1);
    printf("Done.\n");
    robot.stop();

    return 0;    
}

