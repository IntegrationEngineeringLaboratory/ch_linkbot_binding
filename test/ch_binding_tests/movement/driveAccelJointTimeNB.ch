#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveAccelJointTime, 2inch/s/s accel, 3 seconds");
    robot.driveAccelJointTime(3.5, 2, 3);
    robot.stop();

    return 0;    
}

