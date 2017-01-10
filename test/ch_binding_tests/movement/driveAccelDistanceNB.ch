#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveAccelDistanceNB, 2inch/s/s accel, travel 6 inches");
    robot.driveAccelToMaxSpeedNB(3.5, 2, 6);
    robot.moveWait();
    robot.stop();

    return 0;    
}

