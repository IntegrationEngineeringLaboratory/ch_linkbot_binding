#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveAccelToVelocityNB, 2inch/s/s accel, 6inch/s");
    robot.driveAccelToVelocityNB(3.5, 2, 6);
    robot.moveWait();
    robot.stop();

    return 0;    
}

