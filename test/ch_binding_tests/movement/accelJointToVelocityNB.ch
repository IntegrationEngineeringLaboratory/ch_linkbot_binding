#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Accel joint 30deg/s/s, to 90deg/s, run for 6 seconds\n");
    robot.accelJointToVelocityNB(JOINT1, 30, 90);
    sleep(6);
    robot.stop();

    return 0;    
}

