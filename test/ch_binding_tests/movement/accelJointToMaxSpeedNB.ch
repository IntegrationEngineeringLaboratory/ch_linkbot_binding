#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Accel joint 100deg/s/s, to 200deg/s, run for 6 seconds\n");
    robot.accelJointToMaxSpeedNB(JOINT1, 100);
    sleep(6);
    robot.stop();

    return 0;    
}

