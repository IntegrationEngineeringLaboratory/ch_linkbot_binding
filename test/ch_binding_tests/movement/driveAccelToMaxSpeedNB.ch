#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveAccelToMaxSpeedNB, 2inch/s/s accel");
    robot.driveAccelToMaxSpeedNB(3.5, 2);
    time.sleep(6);
    robot.stop();

    return 0;    
}

