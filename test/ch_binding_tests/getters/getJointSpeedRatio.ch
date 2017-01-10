#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    double speed;
    robot.getJointSpeedRatio(JOINT1, speed);
    printf("speed ratio: %lf\n", speed);

    return 0;
}

