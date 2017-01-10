#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    double angle;
    robot.getJointAngle(JOINT1, angle);
    printf("angle: %lf\n", angle);

    return 0;
}

