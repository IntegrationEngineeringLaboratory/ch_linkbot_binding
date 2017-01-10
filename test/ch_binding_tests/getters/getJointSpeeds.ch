#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    double angles[3];
    robot.getJointSpeeds(angles[0], angles[1], angles[2]);
    printf("speeds: %lf, %lf, %lf\n", angles[0], angles[1], angles[2]);

    return 0;
}

