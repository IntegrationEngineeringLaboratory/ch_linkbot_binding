#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    double timeout;
    robot.getJointSafetyAngleTimeout(timeout);
    printf("safety angle timeout: %lf\n", timeout);

    return 0;
}

