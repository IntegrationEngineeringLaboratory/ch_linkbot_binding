#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    double distance;
    robot.getDistance(distance, 3.5);
    printf("Distance: %lf\n", distance);

    return 0;
}

