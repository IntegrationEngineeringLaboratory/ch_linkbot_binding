#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    double x, y, z;
    robot.getAccelerometerData(x, y, z);
    printf("Accelerometer data: %lf, %lf, %lf\n", x, y, z);

    return 0;
}

