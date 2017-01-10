#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    double v;
    robot.getBatteryVoltage(v);
    printf("Battery Voltage: %lf\n", v);

    return 0;
}

