#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    int r, g, b;
    robot.getLEDColorRGB(r, g, b);
    printf("LED Color RGB: %d, %d, %d\n", r, g, b);

    return 0;
}

