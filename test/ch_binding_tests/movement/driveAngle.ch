#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveAngle(90)");
    robot.driveAngle(90);
    robot.stop();

    return 0;    
}
