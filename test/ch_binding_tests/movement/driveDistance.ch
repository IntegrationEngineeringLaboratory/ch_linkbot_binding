#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveDistance(6inches, 3.5 inch radius)");
    robot.driveDistance(6, 3.5);
    robot.stop();

    return 0;    
}

