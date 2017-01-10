#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Setting speed to 3in/sec and moving 6 inches...\n");
    robot.setSpeed(3, 3.5);
    robot.driveDistance(6, 3.5);

    return 0;    
}

