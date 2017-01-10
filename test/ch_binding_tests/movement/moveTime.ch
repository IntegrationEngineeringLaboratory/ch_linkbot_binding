#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Moving all joints 3 seconds");
    robot.moveTime(3);
    printf("Done.\n");
    robot.stop();

    return 0;    
}

