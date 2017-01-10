#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    robot.moveNB(90, 90, 90);
    printf("Moving all joints 90 degrees...");
    printf("Done.\n");
    robot.stop();

    return 0;    
}

