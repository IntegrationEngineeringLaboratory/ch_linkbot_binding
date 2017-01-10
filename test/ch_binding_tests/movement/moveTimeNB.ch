#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    robot.moveTimeNB(3);
    printf("Moving all joints 3 seconds");
    robot.moveWait();
    printf("Done.\n");
    robot.stop();

    return 0;    
}

