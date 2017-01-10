#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Moving forever for 5 seconds...");
    robot.moveForeverNB();
    sleep(5);
    robot.stop();

    return 0;    
}

