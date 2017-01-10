#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveTimeNB() for 3 seconds");
    robot.driveTimeNB(3);
    sleep(5);
    robot.stop();

    return 0;    
}

