#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveForeverNB() for 3 seconds");
    robot.driveForeverNB();
    sleep(3)
    robot.stop();

    return 0;    
}
