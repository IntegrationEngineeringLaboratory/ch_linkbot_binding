#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Moving joint1 for 5 seconds...");
    robot.moveJointTimeNB(JOINT1, 5);
    robot.moveWait();
    robot.stop();

    return 0;    
}

