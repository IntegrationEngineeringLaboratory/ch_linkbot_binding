#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveBackward(90)");
    robot.driveBackwardNB(90);
    robot.moveWait();
    robot.stop();

    return 0;    
}

