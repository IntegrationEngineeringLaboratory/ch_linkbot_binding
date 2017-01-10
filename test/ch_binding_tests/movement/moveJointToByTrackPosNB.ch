#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    robot.moveJointToByTrackPosNB(JOINT1, 90);
    printf("moveJointToByTrackPos 90 degrees");
    robot.moveWait();
    printf("Done.\n");
    robot.stop();

    return 0;    
}

