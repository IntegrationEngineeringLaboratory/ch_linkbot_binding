#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("Accel joint angle 30deg/s/s, 3 seconds\n");
    robot.accelJointTimeNB(JOINT1, 30, 3);
    robot.moveWait();

    return 0;    
}

