#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting Joint 1 power to 128 for 2 seconds...\n");
    robot.setJointPower(JOINT1, 128);
    sleep(2);
    robot.stop();
    printf("Setting Joint 3 power to 128 for 2 seconds...\n");
    robot.setJointPower(JOINT3, 128);
    sleep(2);
    robot.stop();

    return 0;
}

