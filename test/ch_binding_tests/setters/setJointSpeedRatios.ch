#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting Joint speed ratios to 0.5...\n");
    robot.setJointSpeedRatios(0.5, 0.5, 0.5);
    robot.move(90, 90, 90);

    return 0;
}

