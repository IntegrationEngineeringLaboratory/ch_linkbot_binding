#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting Joint safety angle to 5...\n");
    robot.setJointSafetyAngle(5);

    return 0;
}

