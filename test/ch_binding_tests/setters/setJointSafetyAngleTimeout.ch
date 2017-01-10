#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting Joint safety angle timeout to 1...\n");
    robot.setJointSafetyAngleTimeout(1);

    return 0;
}

