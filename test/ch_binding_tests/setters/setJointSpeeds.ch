#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting Joint speed to 30...\n");
    robot.setJointSpeeds(30, 30, 30);
    robot.move(90, 90, 90);

    return 0;
}

