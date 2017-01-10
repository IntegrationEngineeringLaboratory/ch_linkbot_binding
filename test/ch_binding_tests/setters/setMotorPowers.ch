#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting motor powers to 128 for 3 seconds...\n");
    robot.setMotorPowers(128, 128, 128);
    sleep(3);
    robot.stop();

    return 0;
}

