#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("driveAngleNB(90)");
    robot.driveAngleNB(90);
    robot.moveWait();
    robot.stop();

    return 0;    
}
