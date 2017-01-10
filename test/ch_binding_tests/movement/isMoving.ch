#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("testing isMoving...");
    robot.moveNB(90, 90, 90);
    printf("ismoving should be 1: %d\n", robot.isMoving());
    robot.moveWait();
    printf("ismoving should be 0: %d\n", robot.isMoving());
    robot.stop();

    return 0;    
}

