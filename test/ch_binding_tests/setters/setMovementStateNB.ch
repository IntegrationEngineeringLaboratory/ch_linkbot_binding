#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Moving forward for 3 seconds...");
    robot.setMovementStateNB(
        ROBOT_FORWARD,
        ROBOT_FORWARD,
        ROBOT_FORWARD
    );
    sleep(3);
    printf("Moving backward for 3 seconds...");
    robot.setMovementStateNB(
        ROBOT_BACKWARD,
        ROBOT_BACKWARD,
        ROBOT_BACKWARD
        );
    sleep(3);
    printf("Holding for 3 seconds...");
    robot.setMovementStateNB(
        ROBOT_HOLD,
        ROBOT_HOLD,
        ROBOT_HOLD
    );
    sleep(3);
    printf("Moving positive for 3 seconds...");
    robot.setMovementStateNB(
        ROBOT_POSITIVE,
        ROBOT_POSITIVE,
        ROBOT_POSITIVE
    );
    sleep(3);
    printf("Moving negative for 3 seconds...");
    robot.setMovementStateNB(
        ROBOT_NEGATIVE,
        ROBOT_NEGATIVE,
        ROBOT_NEGATIVE
    );
    sleep(3);

    robot.stop();

    return 0;
}

