#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Moving forward for 3 seconds...");
    robot.setMovementStateTimeNB(
        ROBOT_FORWARD,
        ROBOT_FORWARD,
        ROBOT_FORWARD,
        3
    );
    robot.moveWait();
    printf("Moving backward for 3 seconds...");
    robot.setMovementStateTimeNB(
        ROBOT_BACKWARD,
        ROBOT_BACKWARD,
        ROBOT_BACKWARD,
        3
        );
    robot.moveWait();
    printf("Holding for 3 seconds...");
    robot.setMovementStateTimeNB(
        ROBOT_HOLD,
        ROBOT_HOLD,
        ROBOT_HOLD,
        3
    );
    robot.moveWait();
    printf("Moving positive for 3 seconds...");
    robot.setMovementStateTimeNB(
        ROBOT_POSITIVE,
        ROBOT_POSITIVE,
        ROBOT_POSITIVE,
        3
    );
    robot.moveWait();
    printf("Moving negative for 3 seconds...");
    robot.setMovementStateTimeNB(
        ROBOT_NEGATIVE,
        ROBOT_NEGATIVE,
        ROBOT_NEGATIVE,
        3
    );
    robot.moveWait();

    robot.stop();

    return 0;
}

