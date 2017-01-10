#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Moving forward for 3 seconds...\n");
    robot.setMovementStateTime(
        ROBOT_FORWARD,
        ROBOT_FORWARD,
        ROBOT_FORWARD,
        3
    );
    printf("Moving backward for 3 seconds...\n");
    robot.setMovementStateTime(
        ROBOT_BACKWARD,
        ROBOT_BACKWARD,
        ROBOT_BACKWARD,
        3
        );
    printf("Holding for 3 seconds...\n");
    robot.setMovementStateTime(
        ROBOT_HOLD,
        ROBOT_HOLD,
        ROBOT_HOLD,
        3
    );
    printf("Moving positive for 3 seconds...\n");
    robot.setMovementStateTime(
        ROBOT_POSITIVE,
        ROBOT_POSITIVE,
        ROBOT_POSITIVE,
        3
    );
    printf("Moving negative for 3 seconds...\n");
    robot.setMovementStateTime(
        ROBOT_NEGATIVE,
        ROBOT_NEGATIVE,
        ROBOT_NEGATIVE,
        3
    );

    robot.stop();

    return 0;
}

