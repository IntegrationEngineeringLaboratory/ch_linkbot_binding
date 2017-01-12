#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Moving Joint 1 forward for 3 seconds...");
    robot.setJointMovementStateNB(JOINT1, ROBOT_FORWARD);
    sleep(3);
    printf("Moving Joint 1 backward for 3 seconds...");
    robot.setJointMovementStateNB(JOINT1, ROBOT_BACKWARD);
    sleep(3);
    printf("Holding Joint 1 for 3 seconds...");
    robot.setJointMovementStateNB(JOINT1, ROBOT_HOLD);
    sleep(3);
    printf("Moving Joint 1 positive for 3 seconds...");
    robot.setJointMovementStateNB(JOINT1, ROBOT_POSITIVE);
    sleep(3);
    printf("Moving Joint 1 negative for 3 seconds...");
    robot.setJointMovementStateNB(JOINT1, ROBOT_NEGATIVE);
    sleep(3);

    printf("Moving Joint 3 forward for 3 seconds...");
    robot.setJointMovementStateNB(JOINT3, ROBOT_FORWARD);
    sleep(3);
    printf("Moving Joint 3 backward for 3 seconds...");
    robot.setJointMovementStateNB(JOINT3, ROBOT_BACKWARD);
    sleep(3);
    printf("Holding Joint 3 for 3 seconds...");
    robot.setJointMovementStateNB(JOINT3, ROBOT_HOLD);
    sleep(3);
    printf("Moving Joint 3 positive for 3 seconds...");
    robot.setJointMovementStateNB(JOINT3, ROBOT_POSITIVE);
    sleep(3);
    printf("Moving Joint 3 negative for 3 seconds...");
    robot.setJointMovementStateNB(JOINT3, ROBOT_NEGATIVE);
    sleep(3);

    robot.stop();

    return 0;
}
