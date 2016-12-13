#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#define sleep(x) Sleep((x)*1000)
#endif

#include <iostream>

int main(int argc, char *argv[]) {
    if(argc != 2) {
        std::cout << "Usage: " << argv[0] << " <serial_id>"<<std::endl;
        return 0;
    }
#if 0

    Linkbot *l = new Linkbot();
    l->connectWithSerialID(argv[1]);
	double seconds=5;
	double angle = 10;
	double angle1;

	l->getJointSafetyAngle(angle1);

	l->setJointSafetyAngle(angle);

	l->getJointSafetyAngle(angle1);
#endif
    /*l->move(90, 90, 90);
    l->move(-90, -90, -90);
    */
    /*
    l->setMovementStateTimeNB(
            ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_FORWARD, 3);
    l->moveWait();
    l->setMovementStateTimeNB(
            ROBOT_BACKWARD, ROBOT_BACKWARD, ROBOT_BACKWARD, 3);
    l->moveWait();
    */
    /*l->setMovementStateTimeNB(
            ROBOT_POSITIVE, ROBOT_POSITIVE, ROBOT_POSITIVE, 3);
    l->moveWait();
    l->setMovementStateTimeNB(
            ROBOT_NEGATIVE, ROBOT_NEGATIVE, ROBOT_NEGATIVE, 3);
    l->moveWait();

    std::cout << "Blinking LED 5 times...\n";
    l->blinkLED(0.3, 5);

    std::cout << "Forward\n";
    l->setJointMovementStateTime(ROBOT_JOINT1, ROBOT_FORWARD, 3);
    std::cout << "Backward\n";
    l->setJointMovementStateTime(ROBOT_JOINT1, ROBOT_BACKWARD, 3);
    std::cout << "Forward\n";
    l->setJointMovementStateTime(ROBOT_JOINT1, ROBOT_POSITIVE, 3);
    std::cout << "Backward\n";
    l->setJointMovementStateTime(ROBOT_JOINT1, ROBOT_NEGATIVE, 3);
    std::cout << "Done\n";

    std::cout << "Forward\n";
    l->setJointSpeed(ROBOT_JOINT1, 45);
    l->moveJointTime(ROBOT_JOINT1, 3);
    std::cout << "Backward\n";
    l->setJointSpeed(ROBOT_JOINT1, -45);
    l->moveJointTime(ROBOT_JOINT1, 3);

    std::cout << "moveTime 3 seconds\n";
    l->moveTime(3);

    #if 0
	l->driveTime(seconds);
	std::cout<<"movement 1 done"<<std::endl;
	l->driveTimeNB(seconds);
	std::cout<<"movement 2 done"<<std::endl;
    #endif
    double distance = 100;
    for(int i = 0; i < 10; i++) {
        std::cout << distance <<std::endl;
        l->moveNB(distance, distance, distance);
        l->moveWait();
        l->moveNB(-distance, -distance, -distance);
        l->moveWait();
        distance /= 2.0;
    }
    for(int i = 0; i < 10; i++) {
        distance *= 2.0;
        std::cout << distance <<std::endl;
        l->moveNB(distance, distance, distance);
        l->moveWait();
        l->moveNB(-distance, -distance, -distance);
        l->moveWait();
    }*/
    return 0;
}
