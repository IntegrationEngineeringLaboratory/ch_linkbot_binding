#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#define sleep(x) Sleep((x)*1000)
#endif

#include <iostream>
//#include "linkbot_wrapper.hpp"

int main(int argc, char *argv[]) {
#if 0
    if(argc != 2) {
        std::cout << "Usage: " << argv[0] << " <serial_id>"<<std::endl;
        return 0;
    }

    Linkbot *l = new Linkbot();
    l->connectWithSerialID(argv[1]);
    sleep(2);
    l->connect();

    std::cout << "Testing joint speeds..." << std::endl;
    /* setJointSpeed */
    l->setMovementStateNB(
            ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_FORWARD);
    for(double s = 45; s > -45; s -= 1.0) {
        l->setJointSpeed(ROBOT_JOINT1, s);
    }
    l->stop();
    std::cout<< "Done." << std::endl;
    sleep(2);

    std::cout << "Testing motor power..." << std::endl;
    /* setMotorPower */
    for(double power = 0; power < 1.0; power += 0.02) {
        l->setJointPower(ROBOT_JOINT1, power);
    }

    /* Set joint movement state */
    l->setJointMovementStateTime(ROBOT_JOINT1, ROBOT_FORWARD, 3);
#endif
    return 0;
}
