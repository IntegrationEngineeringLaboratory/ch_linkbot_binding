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
    l->setJointMovementStateNB(ROBOT_JOINT1, ROBOT_FORWARD);
    sleep(3);
    l->setJointMovementStateNB(ROBOT_JOINT1, ROBOT_BACKWARD);
    sleep(3);

    l->setMovementStateTimeNB(ROBOT_FORWARD, ROBOT_FORWARD, ROBOT_FORWARD, 3);
    l->moveWait();

    return 0;
#endif
}

