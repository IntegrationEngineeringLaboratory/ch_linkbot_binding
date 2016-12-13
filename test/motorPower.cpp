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

    std::cout << "Creating new linkbot..." << std::endl;
    Linkbot *l = new Linkbot();
    l->connectWithSerialID(argv[1]);
    std::cout << "sleeping..." << std::endl;
    sleep(2);
    std::cout << "Connecting..." << std::endl;
    l->connect();

    std::cout << "Setting motor power..." << std::endl;
    for(double p = -1; p < 1.0; p += 0.01) {
        l->setJointPower(ROBOT_JOINT1, p);
    }
#endif
    return 0;
}
