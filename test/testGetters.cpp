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

    l->move(90, 90, 90);
    std::cout << "Done moving." << std::endl;
    double arg[3];
    l->getDistance(arg[0], 3.5);
    std::cout << "getDistance: " << arg[0] << std::endl;
    
    l->getJointAngle(ROBOT_JOINT1, arg[0]);
    std::cout << "getJointAngle: " << arg[0] << std::endl;

    l->getJointAngles(arg[0], arg[1], arg[2]);
    std::cout << "getJointAngles: " << 
            arg[0] << " " <<
            arg[1] << " " <<
            arg[2] << " " << std::endl;

    l->getJointSpeed(ROBOT_JOINT1, arg[0]);
    std::cout << "getJointSpeed: " << arg[0] << std::endl;

    l->getJointSpeedRatio(ROBOT_JOINT1, arg[0]);
    std::cout << "getJointSpeedRatio: " << arg[0] << std::endl;

    l->getJointSpeeds(arg[0], arg[1], arg[2]);
    std::cout << "getJointSpeeds: " << 
            arg[0] << " " <<
            arg[1] << " " <<
            arg[2] << " " << std::endl;

    l->getJointSpeedRatios(arg[0], arg[1], arg[2]);
    std::cout << "getJointSpeedRatios: " << 
            arg[0] << " " <<
            arg[1] << " " <<
            arg[2] << " " << std::endl;
#endif
    return 0;
}
