#include <linkbot.h>

int main()
{
    char serial_id[32];
    CLinkbotIGroup group;
    CLinkbotI* robot = new CLinkbotI("ZRG6");
    group.addRobot(*robot);

    printf("Moving all joints 90 degrees...");
    group.move(90, 90, 90);
    printf("Done.\n");
    group.stop();

    return 0;    
}

