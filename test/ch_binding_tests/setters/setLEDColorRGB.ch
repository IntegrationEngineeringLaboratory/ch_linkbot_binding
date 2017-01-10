#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting LED to red for 2 seconds...\n");
    robot.setLEDColorRGB(255, 0, 0);
    sleep(2);

    printf("Setting LED to green for 2 seconds...\n");
    robot.setLEDColorRGB(0, 255, 0);
    sleep(2);

    printf("Setting LED to blue for 2 seconds...\n");
    robot.setLEDColorRGB(0, 0, 255);
    sleep(2);

    return 0;
}

