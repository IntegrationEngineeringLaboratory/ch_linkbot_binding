#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);
    
    printf("Setting buzzer to 440 for 1 second...\n");
    robot.setBuzzerFrequency(440, 1);

    return 0;
}

