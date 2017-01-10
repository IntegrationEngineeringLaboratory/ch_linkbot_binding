#include <linkbot.h>

int main()
{
    char serial_id[32];
    printf("Enter Robot ID: ");
    scanf("%s", serial_id);

    CLinkbotI robot = CLinkbotI(serial_id);

    printf("testing isConnected()");
    printf("isConnected should be 1: %d\n", robot.isConnected());

    return 0;    
}

