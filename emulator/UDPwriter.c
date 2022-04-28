#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <errno.h>
#include <unistd.h>

#define GETSOCKETERRNO() (errno)

int main(int argc)
{
    // char message[2000];         //Use these two lines for sending/recieving strings
    // char server_message[2000];
    float server_message;
    float message;
    int message_size = sizeof(message);
    int reply_size = sizeof(server_message);
    unsigned int flags=0;

    //Create socket
    printf("Creating socket...\n");
    int my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    printf("Status: %s\n", strerror(errno));

    //Initialize server address
    printf("Initializing server address...\n");
    struct sockaddr_in client_address;   //ina = internet address
    client_address.sin_family = AF_INET;  //Use IPv4
    client_address.sin_port = htons(2000);  //Port number (it's gotta be in network order, hence htons()!)
    client_address.sin_addr.s_addr = inet_addr("127.0.0.1");  //Sets address
    if (client_address.sin_addr.s_addr < 0) {
        printf("Error setting address!");
    }
    else if (client_address.sin_addr.s_addr == 0) {
        printf("Address is messed up!");
    }
    int sockaddr_size = sizeof(client_address);
    memset(&client_address.sin_zero, 0, sizeof(client_address.sin_zero));
    printf("Status: %s\n", strerror(errno));

    do {
        //Write message
        printf("Enter a float to be sent: ");
        scanf("%f", &message);

        //Send message
        if (sendto(my_socket, &message, message_size, flags, (struct sockaddr*)&client_address,
            sockaddr_size) < 0) {
            printf("Failed to send\n");
            printf("The last error message is: %s\n", strerror(errno));
            return -1;
        }

        //Receive reply from server
        if (recvfrom(my_socket, &server_message, reply_size, flags, 
                    (struct sockaddr*)&client_address, &sockaddr_size) < 0) {
            printf("Error when receiving reply\n");
            printf("The last error message is: %s\n", strerror(errno));
            return -1;
        }

        //What's the response from the server?
        printf("Server response is: %f\n", server_message);

    }
    while (message != 0);

    //Close socket
    close(my_socket);

    return 0;
}
