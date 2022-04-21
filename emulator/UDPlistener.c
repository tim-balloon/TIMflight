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

int main()
{
    //char message[2000];   //Use these 2 lines for sending strings
    //char reply[2000];

    float message[1000];
    float reply[1000];

    //Create socket:
    printf("Creating socket...\n");
    int my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    printf("Status: %s\n", strerror(errno));

    //Initialize server address (loopback)
    printf("Initializing server address...\n");
    struct sockaddr_in listener_address, client_address;
    listener_address.sin_family = AF_INET;
    listener_address.sin_port = htons(2000);
    listener_address.sin_addr.s_addr = inet_addr("127.0.0.1");
    int flags=0;
    int listener_address_size = sizeof(listener_address);
    int client_address_size = sizeof(client_address);

    //Bind to port and IP
    printf("Binding to port and IP...\n");    //ADD BETTER ERROR HANDLING HERE
    bind(my_socket, (struct sockaddr *) &listener_address, listener_address_size);
    printf("Status: %s\n", strerror(errno));
    printf("Listening...\n");  //Maybe add what port/IP it's listening on?

    //Revieve any incoming messages
    // If sending/recieving strings, use strlen(message) instead of sizeof(message)
    if (recvfrom(my_socket, &message, sizeof(message), flags, 
                (struct sockaddr*)&client_address, &client_address_size) < 0) {
        printf("Failed to recieve message\n");
        return -1;
    
    printf("Client says: %lf\n", message);

    //strcpy(reply, message);   //Use this line for strings
    for (int i=0; i<4; ++i) {
        reply[i] = message[i];
    }

    //Send reply
    if (sendto(my_socket, &reply, sizeof(reply), flags, (struct sockaddr*)&client_address,
                client_address_size) < 0) {
        printf("Failed to send reply\n");
        printf("The last error message is: %s\n", strerror(errno));
        return -1;
        }

    //Close socket
    close(my_socket);
}
