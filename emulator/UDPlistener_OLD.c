#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>

#define GETSOCKETERRNO() (errno)
//#define SERVER_ADDR "10.194.48.49"
//#define SERVER_ADDR "10.192.186.249"      // THIS IS THE IP OF THIS COMPUTER...HOW DOES THIS WORK???????
#define SERVER_ADDR "192.168.1.121"

int main()
{
    struct data {
        double value[8001];    // Array to store the values
        int packetnum;
        double timestamp;
        char location_ip[20];
        char destination_ip[20];
    } message;
    float reply;

    FILE * fileptr;

    //Create socket:
    printf("Creating socket...\n");
    int my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    printf("Status: %s\n", strerror(errno));

    //Initialize server address (loopback)
    printf("Initializing server address...\n");
    struct sockaddr_in client_address, listener_address;
    client_address.sin_family = AF_INET;
    client_address.sin_port = htons(2000);
    // client_address.sin_addr.s_addr = inet_addr("127.0.0.1");
    client_address.sin_addr.s_addr = inet_addr(SERVER_ADDR);
    int flags = 0;
    int client_address_size = sizeof(client_address);
    int listener_address_size = sizeof(listener_address);

    //Bind to port and IP
    printf("Binding to port and IP...\n");    //ADD BETTER ERROR HANDLING HERE
    bind(my_socket, (struct sockaddr *) &client_address, client_address_size);
    printf("Status: %s\n", strerror(errno));
    printf("Listening...\n");  //Maybe add what port/IP it's listening on?

    //Open file for saving data
    
    printf("Attempting file opening...\n");
    
    if ((fileptr = fopen("/Users/brendal4/Documents/newfile.bin", "wb")) == NULL) {
        printf("Error opening file.");
        exit(1);
    }

    //Revieve any incoming messages
    // If sending/recieving strings, use strlen(message) instead of sizeof(message)

    printf("Made it past file opening...\n");
    
    do {
        int recv_func = recvfrom(my_socket, (struct data*)&message, sizeof(message), flags, 
                                (struct sockaddr*)&client_address, &client_address_size);
        if (recv_func < 0) {
            printf("Failed to recieve message\n");
            return -1;
        }

        printf("Client says: packet %d at %lf from %s\n", message.packetnum, message.timestamp, message.location_ip);
        reply = message.packetnum;
        // char ip[] = "127.0.0.1";
        strcpy(message.destination_ip, SERVER_ADDR);

        //Send reply
        if (sendto(my_socket, &reply, sizeof(reply), flags, (struct sockaddr*)&client_address,
                    client_address_size) < 0) {
            printf("Failed to send reply\n");
            printf("The last error message is: %s\n", strerror(errno));
            return -1;
            }

        //Add data to file
        fwrite(&message, sizeof(struct data), 1, fileptr);
        }
    while (message.packetnum != 8);
    
    printf("Outside the dowhile loop/n");

    //Check value array:
//    for (int j=0; j<message.packetnum*1000; ++j) {
//        printf("Value %d: %f\n", j, message.value[j]);
//    }

    //Close socket
    close(my_socket);

    fclose(fileptr);

    return 0;
}
