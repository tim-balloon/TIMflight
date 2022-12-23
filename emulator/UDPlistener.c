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
//#define SERVER_ADDR "10.192.186.249"
#define SERVER_ADDR "127.0.0.1"
#define UDP_PORT 2000

int main()
{
    struct data {               //Struct for storing recieved message
        double value[1000];     //Array to store the values recieved from client
        int packetnum;
        char location_ip[20];   // where the message came from
        char destination_ip[20]; // where the message is going (this ip)
    } message;
    float reply;

    FILE * fileptr;    // pointer for bin file where message will be stored on local memory

    /* Create socket: */
    printf("Creating socket...\n");
    int my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    printf("Status: %s\n", strerror(errno));

    /* Initialize server address */
    printf("Initializing server address...\n");
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(UDP_PORT);
    server_address.sin_addr.s_addr = inet_addr(SERVER_ADDR);
    unsigned int flags = 0;
    int server_address_size = sizeof(server_address);

    /* Bind to IP and port */
    printf("Binding to IP and port...\n");
    bind(my_socket, (struct sockaddr *) &server_address, server_address_size);
    printf("Status: %s\n", strerror(errno));
    printf("Listening on port %d...\n", UDP_PORT);

    /* Open a bin file for saving data */
    if ((fileptr = fopen("/Users/brendal4/Documents/newfile.bin", "wb")) == NULL) {
        printf("Error opening file.");
        exit(1);
    }

    /* Recieve incoming message */
    do {
        
        if (recvfrom(my_socket, (struct data*)&message, sizeof(message), flags, 
                    (struct sockaddr*)&server_address, &server_address_size) < 0) {
            printf("Failed to recieve message\n");
            return -1;
        }
        printf("Client says: packet %d from %s\n", message.packetnum, message.location_ip);
        reply = message.packetnum;
        strcpy(message.destination_ip, SERVER_ADDR);

        /* Send reply to client */
        if (sendto(my_socket, &reply, sizeof(reply), flags, (struct sockaddr*)&server_address, 
                    server_address_size) < 0) {
            printf("Failed to send reply\n");
            printf("The last error message is: %s\n", strerror(errno));
            return -1;
        }
        /* Write data to file */
        fwrite(&message, sizeof(struct data), 1, fileptr);
    }
    while (message.packetnum != 122);

    /* Close socket and file */
    close(my_socket);
    fclose(fileptr);

    return 0;

}