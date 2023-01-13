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
#define SERVER_ADDR "192.168.1.121"  //Mac address at home
//#define SERVER_ADDR "127.0.0.1"
#define UDP_PORT 2000

int main()
{
    FILE * fileptr;    // pointer for bin file where message will be stored on local memory

    int packet_count = 1;

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
    for (int i=0; i<5; ++i) {
        printf("In the for loop...\n");

        while (1==1) {

            printf("In the while loop...\n");
            struct data {               //Struct for storing recieved message
                double value[5];     //Array to store the values recieved from client
                int packetnum;
                char location_ip[20];   // where the message came from
                char destination_ip[20]; // where the message is going (this ip)
            } message;
            float reply;
            
            if (recvfrom(my_socket, (struct data*)&message, sizeof(message), flags, 
                        (struct sockaddr*)&server_address, &server_address_size) < 0) {
                printf("Failed to recieve message\n");
                return -1;
            }
            printf("Client says: packet %d from %s\n", message.packetnum, message.location_ip);
            reply = message.packetnum;
            printf("Reply is: %f\n", reply);
            strcpy(message.destination_ip, SERVER_ADDR);
            printf("Packet_count = %d\n", packet_count);
            packet_count++;

            /* Check data in packet */
            printf("Values in value array: \n");
            for (int k=0; k<5; k++) {
                printf("value[%d] = %f\n", k, message.value[k]);
            }

            /* Send reply to client */
            if (sendto(my_socket, &reply, sizeof(reply), flags, (struct sockaddr*)&server_address, 
                        server_address_size) < 0) {
                printf("Failed to send reply\n");
                printf("The last error message is: %s\n", strerror(errno));
                return -1;
            }
            /* Write data to file */
            fwrite(&message, sizeof(struct data), 1, fileptr);

            if (message.packetnum == 122) {
                break;
            }
        }
    }
    printf("Total packets: %d", packet_count);

    /* Close socket and file */
    close(my_socket);
    fclose(fileptr);

    return 0;

}