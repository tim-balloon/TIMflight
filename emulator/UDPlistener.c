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
#include <stdint.h>
#include <hiredis/hiredis.h>

#define GETSOCKETERRNO() (errno)
//#define SERVER_ADDR "10.192.186.249"   //Mac address in lab
//#define SERVER_ADDR "172.16.145.245"
//#define SERVER_ADDR "192.168.1.121"  //Mac address at home
//#define SERVER_ADDR "169.254.147.48"
#define SERVER_ADDR "127.0.0.1"
#define UDP_PORT 2000
#define REDIS_PORT 6379

int main()
{
    
    /* Connecting to Redis */
    printf("Attempting to connect to Redis...\n");
    redisContext *c = redisConnect(SERVER_ADDR, REDIS_PORT);
    if (c != NULL && c->err) {
    printf("Error: %s\n", c->errstr);
    }
    else {
        printf("Connected to Redis\n");
    }
    redisReply *reply;
    char key_name[] = "stopkey";  //Redis DB keyname for executing command
    char key_value[20];   //holds value of key_name
    
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

    /* Setup time checking */
    //struct timeval recvtime_var;

    double latency_values[1000];

    while (strcmp(key_value, "1") != 0) {

        reply = redisCommand(c,"GET %s", key_name);
        strcpy(key_value, reply->str);

        /* Reset command key to 0 */
        reply = redisCommand(c, "SET %s %s", key_name, "0");


        /* DATA PACKET */
        struct data {
            double value[5];     //Array to store the values received from client
            int packetnum;
            char location_ip[20];   // where the message came from, will be IP of RFSoC board
            char destination_ip[20]; // where the message is going (this ip)
            struct timeval sendtime_var;
            struct timeval recvtime_var;
            double latency;
        } message;
        float reply;
        
        if (recvfrom(my_socket, (struct data*)&message, sizeof(message), flags, 
                    (struct sockaddr*)&server_address, &server_address_size) < 0) {
            printf("Failed to receive message\n");
            return -1;
        }
        gettimeofday(&message.recvtime_var, NULL);

        double sendtime_double = (double)message.sendtime_var.tv_sec + ((double)message.sendtime_var.tv_usec/1000000);
        double recvtime_double = (double)message.recvtime_var.tv_sec + ((double)message.recvtime_var.tv_usec/1000000);
        message.latency = recvtime_double - sendtime_double;

        printf("Received packet %d from %s at %f s\n", message.packetnum, message.location_ip, sendtime_double);
        printf("recvtime: %f s\n", recvtime_double);
        printf("Latency: %f s\n", message.latency);
        latency_values[packet_count] = message.latency;
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
    }

    printf("Total packets: %d\n", packet_count);

    double latency_values_sum = 0;
    printf("latency_values_sum = %f\n", latency_values_sum);
    for (int i=0; i<packet_count; ++i) {
        printf("latency_values[%d] = %f\n", i, latency_values[i]);
        latency_values_sum += latency_values[i];
    }

    double latency_values_avg = latency_values_sum / (double)packet_count;
    printf("latency_values_sum = %f\n", latency_values_sum);
    printf("Avg latency = %f\n", latency_values_avg);

    /* Close socket and file */
    close(my_socket);
    fclose(fileptr);

    return 0;

}