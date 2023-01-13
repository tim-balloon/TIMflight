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
//#define SERVER_ADDR "10.192.186.249"
#define SERVER_ADDR "192.168.1.121"   // Mac address at home
//#define SERVER_ADDR "127.0.0.1"
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

    float server_message;   //message recieved from server, will just be packetnum
    char key_name[] = "mykey";  //Redis DB keyname for executing command
    char value[20];   //holds value of key_name

    redisReply *reply;

    int j = 1;   //iterator

    /* Create socket */
    printf("Creating socket...\n");
    int my_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    printf("Status: %s\n", strerror(errno));

    printf("Initializing server address...\n");
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(2000);
    server_address.sin_addr.s_addr = inet_addr(SERVER_ADDR);
    if (server_address.sin_addr.s_addr < 0) {
        printf("Error setting address!\n");
    }
    else if (server_address.sin_addr.s_addr == 0) {
        printf("Address is messed up!");
    }
    int sockaddr_size = sizeof(server_address);
    memset(&server_address.sin_zero, 0, sizeof(server_address.sin_zero));
    printf("Status: %s\n", strerror(errno));

    struct timeval t, t_0;      // set up time checking

    /* Continuously check Redis DB for command */
    while (strcmp(value, "1") !=0 ) {
        reply = redisCommand(c,"GET %s", key_name);
        strcpy(value, reply->str);
        printf("Sleeping...\n");
        usleep(100000);
    }

    /* Reset command key to 0 */
    reply = redisCommand(c, "SET %s %s", key_name, "0");

    int increasing_number = 0;

    /* Send data */
    //for (message.packetnum = 1; message.packetnum < 9; message.packetnum++) {
    for (int i=0; i<5; ++i) {
        printf("In for loop...\n");
        gettimeofday(&t_0, NULL);

        /* Data struct for storing message */
        struct data {
            double value[5];
            int packetnum;
            char location_ip[20];
            //int timestamp;
        } message;

        memset(message.value, 0, 5*sizeof(double));
        printf("Values in value array before looping: \n");
        for (int k=0; k<5; k++) {
            printf("value[%d] = %f\n", k, message.value[k]);
        }

        message.packetnum = 1;
        int message_size = sizeof(message);
        int reply_size = sizeof(server_message);

        while(1 == 1) {
            /* Generate data */
            int counter = 0;   // "counter" is for the while loop that adds the fake data
            printf("message.packetnum = %d\n", message.packetnum);
            while (counter < 5) {
                message.value[counter] = increasing_number;
                printf("message.value[%d] = %f\n", counter, message.value[counter]);
                counter++;
                increasing_number++;
            }

            strcpy(message.location_ip, SERVER_ADDR);

            /* Send data */
            if (sendto(my_socket, (struct data*)&message, message_size, 0, (struct sockaddr*)&server_address,
                sockaddr_size) < 0) {
                printf("Failed to send\n");
                printf("The last error message was: %s\n", strerror(errno));
                return -1;
            }
            /* Recieve reply from server */
            if (recvfrom(my_socket, &server_message, reply_size, 0,
                        (struct sockaddr*)&server_address, &sockaddr_size) < 0) {
                printf("Error when receiving reply\n");
                printf("The last error message is: %s\n", strerror(errno));        
            }

            if (message.packetnum == 122) {      // send packets at 122 Hz
                printf("Ready to sleep!\n");
                gettimeofday(&t, NULL);
                int timedif = t_0.tv_usec - t.tv_usec;
                printf("Time remaining: %ld us\n", 1000000 - t.tv_usec);
                usleep(1000000 - t.tv_usec);
                break;
            }

            message.packetnum++;
            printf("increasing_number: %d\n", increasing_number);

            /* What is response from server? */
            printf("Server response is: %f\n", server_message);
            // printf("Location_ip = %s\n", message.location_ip);
            // printf("packetnum = %d", message.packetnum);

            printf("--------------------------------------------------\n");
        }
        printf("Values in value array after looping: \n");
        for (int k=0; k<5; k++) {
            printf("value[%d] = %f\n", k, message.value[k]);
        }   
    }

    /* Close socket */
    close(my_socket);

    return 0;

}