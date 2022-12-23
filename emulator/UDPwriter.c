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
#define SERVER_ADDR "127.0.0.1"
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

    /* Data struct for storing message */
    struct data {
        double value[1000];
        int packetnum;
        char location_ip[20];
        //int timestamp;
    } message;

    int j = 1;   //iterator

    int message_size = sizeof(message);
    int reply_size = sizeof(server_message);
    unsigned int flags = 0;

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

    int counter = 0;
    int stop = 0;
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

    gettimeofday(&t, NULL);     // Initial time (start of packet sending)
    message.packetnum = 1;      // Start with first packet

    /* Send data */
    //for (message.packetnum = 1; message.packetnum < 9; message.packetnum++) {
    while(t.tv_sec - t_0.tv_sec <= 1) {

        gettimeofday(&t, NULL);

        /* Generate data */
        printf("counter = %d\n", counter);
        while (counter < stop+5) {
            message.value[counter] = 3;
            printf("message.value[%d] = %f\n", counter, message.value[counter]);
            printf("message.packetnum = %d\n", message.packetnum);
            counter += 1;
        }
        
        stop = counter;

        strcpy(message.location_ip, SERVER_ADDR);

        /* Send data */
        if (sendto(my_socket, (struct data*)&message, message_size, flags, (struct sockaddr*)&server_address,
            sockaddr_size) < 0) {
            printf("Failed to send\n");
            printf("The last error message was: %s\n", strerror(errno));
            return -1;
        }
        /* Recieve reply from server */
        if (recvfrom(my_socket, &server_message, reply_size, flags,
                    (struct sockaddr*)&server_address, &sockaddr_size) < 0) {
            printf("Error when receiving reply\n");
            printf("The last error message is: %s\n", strerror(errno));        
        }

        if (message.packetnum == 122) {
            printf("Ready to sleep!\n");
            printf("Time remaining: %d\n", 1000000 - t.tv_usec);
            usleep(1000000 - t.tv_usec);
            break;
        }

        message.packetnum++;

        /* What is response from server? */
        // printf("Server response is: %f\n", server_message);
        // printf("Location_ip = %s\n", message.location_ip);
        // printf("packetnum = %d", message.packetnum);
    }

    /* Close socket */
    close(my_socket);

    return 0;

}