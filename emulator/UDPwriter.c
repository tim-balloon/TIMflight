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
#define HOST "127.0.0.1"
#define PORT 6379

int main(int argc)
{
    redisContext *c = redisConnect(HOST, PORT);
    if (c != NULL && c->err) {
    printf("Error: %s\n", c->errstr);
    // handle error
    } else {
        printf("Connected to Redis\n");
    }
    
    float server_message;
    char key_name[] = "mykey";
    char value[20];
    redisReply *reply;

    // Data struct for header, data, and footer for each packet
    struct data {
        double value[8000];   // Array for storing all the data values
        int packetnum;
        double timestamp;
        char location_ip[20];
    } message;

    clock_t t_0 = clock();
    int j = 1;  //iterator
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
    client_address.sin_addr.s_addr = inet_addr(HOST);  //Sets address
    if (client_address.sin_addr.s_addr < 0) {
        printf("Error setting address!");
    }
    else if (client_address.sin_addr.s_addr == 0) {
        printf("Address is messed up!");
    }
    int sockaddr_size = sizeof(client_address);
    memset(&client_address.sin_zero, 0, sizeof(client_address.sin_zero));
    printf("Status: %s\n", strerror(errno));

    int counter = 0;    // The while loop for making data needs a stopping point that can be updated for each
    int stop = 0;       // packet. "counter" and "stop" are just iterators and stopping points for that loop.
                        // These are needed because I'm indexing the message.value array. If I could just append
                        // like Python does that would be MUCH simpler.
    while (strcmp(value, "1") != 0) {
        reply = redisCommand(c,"GET %s",key_name);    // while loop that waits until the reply from the redis server is 1
        strcpy(value,reply->str);
        printf("Sleeping...\n");
        usleep(100000);
    }

    reply = redisCommand(c,"SET %s %s",key_name,"0");

    for (message.packetnum = 1; message.packetnum < 9; message.packetnum++) {
        printf("In the for loop!\n");

        sleep(4);
        
        //Write message
        time_t t_i = clock();
        // Generating 5 random floats
        // int end_counter = counter;

        /* Intializes random number generator */
        //srand((unsigned) time(&t));   //Seed for rand() if I need it
        
        printf("counter = %d\n", counter);
        while (counter < stop+1000) {
            //number = rand() % 50 * .23;
            message.value[counter] = 2;
            //printf("counter = %d\n", counter);
            printf("message.value[counter] = %f\n", message.value[counter]);
            counter += 1;
        }

        stop = counter;

        message.timestamp = (double)(t_i - t_0) / 1000;
        char ip[] = "127.0.0.1";
        strcpy(message.location_ip, ip);

        //Send message
        if (sendto(my_socket, (struct data*)&message, message_size, flags, (struct sockaddr*)&client_address,
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
        printf("Location_ip = %s\n", message.location_ip);
        printf("packetnum = %d\n", message.packetnum);

    }

    //Close socket
    close(my_socket);

    return 0;
}