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

#define GETSOCKETERRNO() (errno)

int main(int argc)
{
    float server_message;
    
    // Data struct for header, data, and footer for each packet
    struct data {
        double value[1100];
        int i;
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

    int counter = 0;
    message.i = 1;
    while (message.i < 31) {
        //Write message
        time_t t_i = clock();
        // Generating 5 random floats
        double number;
        int end_counter = counter;
            
        /* Intializes random number generator */
        //srand((unsigned) time(&t));   //Seed for rand() if I need it
        while (counter < end_counter+5) {
            number = rand() % 50 * .23;
            message.value[counter] += number;
            counter += 1;
        }
        printf("counter = %d\n", counter);
        printf("end_counter = %d\n", end_counter);

        for (int spot=0; spot<counter; spot++) {
            printf("Value %d: %f\n", spot, message.value[spot]);
        }

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
        printf("Location_ip = %s", message.location_ip);

        message.i += 1;

    }

    //Close socket
    close(my_socket);

    return 0;
}
