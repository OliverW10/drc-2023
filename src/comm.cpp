#include "comm.hpp"
#include <chrono>
/* --- server.c --- */
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

const int port = 5000;
int sockfd;
sockaddr_in servaddr, cliaddr;
Message receive_buffer;

Message latest_message;
std::chrono::time_point<std::chrono::high_resolution_clock> latest_message_time
    = std::chrono::high_resolution_clock::now();

void startServer(){       
    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));
       
    // Filling server information
    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);
       
    // Bind the socket with the server address
    if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
            sizeof(servaddr)) < 0 )
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
       
    socklen_t len;
    int bytes_received;
   
    len = sizeof(cliaddr);  //len is value/result
   
    bytes_received = recvfrom(sockfd, &receive_buffer, sizeof(Message), 
                MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                &len);

    // take mutex
    latest_message = receive_buffer;
    // release mutex

    int packet_id = 999;
    printf("Recieved %d bytes from: %s\nt: %f, s: %f, e: %d",
        bytes_received, inet_ntoa(cliaddr.sin_addr),
        receive_buffer.speed, receive_buffer.turn, receive_buffer.enabled
    );
    // sendto(sockfd, &packet_id, sizeof(int), 
    //     MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
    //         len);
}


const double msg_max_age = 0.1;

bool getLatestMessage(Message& message) {
    // *wait for mutex*
    message = latest_message;
    double age_ms = std::chrono::duration_cast<std::chrono::milliseconds>( 
        std::chrono::high_resolution_clock::now() - latest_message_time 
    ).count();
    return age_ms / 1000 > msg_max_age;
}

CarState Message::toCarState(){
    if(enabled){
        return CarState{speed, turn};
    }else{
        return CarState{0, 0};
    }
}