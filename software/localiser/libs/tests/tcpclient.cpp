#include <iostream>

#include <netdb.h> 
#include <stdio.h> 
#include <unistd.h>
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 

#define MAX 80 
#define PORT 2115
#define SA struct sockaddr 

#define RATE_TO_US(R_HZ)  ((1.0/R_HZ)*1e6)

#define CLIENT_RATE_HZ         (10)
#define MSGLEN 256


int main() 
{ 


    char buff[MSGLEN]; 
    int n; 
    const char * message = "hello";

    while(1){
        int sockfd, connfd; 
        struct sockaddr_in servaddr, cli; 
    
        // socket create and varification 
        sockfd = socket(AF_INET, SOCK_STREAM, 0); 
        if (sockfd == -1) { 
            printf("socket creation failed...\n"); 
            exit(0); 
        } 
        else
            printf("Socket successfully created..\n"); 
        bzero(&servaddr, sizeof(servaddr)); 
    
        // assign IP, PORT 
        servaddr.sin_family = AF_INET; 
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(PORT); 
    
        // connect the client socket to server socket 
        if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) { 
            std::cerr << "connection with the server failed..." << std::endl; 
            return -1; 
        } 
        else
            std::cout << "Connected" << std::endl;
    

        bzero(buff, sizeof(buff)); 
        memcpy(buff, message, sizeof(message));
        write(sockfd, buff, sizeof(buff)); 
        bzero(buff, sizeof(buff)); 
        read(sockfd, buff, sizeof(buff)); 
        printf("From Server : %s", buff); 

                // close the socket 
        close(sockfd);

        usleep(RATE_TO_US(CLIENT_RATE_HZ));

    }
} 
