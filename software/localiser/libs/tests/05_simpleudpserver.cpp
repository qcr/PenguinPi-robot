#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <iostream>
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
  
#define PORT     2115 
#define MAXLINE 1024 

/*
 * Test a udp server in c++.
 * To test, run this program, and in another terminal:
 * 
 * $ echo "hello" | socat - udp:0.0.0.0:2115 
 *
 * This program will stream forever until it is interrupted with a sigint or sigkill. (Even if the client quits)
 * During this time, no one else may use the port 2115.
 * Eg, the client may not reconnect until this stream is closed and reopened.
 * 
 */
  

int main() { 
    int sockfd; 
    char buffer[MAXLINE]; 
    struct sockaddr_in servaddr, cliaddr; 
      
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
    servaddr.sin_port = htons(PORT); 
      
    // Bind the socket with the server address 
    if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
            sizeof(servaddr)) < 0 ) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    socklen_t len;
    size_t n; 
  
    len = sizeof(cliaddr); 
  
    n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
                &len); 

    std::cout << "Started connection with client. Message from client: "<<  buffer << std::endl;
    uint16_t count = 0;

    memset(buffer, 0, 256);

    do { 
        count++;
        sprintf(buffer, "\rSent %d messages", count );
        usleep(5*1e4);
    } while( !(sendto(sockfd, buffer, strlen(buffer),  
            MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
                len) < 0));

    printf("\nConnection interrupted!");

      
    return 0; 
} 