

/* 
 * tcpserver.c - A simple TCP echo server 
 * usage: tcpserver <port>
 */

#include <iostream>
#include <unistd.h>
#include <stdlib.h>

#include <string.h>

#include "tcp_srvr.h"
#include "penguinpi.h"

#define BUFSIZE 1024


namespace PenguinPi {

TCPServer :: TCPServer (int portno, size_t msglen) : msglen_(msglen) {

    sendbuf_ = new char(msglen_);

    parentfd_ = socket(AF_INET, SOCK_STREAM, 0);

    if (parentfd_ < 0){
        std::cerr << "Error opening socket" << std::endl;
    }

    int optval = 1;
    setsockopt(parentfd_, SOL_SOCKET, SO_REUSEADDR,
               (const void *)&optval, sizeof(int));

    /*
   * build the server's Internet address
   */
    bzero((char *)&serveraddr_, sizeof(serveraddr_));

    /* this is an Internet address */
    serveraddr_.sin_family = AF_INET;

    /* let the system figure out our IP address */
    serveraddr_.sin_addr.s_addr = htonl(INADDR_ANY);

    /* this is the port we will listen on */
    serveraddr_.sin_port = htons((unsigned short)portno);

    /* 
   * bind: associate the parent socket with a port 
   */
    if (bind(parentfd_, (struct sockaddr *)&serveraddr_,
             sizeof(serveraddr_)) < 0) {
                 std::cerr << "Error on binding" << std::endl;
             }
}

int TCPServer :: connect(){
        /* 
   * listen: make this socket ready to accept connection requests 
   */

   /* allow 5 requests to queue up */
    if (listen(parentfd_, 5) < 0)
        return -1;

    return 0;
}
    
/*
 * Service one incoming connection
 * 
 */

int TCPServer :: getreq(char * request) {

    clientlen_ = sizeof(clientaddr_);

    int n;
    char buf[BUFSIZE];             /* message buffer */

    /* 
    * accept: wait for a connection request 
    */
    childfd_ = accept(parentfd_, (struct sockaddr *)&clientaddr_, &clientlen_);
    if (childfd_ < 0){
        return -1;
    }

    /* 
    * gethostbyaddr: determine who sent the message 
    */
    hostp_ = gethostbyaddr((const char *)&clientaddr_.sin_addr.s_addr,
                            sizeof(clientaddr_.sin_addr.s_addr), AF_INET);
    if (hostp_ == NULL) {
        return -1;
    }

    hostaddrp_ = inet_ntoa(clientaddr_.sin_addr);

    if (hostaddrp_ == NULL){
        return -1;
    }

    /* 
    * read: read input string from the client
    */
    bzero(buf, BUFSIZE);
    n = read(childfd_, buf, BUFSIZE);

    if (n < 0) {
        return -1;
    }

    std::cout << "server received " << n << " bytes: " << buf << std::endl;

    // Return the request
    memcpy(request, buf, msglen_);

    return 0;
}


int TCPServer :: sendmsg(char * msg, size_t n){

/* 
    * write: echo the input string back to the client 
    */

   if (n > msglen_){
       std::cerr << "Message length " << n << " exceeds max buffer length " << msglen_ << std::endl;
       return -1;
   }

   std::cout << "Message to send: " << msg << " with " << n << " bytes " << std::endl;
   memcpy(sendbuf_, msg, n);

   std::cout << "Message ready: " << sendbuf_ << std::endl;

   int n_;
    n_ = write(childfd_, sendbuf_, n);
    if (n_ < 0){
        std::cerr << "Error writing to socket" << std::endl;
        return -1;
    }

    close(childfd_);
    return 0;
    
} 


TCPServer :: ~TCPServer () {

    // delete sendbuf_;
}
}

