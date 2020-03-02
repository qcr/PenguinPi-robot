#include <penguinpi.h>

#include <iostream>


/*
 * Tests a local TCP server.
 * 
 * To test:
 * $ ./simpletcpserver <port>
 * 
 * In another terminal/ computer on network:
 * 
 * 
 * 
 */


void print_usage( char ** argv){
    std::cout << "Usage: " << argv[0] << " <PORT>" << std::endl;
}

int main(int argc, char ** argv){

    if (argc < 2){
        print_usage(argv);
        return -1;
    }

    int port;
    
    port = atoi(argv[1]);

    std::cout << "Using port  "<< port << std::endl;

    if (port < 0){
        print_usage(argv);
        return -1;
    }

    size_t msglen = 256;

    PenguinPi::TCPServer stream(port, msglen);

    if (stream.connect() < 0){
        std::cerr << "Error connecting to stream " << std::endl;
    }
    
    char request[msglen];
    stream.connect(); 
    stream.getreq(request);

    std::cout << "Received: " << request << std::endl;

    char msg[] = "boop";
    stream.sendmsg(msg, sizeof(msg));

    return 0;
}