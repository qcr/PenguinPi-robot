
#include <iostream>

#include <penguinpi/sock_srvr.h>

using namespace std;

UDPServer :: UDPServer () {}

// : server_addr(), client_addr(), 
SocketServer :: SocketServer (socksrvconf * config) {
    configure(config);
}

int UDPServer :: configure(socksrvconf * config){

    buflen = config->buflen;
    clientlen = sizeof(client_addr);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strcpy(server_addr.sun_path, config->sun_path);
    buf = new char[buflen+1];
    return 0;
}

int UDPServer :: connect(void){

    sockfd = socket(server_addr.sun_family, SOCK_STREAM, 0);
    if (!sockfd) {
        cout << "Failed to get socket file descriptor" << endl;
        return -1;
    }

    /*
    int enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        cout << "setsockopt(SO_REUSEADDR) failed" << endl;
        return -1;
    }*/

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(struct sockaddr_un))) {
        cout << "Failed to bind socket" << endl;
        return -1;
    }

    if (listen(sockfd,BACKLOG)){
        cout << "Failed to set socket as passive" << endl;
        return -1;
    };

    return 0;

}

int UDPServer :: wait_for_request () {

    clientlen = sizeof(client_addr);
    clientfd = accept(sockfd, (struct sockaddr *) &client_addr, &clientlen);
    if (clientfd < 0) {
        cout << ("ERROR on accept") << endl;
        return -1;
    }

    bzero(buf,buflen);
    int n = read(clientfd,buf,buflen);
    if (n < 0) {
        cout << "ERROR reading from socket" << endl;
        return -1;
    }
    else {
        #ifdef DEBUG
        cout << "Message from socket: " << buf << endl;
        #endif
        return 0;
    }
}

int SocketServer :: pack_response(void * data){
    bzero(buf,buflen);
    memcpy(buf,data,buflen);
    return 0;
}

int SocketServer :: send_response(){
    send(clientfd,buf,buflen,0);
    return 0;
}

SocketServer :: ~SocketServer () {
    delete buf;
    close(sockfd);
}

std::ostream & operator<<(std::ostream & os, const SocketServer & sock)
{
    os << std::endl << "Socketserver:" << std::endl;
    os << "Server with family " << sock.server_addr.sun_family << " at " << sock.server_addr.sun_path << std::endl;
    return os;
}



#define PORT     2115 
#define MAXLINE 1024 



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