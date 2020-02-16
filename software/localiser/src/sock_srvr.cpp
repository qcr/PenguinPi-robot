
#include <iostream>

#include "sock_srvr.h"

using namespace std;

SocketServer :: SocketServer (socksrvconf * config) : server_addr(), client_addr(), buflen(config->buflen) {

    clientlen = sizeof(client_addr);
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sun_family = AF_UNIX;
    strcpy(server_addr.sun_path, config->sun_path);
    buf = new char[buflen+1];

}

int SocketServer :: connect(void){

    sockfd = socket(server_addr.sun_family, SOCK_STREAM, 0);
    if (!sockfd) {
        cerr << "Failed to get socket file descriptor" << endl;
        return -1;
    }

    int enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
        cerr << "setsockopt(SO_REUSEADDR) failed" << endl;
        return -1;
    }

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(struct sockaddr_un))) {
        cerr << "Failed to bind socket" << endl;
        return -1;
    }

    if (listen(sockfd,BACKLOG)){
        cerr << "Failed to set socket as passive" << endl;
        return -1;
    };

    return 0;

}

int SocketServer :: wait_for_request () {

    clientlen = sizeof(client_addr);
    clientfd = accept(sockfd, (struct sockaddr *) &client_addr, &clientlen);
    if (clientfd < 0) {
        cerr << ("ERROR on accept") << endl;
    }

    bzero(buf,buflen);
    int n = read(clientfd,buf,buflen);
    if (n < 0) {
        cerr << "ERROR reading from socket" << endl;
    }
    else {
        cout << "Here is the message: " << buf << endl;
    }
}

int SocketServer :: pack_response(void * data){
    bzero(buf,buflen);
    memcpy(buf,data,buflen);
}

int SocketServer :: send_response(){
    send(clientfd,buf,buflen,0);
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