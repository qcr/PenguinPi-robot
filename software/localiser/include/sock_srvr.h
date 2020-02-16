#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <cstring>


#define BUFFER_SIZE             (1024)
#define MAX_SOCK_PATH_CHARS     (108)
#define MAX_UNAME_CHARS         (32)
#define BACKLOG                 (5)

typedef char unamestr[MAX_UNAME_CHARS];
typedef char sockpath[MAX_SOCK_PATH_CHARS];

struct socksrvconf {
    socksrvconf() : buflen(0) {};
    sockpath sun_path;
    size_t buflen;
};

class SocketServer {
    private:
        sockaddr_un server_addr;
        sockaddr_un client_addr;
        socklen_t clientlen;
        int sockfd;
        int clientfd;
    public:
        char * buf;
        int buflen;
        SocketServer () ;
        SocketServer (socksrvconf * config) ;
        int configure(socksrvconf * config) ;
        int connect(void);
        int wait_for_request(void); 
        int pack_response(void * data);
        int send_response(void);
        ~SocketServer ();
        friend std::ostream & operator<<(std::ostream & os, const SocketServer & sock);

};


