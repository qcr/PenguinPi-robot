#include <stdlib.h>
#include <iostream>
#include <string.h>
#include "fcgio.h"
#include "pose.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

using namespace std;
using namespace boost::interprocess;

// Maximum bytes
const unsigned long STDIN_MAX = 1000000;

/**
 * Note this is not thread safe due to the static allocation of the
 * content_buffer.
 */
string get_request_content(const FCGX_Request & request) {
    char * content_length_str = FCGX_GetParam("CONTENT_LENGTH", request.envp);
    unsigned long content_length = STDIN_MAX;

    if (content_length_str) {
        content_length = strtol(content_length_str, &content_length_str, 10);
        if (*content_length_str) {
            std::cerr << "Can't Parse 'CONTENT_LENGTH='"
                 << FCGX_GetParam("CONTENT_LENGTH", request.envp)
                 << "'. Consuming stdin up to " << STDIN_MAX << endl;
        }

        if (content_length > STDIN_MAX) {
            content_length = STDIN_MAX;
        }
    } else {
        // Do not read from stdin if CONTENT_LENGTH is missing
        content_length = 0;
    }

    char * content_buffer = new char[content_length];
    std::cin.read(content_buffer, content_length);
    content_length = std::cin.gcount();

    // Chew up any remaining stdin - this shouldn't be necessary
    // but is because mod_fastcgi doesn't handle it correctly.

    // ignore() doesn't set the eof bit in some versions of glibc++
    // so use gcount() instead of eof()...
    do std::cin.ignore(1024); while (std::cin.gcount() == 1024);

    string content(content_buffer, content_length);
    delete [] content_buffer;
    return content;
}

int main(void) {
    // Backup the stdio streambufs
    streambuf * cin_streambuf  = std::cin.rdbuf();
    streambuf * cout_streambuf = std::cout.rdbuf();
    streambuf * cerr_streambuf = std::cerr.rdbuf();

    FCGX_Request request;

    FCGX_Init();
    FCGX_InitRequest(&request, 0, 0);

    try{
        shared_memory_object shm_obj
            (open_only               //open or create
            ,"shared_memory"              //name
            ,read_only                    //read-only mode
            );
        
        std::size_t ShmSize = sizeof(Pose2D);

        //Map the second half of the memory
        mapped_region region
            ( shm_obj                      //Memory-mappable object
            , read_only
            );

        std::cout << "Opened " << ShmSize << " bytes of memory" << endl;

    } catch(interprocess_exception &ex){
        std::cout << "Unexpected exception: " << ex.what() << std::endl;
        shared_memory_object::remove("shared_memory");
        return 1;
    }

    //Get the address of the mapped region
    void * addr = region.get_address();

    //Construct the shared structure in memory
    shared_memory_log * data = static_cast<shared_memory_log*>(addr);

    while (FCGX_Accept_r(&request) == 0) {
        fcgi_streambuf cin_fcgi_streambuf(request.in);
        fcgi_streambuf cout_fcgi_streambuf(request.out);
        fcgi_streambuf cerr_fcgi_streambuf(request.err);

        std::cin.rdbuf(&cin_fcgi_streambuf);
        std::cout.rdbuf(&cout_fcgi_streambuf);
        std::cerr.rdbuf(&cerr_fcgi_streambuf);

        const char * uri = FCGX_GetParam("REQUEST_URI", request.envp);

        string content = get_request_content(request);

        if (content.length() == 0) {
            content = ", World!";
        }

        //if (strcmp(uri,"/pose/get/")==0){
            // TODO json
        std::cout << "Content-type: text/html\r\n"
             << "\r\n"
             << "{\"pose\": {\"x\": 0, \"theta\": 0, \"y\": 0}}";
        //} 
    }

    // restore stdio streambufs
    std::cin.rdbuf(cin_streambuf);
    std::cout.rdbuf(cout_streambuf);
    std::cerr.rdbuf(cerr_streambuf);

    return 0;
}
