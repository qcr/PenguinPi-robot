#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include "localiser.h"
#include "sock_srvr.h"

#define DOMAIN_SOCK_LEN     (128)

using namespace std;

int main(int argc, char * argv[]){

    socksrvconf config;
    std::strcpy(config.sun_path, "/var/run/penguinpi/localiser.sock");
    config.buflen = DOMAIN_SOCK_LEN;

    SocketServer sock(&config);

    cout << sock << endl;

    if(sock.connect()){
        cerr << "Socket failed to connect, exiting..." << endl;
        return -1;
    }

    PenguinPi::Localiser localiser;
    cout << localiser << endl;

    #ifdef DEBUG 
    cout << "Entering main program loop.." << endl;
    #endif

    uint8_t camera_save_timer = 0;

    while(1){

        PenguinPi::Pose2D latest_pose;

        // Wait for a request
        sock.wait_for_request();

        // Get a camera image and compute pose
        localiser.update_camera_img();
        localiser.compute_pose(&latest_pose);

        // Prepare response 
        // TODO PROPER SIZES
        char buffer[DOMAIN_SOCK_LEN];
        memset(buffer,0,DOMAIN_SOCK_LEN);
        sprintf(buffer,"{\"pose\":{\"x\":%f,\"y\":%f,\"theta\":%f}}\0",latest_pose.x,latest_pose.y,latest_pose.theta);
        sock.pack_response(buffer);

        // Send data on the socket 
        sock.send_response();

        // Save the pose image periodically
        if (!(camera_save_timer % 25)){
            localiser.save_pose_img();
        }
        camera_save_timer++;
        
    }

}


