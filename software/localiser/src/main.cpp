#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include "localiser.h"
#include "sock_srvr.h"

using namespace std;

void print_help(char * argv[]){
    cerr << "Usage: " << argv[0] << " <www-data-gid> " << "<socket address> " << endl;
}

int main(int argc, char * argv[]){

    if (argc != 3){
        print_help(argv);
        return -1;
    }

    socksrvconf config;
    gid_t app_group = atoi(argv[1]);
    config.app_group = app_group;
    std::strcpy(config.sun_path, argv[2]);
    config.buflen = sizeof(PenguinPi::Pose2D);

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
        sock.pack_response(&latest_pose);

        // Send data on the socket 
        sock.send_response();

        // Save the pose image periodically
        if (!(camera_save_timer % 25)){
            localiser.save_pose_img();
        }
        camera_save_timer++;
        
    }

}


