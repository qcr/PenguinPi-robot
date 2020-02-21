#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include "localiser.h"

using namespace std;

int main(int argc, char * argv[]){

    PenguinPi::Localiser localiser;

    localiser.init_networking();

    cout << localiser << endl;

    if (localiser.wait_for_stream() < 0){
        cout << "Failed to connect to stream" << endl;
        return -1;
    }

    while(1){    

        int request = localiser.listen(); 

        switch(request){
            case LOC_REQ_GET_POSE:{
                if (localiser.send_pose() < 0){
                    cout << "Failed to send pose" << endl;
                    return -1;
                }
                break;
            }

            case LOC_REQ_SAVE_POSE_IMG: {
                
                if (localiser.save_pose_img() < 0){
                    cout << "Failed to save pose img" << endl;
                    return -1;
                }
                break;
            }

            case LOC_REQ_SAVE_CAM_IMG: {
                if (localiser.save_camera_img() < 0){
                    cout << "Failed to save camera frame" << endl;
                    return -1;
                }
                break;
            }

            case LOC_REQ_GET_TIEPOINT: {
                localiser.send_tie_points();
                break;
            }

            case LOC_REQ_POST_TIEPOINT: {
                localiser.update_tie_point();
                cout << localiser << endl;
                break;
            }

            default: std::cout << "Request to localiser not recognised: " << request << std::endl;
            break;
        }
    }
    return 0;
}


