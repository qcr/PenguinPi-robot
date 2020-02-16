#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include "localiser.h"

using namespace std;

int main(int argc, char * argv[]){

    PenguinPi::Localiser localiser;

    localiser.init_networking();

    cout << localiser << endl;

    while(1){

        int request = localiser.listen(); 

        switch(request){
            case LOC_REQ_GET_POSE:{
                localiser.send_pose();
                break;
            }

            case LOC_REQ_SAVE_POSE_IMG: {
                localiser.save_pose_img();
                break;
            }

            case LOC_REQ_SAVE_CAM_IMG: {
                localiser.save_camera_img();
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

            default: std::cerr << "Request to localiser not recognised: " << request << std::endl;
            break;
        }
    }
}


