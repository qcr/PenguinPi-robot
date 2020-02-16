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
                localiser.pose_get();
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

            case LOC_REQ_POST_TIEPOINT: {
                break;
            }

            default: std::cerr << "Request to localiser not recognised!\n";
            break;
        }
    }
}


