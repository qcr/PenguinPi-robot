#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/shm.h>
#include <sys/ipc.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>

#include "vision.h"

using namespace std;
using namespace cv;

#define	SEMAPHORE_KEY			(291623581)


int init(key_t * ShmKEY, int * ShmID){

    *(ShmKEY) = ftok("/etc", 'x');
    *(ShmID) = shmget(*(ShmKEY), sizeof(struct Pose2D), IPC_CREAT | 0666);
    if (*(ShmID) < 0){
        return -1;
    } else {
        return 0;
    }
}

int main(int argc, char * argv[]){

    Localiser localiser;

    // Use dummy image 
    Mat image;
    image = imread(argv[1], IMREAD_COLOR); 

    // Set up shared memory 
    key_t   ShmKEY;
    int     ShmID;
    Pose2D *ShmPTR;
    static int semaphore1_id;

     if (init(&ShmKEY, &ShmID) < 0) {
        cerr << "*** shmget error (server) ***" << endl;
        exit(1);
     }
     cout << "Localiser has set up shared memory with key " << ShmKEY << " ..." << endl;

     ShmPTR = (struct Pose2D *) shmat(ShmID, NULL, 0);
     if ((uintptr_t) ShmPTR == -1) {
        cerr << "*** shmat error (server) ***" << endl; 
        exit(1);
     }
     cout << "Localiser has attached the shared memory of " << sizeof(Pose2D) << " bytes ..." << endl;

  

    // Loop for now 
    while(1){
        Pose2D latest_pose;
        int result = localiser.compute_pose(image, &latest_pose);

        // Copy pose to shared memory 
        ShmPTR->x = latest_pose.x;
        ShmPTR->y = latest_pose.y;
        ShmPTR->theta = latest_pose.theta;

    }
}