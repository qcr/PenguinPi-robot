#include <iostream>
#include <unistd.h>
#include <stdint.h>

#include "localiser.h"
#include "shmemkey.h"

using namespace std;
using namespace cv;
using namespace boost::interprocess;

int main(int argc, char * argv[]){

    PenguinPi::Localiser localiser;
     
    //Localiser localiser("/var/www/EGB439/console/arena.jpg");
    
    // Print localiser info 
    cout << localiser << endl;

    #ifdef DEBUG
    cout << "Setting up shared memory..." << endl;
    #endif 

    try{
        struct shm_remove
        {
            shm_remove() { shared_memory_object::remove(SHARED_MEMORY_KEY); }
            ~shm_remove(){ shared_memory_object::remove(SHARED_MEMORY_KEY); }
        } remover;

        shared_memory_object shm_obj(create_only,SHARED_MEMORY_KEY,read_write);
    
        size_t ShmSize = sizeof(SharedPose);
        shm_obj.truncate(ShmSize);

        //Map the memory 
        mapped_region mpdregion(shm_obj, read_write);

        //Get the address of the mapped region
        void * addr = mpdregion.get_address();

        cout << "Set up " << ShmSize << " bytes of memory at " << addr << endl;

        //Construct the shared structure in memory
        SharedPose * shared_data = new (addr) SharedPose;

        #ifdef DEBUG 
        cout << "Entering main program loop.." << endl;
        #endif

        uint8_t camera_save_timer = 0;
        // Compute pose forever 
        while(1){

            Pose2D latest_pose;

            localiser.update_camera_img();

            int result = localiser.compute_pose(&latest_pose);

            /* ~~~~~~ BEGIN CRITICAL SECTION ~~~~~~~~~ */

            shared_data->mutex.lock();

            shared_data->pose.x = latest_pose.x;
            shared_data->pose.y = latest_pose.y;
            shared_data->pose.theta = latest_pose.theta;

            shared_data->mutex.unlock();

            /* ~~~~~~ END CRITICAL SECTION ~~~~~~~~~ */

            // Save the pose image periodically
            if (!(camera_save_timer % 25)){
                localiser.save_pose_img();
            }
            camera_save_timer++;
            usleep(1000); // TODO set rate somewhere
            
        }

    } catch(interprocess_exception &ex){
        std::cout << "Unexpected exception: " << ex.what() << std::endl;
        shared_memory_object::remove("shared_memory");
        return 1;
    }

    shared_memory_object::remove("shared_memory");

}
