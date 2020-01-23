#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <unistd.h>
#include <stdint.h>
#include <mutex>          // std::mutex

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include "vision.h"

using namespace std;
using namespace cv;
using namespace boost::interprocess;





int main(int argc, char * argv[]){

    Localiser localiser;

    // Use dummy image 
    Mat image;
    image = imread(argv[1], IMREAD_COLOR); 

    try{
        struct shm_remove
        {
            shm_remove() { shared_memory_object::remove("MySharedMemory"); }
            ~shm_remove(){ shared_memory_object::remove("MySharedMemory"); }
        } remover;

        shared_memory_object shm_obj
            (create_only,"MySharedMemory",read_write);
    
        size_t ShmSize = sizeof(SharedPose);
        shm_obj.truncate(ShmSize);

        //Map the memory 
        mapped_region mpdregion(shm_obj, read_write);

        //Get the address of the mapped region
        void * addr = mpdregion.get_address();

        cout << "Set up " << ShmSize << " bytes of memory at" << addr << endl;

        //Construct the shared structure in memory
        SharedPose * shared_data = new (addr) SharedPose;

        // Compute pose forever 
        while(1){

            Pose2D latest_pose;
            int result = localiser.compute_pose(image, &latest_pose);

            shared_data->mutex.lock();

            shared_data->pose.x = latest_pose.x;
            shared_data->pose.y = latest_pose.y;
            shared_data->pose.theta = latest_pose.theta;

            shared_data->mutex.unlock();

            usleep(10);
            
        }

    } catch(interprocess_exception &ex){
        std::cout << "Unexpected exception: " << ex.what() << std::endl;
        shared_memory_object::remove("shared_memory");
        return 1;
    }

    shared_memory_object::remove("shared_memory");

}