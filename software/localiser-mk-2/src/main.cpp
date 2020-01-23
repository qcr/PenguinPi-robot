#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <unistd.h>
#include <stdint.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

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

        //Erase previous shared memory
        shared_memory_object::remove("shared_memory");

        shared_memory_object shm_obj
            (create_only               //open or create
            ,"shared_memory"              //name
            ,read_write                    //read-only mode
            );
    
        std::size_t ShmSize = sizeof(Pose2D);
        shm_obj.truncate(ShmSize);

        //Map the memory 
        mapped_region region
            ( shm_obj                      //Memory-mappable object
            , read_write               //Access mode
            );

        cout << "Set up " << ShmSize << " bytes of memory" << endl;

    } catch(interprocess_exception &ex){
        std::cout << "Unexpected exception: " << ex.what() << std::endl;
        shared_memory_object::remove("shared_memory");
        return 1;
    }
    // Loop for now 
    while(1){
        Pose2D latest_pose;
        int result = localiser.compute_pose(image, &latest_pose);
    }

    shared_memory_object::remove("shared_memory");

}