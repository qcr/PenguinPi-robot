#include <iostream>
#include "vision.h"


Localiser :: Localiser () = default ; 	

int Localiser::compute_pose(Mat img, Pose2D * result){

    // Populate with dummy values for now 

    result->x=1.0;
    result->y=2.0;
    result->theta=3.0;

    return 0;
};



