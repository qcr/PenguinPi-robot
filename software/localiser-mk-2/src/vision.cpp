#include <iostream>
#include "vision.h"
using namespace cv; 

Localiser :: Localiser () {
}

int Localiser::compute_pose(Mat img, Pose2D * result){

    // Populate with dummy values for now 

    // # Homography 
    // src_points = np.array([[558, 6], [107, 5],[77, 473], [580, 474]])
    // dst_points = np.array([[0,0],[500,0],[500,500], [0,500]])
    // h, status = cv2.findHomography(src_points, dst_points)

    Mat homography = findHomography(srcPoints, dstPoints);

    result->x=1.0;
    result->y=2.0;
    result->theta=3.0;

    return 0;
}



