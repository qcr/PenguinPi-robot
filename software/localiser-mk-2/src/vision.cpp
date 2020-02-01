#include "vision.h"
#include "contours.h"

using namespace cv; 

Localiser :: Localiser () : 
    size(500,500), lower_bound(220), upper_bound(255), flipCode(1)  
    {

    const float in[] = {558,6,107,5,77,473,580,474};
    const float out[] = {0,0,500,0,500,500,0,500};

    for (int i=0; i<8; i+=2){

        Point2f input_point(in[i],in[i+1]);
        Point2f output_point(out[i],out[i+1]);

        srcPoints.push_back(input_point);
        dstPoints.push_back(output_point);
    }
    homography = findHomography(srcPoints, dstPoints);

}

int Localiser::compute_pose(Mat img, Pose2D * result){



    Mat img2, img3, mask, mask2;
    std::vector<Mat> robot_contours;
    
    cvtColor(img, img2, COLOR_BGR2GRAY);
    warpPerspective(img2, img3, homography, size); 
    inRange(img3, lower_bound, upper_bound, mask);
    flip(mask,mask2,flipCode);                          // Flip around y axis
    findContours(mask2, robot_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    Contours conts;
    conts.get_contours(robot_contours);
    std::vector<Box> boxes = conts.get_boxes();

    // Find outer limits of IR LEDs        
    uint min_x = 1e5;
    uint max_x = 0;
    uint min_y = 1e5;
    uint max_y = 0;

    for (auto box : boxes){
        if (box.cx < min_x){
            min_x = box.cx;
        }
        if (box.cx > max_x){
            max_x = box.cx;
        }
        if (box.cy < min_y){
            min_y = box.cy;
        }
        if (box.cy > max_y){
            max_y = box.cy;
        }
    }

    // Find the center LED
    Box * center_box = NULL;
    for (auto box : boxes){
        if (box.cx < max_x && box.cx > min_x && box.cy < max_y && box.cy > min_y){
            center_box = &box;
        }
    }

    // Found robot 
    if (center_box != NULL){


    } else { // found no robot
        result->x=0;
        result->y=0;
        result->theta=0;
    }



            

    return 0;
}


std::ostream & operator<<(std::ostream & os, const Localiser & localiser)
{
    os << "Homography source points:" << std::endl;
    for (auto i: localiser.srcPoints)
        os << i << " ";
    os << std::endl;
    os << "Homography destination points:" << std::endl;
    for (auto i: localiser.dstPoints)
        os << i << " ";
    os << std::endl;
    os << "Homography between points: " << std::endl;
    os << localiser.homography << std::endl;
    return os;
}