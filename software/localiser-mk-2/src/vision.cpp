#include "vision.h"
#include "contours.h"


using namespace cv; 
using namespace std;


Localiser :: Localiser () : 

    #ifdef CAMERA
    camera(),
    #endif 
    camera_image(), pose_image(), cartesian_size(500,500), lower_bound(220), upper_bound(255), flipCode(1), camera_save_timer(0)  
    {

    #ifdef CAMERA

    camera.set(CAP_PROP_FORMAT, CV_8UC1);
    camera.set(CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CAP_PROP_FRAME_HEIGHT, 480);
    cout << "Opening camera.. " << endl; 
    if (!camera.open()) { cerr << "Error opening camera " << endl; }
    #else 
    camera_image = cv::imread("/var/www/EGB439/console/arena.jpg", IMREAD_GRAYSCALE);
    #ifdef DEBUG 
    cout << "Displaying image... " << endl;
    cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", camera_image );                   // Show our image inside it.
    waitKey(0);                                          // Wait for a keystroke in the window
    #endif 
    #endif 


    #ifdef DEBUG 
    cout << "Setting up homography transform..." << endl;
    #endif
    const float in[] = {107,5,558,6,580,474,77,473};
    const float out[] = {0,0,500,0,500,500,0,500};

    for (int i=0; i<8; i+=2){

        Point2f input_point(in[i],in[i+1]);
        Point2f output_point(out[i],out[i+1]);

        srcPoints.push_back(input_point);
        dstPoints.push_back(output_point);
    }
    homography = findHomography(srcPoints, dstPoints);
}

Localiser :: ~Localiser(){

    #ifdef CAMERA
    camera.release();
    #endif 
}


int Localiser::update_camera_img(void){

    #ifdef CAMERA
    Mat img;
    camera.grab();
    camera.retrieve(camera_image);
    //cvtColor(img, camera_image, COLOR_BGR2GRAY);
    #endif 
    return 0;
}

int Localiser::save_pose_img(void){

    

    if (!camera_save_timer){
        Mat out_img; 
        //cvtColor(camera_image, out_img, cv::COLOR_GRAY2BGR);
        cv::imwrite("/var/www/EGB439/camera/get/arena.jpg",pose_image);
    }    
    camera_save_timer++;
    return 0;
}


int Localiser::compute_pose(Pose2D * result){

    Mat registered_img, mask;
    std::vector<Mat> robot_contours;

    warpPerspective(camera_image, registered_img, homography, cartesian_size); 
    
    #ifdef DEBUG 
    cout << "Displaying registered image... " << endl;
    cv::namedWindow( "Registered image", WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Registered image", registered_img );                   // Show our image inside it.
    waitKey(0);  
    #endif 

    inRange(registered_img, lower_bound, upper_bound, mask);
    //flip(mask,mask2,flipCode);                          // Flip around y axis
    findContours(mask, robot_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    #ifdef DEBUG 
    cout << "Displaying LED mask... " << endl;
    cv::namedWindow( "LED mask", WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "LED mask", mask );                   // Show our image inside it.
    waitKey(0);  
    #endif 

    Contours conts;
    conts.get_contours(robot_contours);
    std::vector<Box> boxes = conts.get_boxes();

    uint NUM_BOXES = boxes.size();

    #ifdef DEBUG
    cout << "num boxes: " << NUM_BOXES << endl;
    #endif 

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
    // Should I get a pointer or an index?
    std::size_t center_box_ix = INIT_VAL;
    for(std::size_t i=0; i<boxes.size(); ++i){
        if (boxes[i].cx < max_x && boxes[i].cx > min_x && boxes[i].cy < max_y && boxes[i].cy > min_y){
            center_box_ix = i;
        }
    }
    
    // Found robot 
    if (center_box_ix != INIT_VAL){

        #ifdef DEBUG
        cout << "Found robot!" << endl;
        #endif

        // Store the distance from each box to the center box 
        float dists_boxes[NUM_BOXES];
        std::memset(dists_boxes, INIT_VAL, NUM_BOXES*sizeof(float));

        for(std::size_t i=0; i<boxes.size(); ++i){
            if (center_box_ix != i){
                std::complex<float> a(boxes[center_box_ix].cx, boxes[center_box_ix].cy);
                std::complex<float> b(boxes[i].cx, boxes[i].cy);
                std::complex<float> diff(a-b);
                float dist_box = std::abs(diff);
                dists_boxes[i] = dist_box;
            }
        }

        // Get the two closest boxes 

        int min_indices[2] = {0,1}; // min_indices[0] points to the closest box, ..[1] to the second closest

        int temp;

        // Base case - ensure min_dist < min_dist_2
        if (dists_boxes[min_indices[0]] > dists_boxes[min_indices[1]]){
            temp = min_indices[0];
            min_indices[0] = min_indices[1];
            min_indices[1] = temp;
        }

        for(int i=0; i<NUM_BOXES; i++){

            // Case -0. ith distance is greater than min_dist and min_dist 2
            // do nothing

            // case 1. ith distance is less than min_dist and min_dist 2
            if (dists_boxes[i] < dists_boxes[min_indices[0]]){

                min_indices[1] = min_indices[0];
                min_indices[0] = i;
            // case 2. ith distance is less than only min_dist_2
            } else if (dists_boxes[i] < dists_boxes[min_indices[1]]){
                min_indices[1] = i;
            }
        }

        // Use point between the two closest LEDs and center of middle LED to find angle
        #ifdef DEBUG
        cout << "Found indices: " << min_indices[0] << "," << min_indices[1] << endl << endl;
        #endif

        float mid_point_x = (boxes[min_indices[0]].cx + boxes[min_indices[1]].cx)/2.0;
        float mid_point_y = (boxes[min_indices[0]].cy + boxes[min_indices[1]].cy)/2.0;
        
        float angle_rad = atan2((boxes[center_box_ix].cy - mid_point_y), (boxes[center_box_ix].cx-mid_point_x));

        float angle_deg = -angle_rad*180.0/M_PI;
        float x = (boxes[center_box_ix].cx / 500.0)*2.0;
        float y = 2.0 - (boxes[center_box_ix].cy / 500.0)*2.0; // convert from img coords to cartesian

        int arrow_len_pixels = 35;
        pose_image = registered_img.clone();
        Point pt1(round(boxes[center_box_ix].cx), round(boxes[center_box_ix].cy));
        Point pt2(round(boxes[center_box_ix].cx + arrow_len_pixels * cos(angle_rad)), round(boxes[center_box_ix].cy+ arrow_len_pixels* sin(angle_rad)));
        cv::arrowedLine(pose_image, pt1, pt2, 200);

        #ifdef DEBUG
        cout << "x,y,theta: " << x << "," << y << "," << angle_deg  << endl;
        
       //Mat arrow_img = camera_image.clone();

        cv::namedWindow( "Img with arrow", WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "img with arrow", pose_img );                   // Show our image inside it.
        waitKey(0);    
        #endif

        result->x = x;
        result->y = y;
        result->theta = angle_deg;

    } else { // found no robot

        #ifdef DEBUG
        cout << "Did not find robot!" << endl;
        #endif

        result->x=0;
        result->y=0;
        result->theta=0;
    }
    return 0;
}

std::ostream & operator<<(std::ostream & os, const Localiser & localiser)
{
    os << std::endl << "Homography source points:" << std::endl;
    for (auto i: localiser.srcPoints)
        os << i << " ";
    os << std::endl;
    os << "Homography destination points:" << std::endl;
    for (auto i: localiser.dstPoints)
        os << i << " ";
    os << std::endl;
    os << "Homography between points: " << std::endl;
    os << localiser.homography << std::endl << std::endl;
    return os;
}
