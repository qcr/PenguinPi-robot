#include "vision.h"
#include "contours.h"


using namespace cv; 
using namespace std;


Localiser :: Localiser () : 

    #ifdef CAMERA
    camera(),
    #endif 
    camera_image(), pose_image(), cartesian_size(500,500), lower_bound(220), upper_bound(255), flipCode(1)   
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
    camera.grab();
    camera.retrieve(camera_image);
    //cvtColor(img, camera_image, COLOR_BGR2GRAY);
    #endif 
    return 0;
}

int Localiser::save_pose_img(void){

    // Collect test images
    //cv::imwrite("test_img.jpg",camera_image);
    //cvtColor(camera_image, out_img, cv::COLOR_GRAY2BGR);
    cv::imwrite("/var/www/EGB439/camera/get/arena.jpg",pose_image);
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

    // Save pose image without arrow in case localisation fails.
    // Calls to /camera/get must succeed even if localisation fails.
    pose_image = registered_img;

    //flip(mask,mask2,flipCode);                          // Flip around y axis
    findContours(mask, robot_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    for( int i = 0; i< robot_contours.size(); i++ ) {
       Scalar color = 100;
       drawContours( pose_image, robot_contours,  i, color);
     }

    #ifdef DEBUG 
    cout << "Displaying LED mask... " << endl;
    cv::namedWindow( "LED mask", WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "LED mask", mask );                   // Show our image inside it.
    waitKey(0);  
    #endif 

    Contours conts;
    conts.get_contours(robot_contours);
    std::vector<Led> leds = conts.get_leds();

    uint NUM_LEDS = leds.size();

    #ifdef DEBUG
    
    cout << "Contours:" << endl;
    for (auto contour : robot_contours){
        cout << contour << endl;
    }

    cout << "num leds found: " << NUM_LEDS << endl;
    for (auto led: leds){
        cout << led << endl;
    }
    #endif 

    // Find outer limits of IR LEDs        
    uint min_x = 1e5;
    uint max_x = 0;
    uint min_y = 1e5;
    uint max_y = 0;

    for (auto led : leds){
        if (led.cx < min_x){
            min_x = led.cx;
        }
        if (led.cx > max_x){
            max_x = led.cx;
        }
        if (led.cy < min_y){
            min_y = led.cy;
        }
        if (led.cy > max_y){
            max_y = led.cy;
        }
    }

    // Find the center LED
    // Should I get a pointer or an index?
    std::size_t center_led_ix = INIT_VAL;
    for(std::size_t i=0; i<leds.size(); ++i){
        if (leds[i].cx < max_x && leds[i].cx > min_x && leds[i].cy < max_y && leds[i].cy > min_y){
            center_led_ix = i;
        }
    }
    
    // Found robot 
    if (center_led_ix != INIT_VAL){

        #ifdef DEBUG
        cout << "Found robot!" << endl;
        #endif

        // Store the distance from each led to the center led 
        int dists_leds[NUM_LEDS];
        std::memset(dists_leds, INIT_VAL, NUM_LEDS*sizeof(float));

        for(std::size_t i=0; i<leds.size(); ++i){
            if (center_led_ix != i){
                int diff_x = leds[center_led_ix].cx - leds[center_led_ix].cy;
                int diff_y = leds[i].cx - leds[i].cy;
                float dist_led_float = sqrt((float)(pow( diff_x,2) + pow(diff_y,2)));
                int dist_led = (int) round(dist_led_float);
                dists_leds[i] = dist_led;
            }
        }

        // Get the two closest leds 

        int min_indices[2] = {0,1}; // min_indices[0] points to the closest led, ..[1] to the second closest

        int temp;

        // Base case - ensure min_dist < min_dist_2
        if (dists_leds[min_indices[0]] > dists_leds[min_indices[1]]){
            temp = min_indices[0];
            min_indices[0] = min_indices[1];
            min_indices[1] = temp;
        }

        for(int i=0; i<NUM_LEDS; i++){

            // Case -0. ith distance is greater than min_dist and min_dist 2
            // do nothing

            // case 1. ith distance is less than min_dist and min_dist 2
            if (dists_leds[i] < dists_leds[min_indices[0]]){

                min_indices[1] = min_indices[0];
                min_indices[0] = i;
            // case 2. ith distance is less than only min_dist_2
            } else if (dists_leds[i] < dists_leds[min_indices[1]]){
                min_indices[1] = i;
            }
        }

        // Use point between the two closest LEDs and center of middle LED to find angle
        #ifdef DEBUG
        cout << "Found two leds closest to center LED:" << std::endl;
        #endif

        int mid_point_x = round((leds[min_indices[0]].cx + leds[min_indices[1]].cx)/2.0);
        int mid_point_y = round((leds[min_indices[0]].cy + leds[min_indices[1]].cy)/2.0);
        
        float angle_rad = atan2((float)(leds[center_led_ix].cy - mid_point_y), (float)(leds[center_led_ix].cx-mid_point_x));

        float angle_deg = -angle_rad*180.0/M_PI;
        float x = ((float) leds[center_led_ix].cx / 500.0)*2.0;
        float y = 2.0 - ((float)leds[center_led_ix].cy / 500.0)*2.0; // convert from img coords to cartesian

        int arrow_len_pixels = 35;
        pose_image = registered_img.clone();
        Point pt1(leds[center_led_ix].cx, leds[center_led_ix].cy);
        Point pt2(leds[center_led_ix].cx + arrow_len_pixels * cos(angle_rad), leds[center_led_ix].cy+ arrow_len_pixels* sin(angle_rad));
        cv::arrowedLine(pose_image, pt1, pt2, 200);

        #ifdef DEBUG
        cout << "x,y,theta: " << x << "," << y << "," << angle_deg  << endl;
        
       //Mat arrow_img = camera_image.clone();

        cv::namedWindow( "Img with arrow", WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "img with arrow", pose_image );                   // Show our image inside it.
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
        return 1;
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
