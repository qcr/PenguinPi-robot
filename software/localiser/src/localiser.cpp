#include "localiser.h"

using namespace std;

namespace PenguinPi {

Localiser :: Localiser () : 

    #ifdef CAMERA
    camera(),
    #endif 
    camera_image(), pose_image(), cartesian_size(500,500), lower_bound(220), upper_bound(255)
    {

    #ifdef CAMERA
    camera.set(CAP_PROP_FORMAT, CV_8UC1);
    camera.set(CAP_PROP_FRAME_WIDTH, PICAM_IMG_WIDTH);
    camera.set(CAP_PROP_FRAME_HEIGHT, PICAM_IMG_HEIGHT);
    cout << "Opening camera.. " << endl; 
    if (!camera.open()) { cerr << "Error opening camera " << endl; }
    #else 
    camera_image = cv::imread("/var/www/EGB439/console/arena.jpg", IMREAD_GRAYSCALE);
    #ifdef DEBUG 
    cout << "Displaying image... " << endl;
    cv::namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", camera_image );                   // Show our image inside it.
    cv::waitKey(0);                                          // Wait for a keystroke in the window
    #endif 
    #endif 


    #ifdef DEBUG 
    cout << "Setting up homography transform..." << endl;
    #endif
    const pixel_coord in[] = {107,5,558,6,580,474,77,473};
    const pixel_coord out[] = {0,0,ARENA_WIDTH_PIXELS,0,ARENA_WIDTH_PIXELS,ARENA_HEIGHT_PIXELS,0,ARENA_HEIGHT_PIXELS};

    for (int i=0; i<8; i+=2){

        cv::Point input_point(in[i],in[i+1]);
        cv::Point output_point(out[i],out[i+1]);

        srcPoints.push_back(input_point);
        dstPoints.push_back(output_point);
    }
    homography = findHomography(srcPoints, dstPoints);
}


int Localiser::compute_pose(Pose2D * result){

    // Apply homography to extract arena image from camera image
    cv::Mat registered_img, mask;
    std::vector<cv::Mat> robot_contours;
    cv::warpPerspective(camera_image, registered_img, homography, cartesian_size); 
    
    #ifdef DEBUG 
    cout << "Displaying registered image... " << endl;
    cv::namedWindow( "Registered image", WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Registered image", registered_img );                   // Show our image inside it.
    waitKey(0);  
    #endif 

    // Threshold image to find bright points
    cv::inRange(registered_img, lower_bound, upper_bound, mask);

    // Save image without pose in case localisation fails
    pose_image = registered_img.clone();

    // Extract features
    cv::findContours(mask, robot_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    for( int i = 0; i< robot_contours.size(); i++ ) {
       cv::Scalar color = 100;
       drawContours( pose_image, robot_contours,  i, color);
    }

    #ifdef DEBUG 
    cv::namedWindow( "LED mask", WINDOW_AUTOSIZE );
    cv::imshow( "LED mask", mask );     
    waitKey(0);  
    #endif 

    // Find the LEDs
    std::vector<Led> leds;
    filter_contours(robot_contours, leds, MINIMUM_BLOB_AREA);
    
    #ifdef DEBUG
    std::size_t NUM_LEDS = leds.size();
    cout << "Number of infrared LEDs found in image: " << NUM_LEDS << endl;
    #endif 

    // Find bounding box around LEDs        
    cv::Point min, max; 
    find_led_bounds(leds, min, max);

    // Find the center LED
    std::size_t center_led_ix = find_center_led(leds, min, max);
    
    if (center_led_ix == INDEX_NOT_FOUND){

        #ifdef DEBUG
        cout << "Did not find robot!" << endl;
        #endif

        result->x=0;
        result->y=0;
        result->theta=0;
        return 1;

    } else {

        #ifdef DEBUG
        cout << "Found robot!" << endl;
        cout << "Center led:" << leds[center_led_ix] << endl;
        #endif

        // Compute distance from each LED to center LED
        get_distances_to_center_led(leds, center_led_ix);

        // Sort LEDs on distance to center
        std::sort(leds.begin(), leds.end());

        #ifdef DEBUG
        cout << "LEDs sorted on distance to center:" << endl;
        for (auto led: leds){
            cout << led << endl;
        }
        cout << "Found two leds closest to center LED:" << std::endl;
        cout << leds[CLOSEST_TO_CENTER] << endl;
        cout << leds[SECOND_CLOSEST_TO_CENTER] << endl;
        #endif

        // Find midpoint of the two LEDs closest to the center point
        pixel_coord mid_point_x = (leds[CLOSEST_TO_CENTER].centroid.x + leds[SECOND_CLOSEST_TO_CENTER].centroid.x)/2;
        pixel_coord mid_point_y = (leds[CLOSEST_TO_CENTER].centroid.y + leds[SECOND_CLOSEST_TO_CENTER].centroid.y)/2;
        
        #ifdef DEBUG
        cout << "Midpoint between two closest leds: " << mid_point_x << "," << mid_point_y << endl;
        #endif 

        // Compute position in metres from location of center LED
        cartesian_coord x = (((eucl_dist)leds[CENTER].centroid.x) / ARENA_WIDTH_PIXELS)*ARENA_WIDTH_M;
        cartesian_coord y = ARENA_HEIGHT_M - (((eucl_dist)leds[CENTER].centroid.y) / ARENA_HEIGHT_PIXELS)*ARENA_HEIGHT_M;

        // Use point between the two closest LEDs and center of middle LED to find angle
        eucl_dist center_led_to_nearest_pixels_x = (eucl_dist)leds[CENTER].centroid.x - (eucl_dist)mid_point_x;
        eucl_dist center_led_to_nearest_pixels_y = (eucl_dist)leds[CENTER].centroid.y - (eucl_dist)mid_point_y;
        radians angle_rad = atan2(center_led_to_nearest_pixels_y,center_led_to_nearest_pixels_x);
        degrees angle_deg = (degrees)round(-angle_rad*DEG_PER_RAD);

        // Draw pose
        Point pt1( leds[CENTER].centroid.x, leds[CENTER].centroid.y);
        Point pt2( (pixel_coord)((eucl_dist) leds[CENTER].centroid.x + ARROW_LENGTH * cos(angle_rad)), (pixel_coord)((eucl_dist)leds[CENTER].centroid.y+ ARROW_LENGTH* sin(angle_rad)));
        cv::arrowedLine(pose_image, pt1, pt2, ARROW_INTENSITY, ARROW_THICKNESS);

        #ifdef DEBUG
        cout << "Angle in radians: " << angle_rad << endl;
        cout << "x,y,theta: " << x << "," << y << "," << angle_deg  << endl;
        cv::namedWindow( "Img with arrow", WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "img with arrow", pose_image );                   // Show our image inside it.
        waitKey(0);    
        #endif

        result->x = x;
        result->y = y;
        result->theta = angle_deg;
    } 
    return 0;
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
    #endif 
    return 0;
}

int Localiser::save_pose_img(void){

    // Collect test images
    //cv::imwrite("test_img.jpg",camera_image);
    cv::imwrite("/var/www/EGB439/camera/get/arena.jpg",pose_image);
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

}