#include "penguinpi.h"
#include "localiser.h"

using namespace std;

namespace PenguinPi {

Localiser :: Localiser () : 

    cartesian_size(ARENA_WIDTH_PIXELS,ARENA_HEIGHT_PIXELS) {

    const pixel_coord in[] = {107,5,558,6,580,474,77,473};
    const pixel_coord out[] = {0,0,ARENA_WIDTH_PIXELS,0,ARENA_WIDTH_PIXELS,ARENA_HEIGHT_PIXELS,0,ARENA_HEIGHT_PIXELS};

    for (int i=0; i<8; i+=2){

        cv::Point input_point(in[i],in[i+1]);
        cv::Point output_point(out[i],out[i+1]);

        tiepoint_src_.push_back(input_point);
        tiepoint_dest_.push_back(output_point);
    }
    homography = findHomography(tiepoint_src_, tiepoint_dest_);

}


int Localiser::compute_pose(const cv::Mat * src, Pose2D * result){

    // Apply homography to extract arena image from camera image
    cv::Mat registered_img, mask;
    std::vector<cv::Mat> robot_contours;

    cv::warpPerspective(*src, registered_img, homography, cartesian_size); 
    
    #ifdef DEBUG 
    cout << "Displaying registered image... " << endl;
    cv::namedWindow( "Registered image", WINDOW_AUTOSIZE );
    cv::imshow( "Registered image", registered_img );   
    waitKey(DEBUG_WINDOW_TIMEOUT_MS);  
    #endif 

    // Threshold image to find bright points
    cv::threshold( registered_img, mask, MASK_LOWER_BOUND, MASK_UPPER_BOUND, cv::THRESH_BINARY );

    // Extract features
    cv::findContours(mask, robot_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    #ifdef DEBUG 
    cout << "Found " << robot_contours.size() << " contours in image" << endl;
    #endif 

    for( int i = 0; i< robot_contours.size(); i++ ) {
        cv::Scalar color = 100;
        drawContours( registered_img, robot_contours,  i, color);
    }
    #ifdef DEBUG 
    cv::namedWindow( "LED mask", WINDOW_AUTOSIZE );
    cv::imshow( "LED mask", mask );     
    waitKey(DEBUG_WINDOW_TIMEOUT_MS);  
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
        radians angle_rad = (radians) atan2(center_led_to_nearest_pixels_y,center_led_to_nearest_pixels_x);
        degrees angle_deg = (degrees) round(-angle_rad*DEG_PER_RAD);

        #ifdef DEBUG
        cout << "Angle in radians: " << angle_rad << endl;
        cout << "x,y,theta: " << x << "," << y << "," << angle_deg  << endl;   
        #endif

        result->x = x;
        result->y = y;
        result->theta = angle_deg;
    } 
    return 0;
}


int Localiser::draw_pose(cv::Mat & frame, Pose2D * pose){

        cout << "Pose: " << pose->x << "," << pose->y << "," << pose->theta <<  endl;
        int x_ = ARENA_WIDTH_PIXELS*(pose->x)/ARENA_WIDTH_M;
        int y_ = ARENA_HEIGHT_PIXELS - ARENA_HEIGHT_PIXELS*(pose->y)/ARENA_HEIGHT_M;
        cv::Point pt1(x_,y_);

        cv::Point pt2( (x_+ ARROW_LENGTH * (cos(pose->theta*M_PI/180))), 
                    (y_ + ARROW_LENGTH * (sin(pose->theta*M_PI/180)))
                );
        cv::arrowedLine(frame, pt1, pt2, ARROW_INTENSITY, ARROW_THICKNESS);
        cout << "Location in pixels:" << x_ << "," << y_ << endl;
        return 0;
}

std::ostream & operator<<(std::ostream & os, const Localiser & localiser)
{
    os << std::endl << "Homography source points:" << std::endl;
    for (auto i: localiser.tiepoint_src_)
        os << i << " ";
    os << std::endl;
    os << "Homography destination points:" << std::endl;
    for (auto i: localiser.tiepoint_dest_)
        os << i << " ";
    os << std::endl;
    os << "Homography between points: " << std::endl;
    os << localiser.homography << std::endl << std::endl;
    return os;
}

Localiser :: ~Localiser(){


}


int filter_contours(const std::vector<cv::Mat> contours, std::vector<PenguinPi::Led> &dest, const pixel_area minimum_area){

    for (auto contour: contours){
        
        cv::Rect rectangle = cv::boundingRect(contour);
        pixel_area area = rectangle.width * rectangle.height;

        if (area > minimum_area){
            Led led(rectangle, contour);
            dest.push_back(led);
        }
    }
    return 0;
}

size_t find_center_led(const std::vector<PenguinPi::Led> leds, const cv::Point min, const cv::Point max){
    size_t center_led_index = INDEX_NOT_FOUND;
    for(std::size_t i=0; i<leds.size(); i++){
        if (leds[i].centroid.x < max.x && leds[i].centroid.x > min.x && leds[i].centroid.y < max.y && leds[i].centroid.y > min.y){
            center_led_index = i;
        }
    }
    return center_led_index;
}

int get_distances_to_center_led(std::vector<PenguinPi::Led> &leds, const std::size_t center_led_index){
        for(std::size_t i=0; i<leds.size(); i++){
            if (center_led_index == i) {
                leds[i].dist_to_center_led = 0;
            } else {
                pixels diff_x = abs((int)leds[center_led_index].centroid.x - (int)leds[i].centroid.x);
                pixels diff_y = abs((int)leds[center_led_index].centroid.y - (int)leds[i].centroid.y);
                eucl_dist dist_led_float = sqrt( (eucl_dist)pow(diff_x,2) + (eucl_dist)pow(diff_y,2));
                pixels dist_led = (pixels) round(dist_led_float);
                leds[i].dist_to_center_led = dist_led;
            }
        }
    return 0;
}

int find_led_bounds(std::vector<PenguinPi::Led> const leds, cv::Point &min, cv::Point &max){

    min.x = ARENA_WIDTH_PIXELS;
    max.x = 0;
    min.y = ARENA_HEIGHT_PIXELS;
    max.y = 0;

    for (auto led : leds){
        if (led.centroid.x < min.x){
            min.x = led.centroid.x;
        }
        if (led.centroid.x > max.x){
            max.x = led.centroid.x;
        }
        if (led.centroid.y < min.y){
            min.y = led.centroid.y;
        }
        if (led.centroid.y > max.y){
            max.y = led.centroid.y;
        }
    }
    return 0;
}

}
