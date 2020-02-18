#include "localiser.h"

#ifdef PROFILE
#include <chrono>
#endif


using namespace std;

namespace PenguinPi {

Localiser :: Localiser () : 

    #ifdef CAMERA
    camera(),
    #endif 
    cartesian_size(ARENA_WIDTH_PIXELS,ARENA_HEIGHT_PIXELS) {

    #ifdef CAMERA
    cout << "Opening camera.. " << endl; 

    wait_for_stream();

    #else 
    camera_image = cv::imread("test_img.jpg", IMREAD_GRAYSCALE);

    #ifdef DEBUG 
    cv::namedWindow( "Display window", WINDOW_AUTOSIZE );
    cv::imshow( "Display window", camera_image );             
    cv::waitKey(0);                 
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

        tiepoint_src.push_back(input_point);
        tiepoint_dest.push_back(output_point);
    }
    homography = findHomography(tiepoint_src, tiepoint_dest);

    socksrvconf config;
    std::strcpy(config.sun_path, SOCKFILE);
    config.buflen = LOC_MSG_LEN;
    sock.configure(&config);

    #ifdef CAMERA
    update_camera_img();
    save_camera_img();
    #endif
}

int Localiser::init_networking(void){
    if(sock.connect()){
        cerr << "Socket failed to connect" << endl;
        return -1;
    }
    return 0;
}

int Localiser::wait_for_stream(void){


    #ifdef CAMERA
    bool camera_open = camera.isOpened();

    while (!camera_open){
        camera.open(VIDEO_STREAM);
        if (camera.isOpened()) {
            camera_open = true;
            cout << "Connected to stream at " << VIDEO_STREAM << endl;
        }
        else { 
            cerr << "Error opening camera, retrying... " << endl; 
//            std::this_thread::sleep_for (std::chrono::seconds(1));
        }
    }
    #endif
    return 0;
}

int Localiser::update_camera_img(void){

    #ifdef CAMERA
    camera.read(video_frame);
    cvtColor(video_frame, camera_image, COLOR_BGR2GRAY);
    #else 
    
    #endif 
    return 0;
}

int Localiser::listen(void){
    
    if (sock.wait_for_request()){
        cerr << "Error getting request from socket" << endl;
        return -1;
    }

    // Get the request type from the buffer
    char request[LOC_REQ_TYPE_LEN+1];
    memcpy(request, (sock.buf + LOC_REQ_TYPE_OFFSET), LOC_REQ_TYPE_LEN);
    request[LOC_REQ_TYPE_LEN] = '\0';
    int request_type = atoi(request);

    #ifdef DEBUG
    cout << "Request code: " << request_type << endl;
    #endif
    return request_type;
}

int Localiser::send_pose(void){

    #ifdef PROFILE
    auto t1 = std::chrono::high_resolution_clock::now();
    #endif 

    char response[LOC_MSG_LEN];
    memset(response,0,LOC_MSG_LEN);
    update_camera_img();
    
    #ifdef PROFILE
    auto t2 = std::chrono::high_resolution_clock::now();
    #endif

    compute_pose(&latest_pose);

    #ifdef PROFILE
    auto t3 = std::chrono::high_resolution_clock::now();
    auto camera_duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    auto imgproc_duration = std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
    cout << "Time taken for camera: " << camera_duration << " us, img proc: " << imgproc_duration << endl;
    #endif

    sprintf(response,"{\"pose\":{\"x\":%f,\"y\":%f,\"theta\":%f}}",latest_pose.x,latest_pose.y,latest_pose.theta);
    sock.pack_response(response);
    sock.send_response();
    return 0;
}

int Localiser::send_tie_points(void){

    char response[LOC_MSG_LEN];
    memset(response,0,LOC_MSG_LEN);

    sprintf(response,"{\"NW\":{\"x\":%d, \"y\":%d}, \"NE\":{\"x\":%d, \"y\":%d}, \"SE\":{\"x\":%d,\"y\":%d}, \"SW\":{\"x\":%d,\"y\":%d}}",
    tiepoint_src[TIEPOINT_NW].x,
    tiepoint_src[TIEPOINT_NW].y,
    tiepoint_src[TIEPOINT_NE].x,
    tiepoint_src[TIEPOINT_NE].y,
    tiepoint_src[TIEPOINT_SE].x,
    tiepoint_src[TIEPOINT_SE].y,
    tiepoint_src[TIEPOINT_SW].x,
    tiepoint_src[TIEPOINT_SW].y
    );

    sock.pack_response(response);
    sock.send_response();
    return 0;
}

int Localiser::update_tie_point(void){

    char tiepoint_str[LOC_REQ_TYPE_LEN+1];
    char x_coord_str[TIE_POINT_WIDTH+1];
    char y_coord_str[TIE_POINT_WIDTH+1];

    memcpy(tiepoint_str, (sock.buf + LOC_REQ_CORNER_OFFSET), LOC_REQ_CORNER_LEN);
    tiepoint_str[LOC_REQ_TYPE_LEN] = '\0';

    memcpy(x_coord_str, (sock.buf + LOC_REQ_CORNER_OFFSET + LOC_REQ_CORNER_LEN), TIE_POINT_WIDTH);
    x_coord_str[TIE_POINT_WIDTH] = '\0';

    memcpy(y_coord_str, (sock.buf + LOC_REQ_CORNER_OFFSET + LOC_REQ_CORNER_LEN + LOC_REQ_COORD_LEN), TIE_POINT_WIDTH);
    x_coord_str[TIE_POINT_WIDTH] = '\0';

    tiepoint_id tiepoint = atoi(tiepoint_str);   
    pixel_coord x = atoi(x_coord_str);
    pixel_coord y= atoi(y_coord_str);

    cout << "Decoded tie point request to " << tiepoint << "," << x << "," << y << endl;

    tiepoint_src[tiepoint].x = x;
    tiepoint_src[tiepoint].y = y;

    homography = findHomography(tiepoint_src, tiepoint_dest);

    char response[LOC_MSG_LEN];
    sprintf(response,"success");
    sock.pack_response(response);
    sock.send_response();

    return 0;
}

int Localiser::save_camera_img(void){
    char response[LOC_MSG_LEN];
    memset(response,0,LOC_MSG_LEN);
    update_camera_img();
    cv::imwrite("camera/camera_raw.jpg",camera_image);
    sprintf(response,"success");
    sock.pack_response(response);
    sock.send_response();
    return 0;
}

int Localiser::save_pose_img(void){
    char response[LOC_MSG_LEN];
    memset(response,0,LOC_MSG_LEN);
    PenguinPi::Pose2D latest_pose;
    update_camera_img();
    compute_pose(&latest_pose);
    cv::imwrite("camera/arena.jpg",pose_image);
    sprintf(response,"success");
    sock.pack_response(response);
    sock.send_response();
    return 0;
}

int Localiser::compute_pose(Pose2D * result){

    // Apply homography to extract arena image from camera image
    cv::Mat registered_img, mask;
    std::vector<cv::Mat> robot_contours;
    cv::warpPerspective(camera_image, registered_img, homography, cartesian_size); 
    
    #ifdef DEBUG 
    cout << "Displaying registered image... " << endl;
    cv::namedWindow( "Registered image", WINDOW_AUTOSIZE );
    cv::imshow( "Registered image", registered_img );   
    waitKey(0);  
    #endif 

    // Threshold image to find bright points
    //cv::inRange(registered_img, MASK_LOWER_BOUND, MASK_UPPER_BOUND, mask);

    // Save image without pose in case localisation fails
    pose_image = registered_img.clone();

    // Extract features
    cv::findContours(registered_img, robot_contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    
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
        radians angle_rad = (radians) atan2(center_led_to_nearest_pixels_y,center_led_to_nearest_pixels_x);
        degrees angle_deg = (degrees) round(-angle_rad*DEG_PER_RAD);

        // Draw pose
        Point pt1( leds[CENTER].centroid.x, leds[CENTER].centroid.y);
        Point pt2( (pixel_coord)((eucl_dist) leds[CENTER].centroid.x + ARROW_LENGTH * cos(angle_rad)), 
                    (pixel_coord)((eucl_dist)leds[CENTER].centroid.y+ ARROW_LENGTH* sin(angle_rad))
                );
        cv::arrowedLine(pose_image, pt1, pt2, ARROW_INTENSITY, ARROW_THICKNESS);

        #ifdef DEBUG
        cout << "Angle in radians: " << angle_rad << endl;
        cout << "x,y,theta: " << x << "," << y << "," << angle_deg  << endl;
        cv::namedWindow( "Img with arrow", WINDOW_AUTOSIZE );
        cv::imshow( "img with arrow", pose_image );   
        waitKey(0);    
        #endif

        result->x = x;
        result->y = y;
        result->theta = angle_deg;
    } 
    return 0;
}

std::ostream & operator<<(std::ostream & os, const Localiser & localiser)
{
    os << std::endl << "Homography source points:" << std::endl;
    for (auto i: localiser.tiepoint_src)
        os << i << " ";
    os << std::endl;
    os << "Homography destination points:" << std::endl;
    for (auto i: localiser.tiepoint_dest)
        os << i << " ";
    os << std::endl;
    os << "Homography between points: " << std::endl;
    os << localiser.homography << std::endl << std::endl;
    os << "Socket:" << localiser.sock << endl;
    return os;
}

Localiser :: ~Localiser(){

    #ifdef CAMERA
    camera.release();
    #endif 
}

}
