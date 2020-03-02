
/* 
 *  Integration test. Video stream from camera, image processing on local host,
 *  TCP server on local host.
 * 
 *  To send requests one at a time: 
 *  echo "hello" | socat -t 30 tcp:<HOST>:2115 -
 * 
 * 
 * To automate requests:
 * 
 * */

#include "opencv2/opencv.hpp"

#include <iostream>
#include <chrono>
#include <thread>        
#include <mutex>   

#include <penguinpi.h>

#define MSGLEN 256

// TODO ARGIFY 
#define STREAM_PORT     "/dev/video0"
#define TCP_PORT         2115

#define IMGPROC_WAIT_MS                 (1000)  // how long the CV window will remain open
#define SLEEP_BETWEEN_FRAME_FETCH_US    (10)    // wait in between decoding video frames
#define SLEEP_BETWEEN_LOCALISATION_US   (100)   // wait in between processing image frames

/* 
 * The protocol for communicating with the web server.
 * (Or any other client) via TCP.
 * 
 */
#define REQUEST_TYPE_OFFSET             (0)
#define REQUEST_TYPE_LEN                (2)

#define LOCALISER_SEND_POSE             (0)
#define LOCALISER_SAVE_IMG              (1)

using namespace std::chrono;
using namespace cv;

std::mutex frame_mutex;           // mutex for critical section
cv::Mat frame_bgr;                // the raw frame from the video stream

std::mutex pose_mutex;
PenguinPi::Pose2D pose;

cv::Mat latest_frame, frame_gray;

int decode_frames(cv::VideoCapture * stream){

  cv::Mat frame;

  while(true){

    *(stream) >> frame;

    if (!frame.empty()) { 
      
      frame_mutex.lock();
      frame_bgr = frame.clone();
      frame_mutex.unlock();
    }
    
    usleep(SLEEP_BETWEEN_FRAME_FETCH_US);
  }
}

int image_processing(PenguinPi::Localiser * localiser){

  while(true){

    #ifdef PROFILE
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    #endif 

    frame_mutex.lock();
    latest_frame = frame_bgr.clone();
    frame_mutex.unlock();

    cv::cvtColor(latest_frame, frame_gray, CV_BGR2GRAY);

    #ifdef PROFILE
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    #endif 

    pose_mutex.lock();
    localiser->compute_pose(&frame_gray, &pose);
    pose_mutex.unlock();

    #ifdef PROFILE
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    #endif

    localiser->draw_pose(frame_gray, &pose);

    #ifdef PROFILE
    duration<double> capture_time = duration_cast<duration<double>>(t1 - t0);
    duration<double> compute_time = duration_cast<duration<double>>(t2 - t1);
    double capture_time_ms = capture_time.count()*1e3;
    double compute_time_ms = compute_time.count()*1e3;
    std::cout << "Time waiting for lock and copying latest frame: " << capture_time_ms << " ms" << std::endl;
    std::cout << "Image processing time: " << compute_time_ms << " ms" << std::endl << std::endl;
    #endif 

    localiser->draw_pose(frame_gray, &pose);
    
    #ifndef HEADLESS 
    cv::imshow( "Frame", frame_gray );
    cv::waitKey(IMGPROC_WAIT_MS);
    #endif 

    usleep(SLEEP_BETWEEN_LOCALISATION_US);

  }
}


int main(int argc, char ** argv){

    std::cout << "Waiting for stream at " << STREAM_PORT << std::endl;

    cv::VideoCapture stream(STREAM_PORT); 
        
    // Check if camera opened successfully
    if(!stream.isOpened()){
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    } else {
      std::cout << "Opened video with backend " << stream.getBackendName() << " at " << stream.get(CAP_PROP_FPS) << " fps " << std::endl;
    }

    // Grab the initial frame 
    while (frame_bgr.empty()){
        stream >> frame_bgr;
    }

    cv::cvtColor(frame_bgr, frame_gray, CV_BGR2GRAY);

    // Start a thread for fetching video frames. 
    // This is to ensure the localiser is only using the most recent frame.
    std::thread frame_fetcher(decode_frames, &stream);

    // Set up the tcp server
    PenguinPi::TCPServer server(TCP_PORT, MSGLEN);

    if (server.connect() < 0){
        std::cerr << "Error connecting to server " << std::endl;
    }
    
    // Start a thread for image processing 
    PenguinPi::Localiser localiser;
    std::thread image_processor(image_processing, &localiser);

    char response[MSGLEN];

    while(1){

        server.connect(); 
        char request[MSGLEN];
        bzero(request, MSGLEN);
        server.getreq(request);

        // Parse incoming requests 
        char request_type_s[REQUEST_TYPE_LEN];
        memcpy(request_type_s, (request + REQUEST_TYPE_OFFSET), REQUEST_TYPE_LEN);
        int request_type = atoi(request_type_s);

        #ifdef DEBUG
        std::cout << "Localise received request type: " << request_type << std::endl;
        #endif 
            
        bzero(response,MSGLEN);

        if(request_type == LOCALISER_SEND_POSE){

          pose_mutex.lock();
          sprintf(response, "{\"pose\":{\"x\":%f,\"y\":%f,\"theta\":%f}",pose.x,pose.y,pose.theta);
          pose_mutex.unlock();

        } else {
          std::cerr << "Localiser failed to parse request: " << request_type << std::endl;
        }
 
        server.sendmsg(response, sizeof(response));
    }

    // Closes all the frames
    #ifndef HEADLESS
    cv::destroyAllWindows();
    #endif 
        
    return 0;
}