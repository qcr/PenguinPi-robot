
/* 
 *  Integration test. Video stream from camera, image processing on local host,
 *  TCP server on local host.
 * 
 *  To send requests one at a time: 
 *  echo "hello" | socat -t 30 tcp:127.0.0.1:2115 -
 * 
 * 
 * To automate requests:
 * 
 * */


/**** vary this quantity to enable image processing at different rates ****/
#define IMGPROC_WAIT_MS      (1000)
/**** ----------------------------------------------------------------- ****/


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

#define SLEEP_BETWEEN_FRAME_FETCH_US    (10)

using namespace std::chrono;
using namespace cv;

std::mutex mtx;           // mutex for critical section
cv::Mat frame_bgr;


int decode_frames(cv::VideoCapture * stream){

  cv::Mat frame;

  while(true){

    *(stream) >> frame;

    if (!frame.empty()) { 
      
      mtx.lock();
      frame_bgr = frame.clone();
      mtx.unlock();
    }
    
    usleep(SLEEP_BETWEEN_FRAME_FETCH_US);
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
      std::cout << "Opened video   at " << stream.get(CAP_PROP_FPS) << " fps " << std::endl;
    }

    cv::Mat latest_frame, frame_gray;

    // Grab the initial frame 
    while (frame_bgr.empty()){
        stream >> frame_bgr;
    }

    cv::cvtColor(frame_bgr, frame_gray, CV_BGR2GRAY);

    // Start a thread for fetching video frames. 
    // This is to ensure the localiser is only using the most recent frame.
    std::thread frame_fetcher(decode_frames, &stream);

    // Now set up the tcp server

    PenguinPi::TCPServer server(TCP_PORT, MSGLEN);

    if (server.connect() < 0){
        std::cerr << "Error connecting to server " << std::endl;
    }
    
    PenguinPi::Localiser localiser;


  char response[MSGLEN];

    // Only serve when asked for it
    while(1){

        //server.connect(); 
        char request[256];
        server.getreq(request);

        PenguinPi::Pose2D pose;
        high_resolution_clock::time_point t0 = high_resolution_clock::now();
        
        mtx.lock();
        latest_frame = frame_bgr.clone();
        mtx.unlock();

        cv::cvtColor(latest_frame, frame_gray, CV_BGR2GRAY);

        high_resolution_clock::time_point t1 = high_resolution_clock::now();

        localiser.compute_pose(&frame_gray, &pose);

        high_resolution_clock::time_point t2 = high_resolution_clock::now();

        localiser.draw_pose(frame_gray, &pose);

        duration<double> capture_time = duration_cast<duration<double>>(t1 - t0);
        duration<double> compute_time = duration_cast<duration<double>>(t2 - t1);
        double capture_time_ms = capture_time.count()*1e3;
        double compute_time_ms = compute_time.count()*1e3;
        std::cout << "Time waiting for lock and copying latest frame: " << capture_time_ms << " ms" << std::endl;
        std::cout << "Image processing time: " << compute_time_ms << " ms" << std::endl << std::endl;

        localiser.draw_pose(frame_gray, &pose);
        
        #ifndef HEADLESS 
        cv::imshow( "Frame", frame_gray );
        #endif 

        cv::waitKey(IMGPROC_WAIT_MS);

        //bzero(response,MSGLEN);
        sprintf(response, "{\"pose\":{\"x\":%f,\"y\":%f,\"theta\":%f}",pose.x,pose.y,pose.theta);
        
        //char tmp_response[] = "reply from srvr";
        server.sendmsg(response, sizeof(response));
    }

    // Closes all the frames
    #ifndef HEADLESS
    cv::destroyAllWindows();
    #endif 
        
    return 0;
}