
/* 
 * This checks that the video can stream faster than the client, and the client won't skip any frames.
 * 
 * Eg if the camera is streaming over udp at 100fps, but the image processing can only run at 20Hz, 
 * then the frame decoding thread must ensure that the most recent frame is always served.
 * 
 * (This is not the case with the underlying OpenCV VideoCapture object, which stores all frames in a buffer,
 * resulting in stale images if the client cannot decode as fast as the stream.)
 * */



/* After compiling and running, run raspivid on a raspberry pi 
 *
 *   $ raspivid -t 0 -w 640 -fps 90 -h 480 -o - | nc -p 1904 -u <YOUR_LOCAL_IP> 5000
 * 
 * /
 

/**** vary this quantity to enable image processing at different rates ****/
#define IMGPROC_WAIT_MS      (1000)
/**** ----------------------------------------------------------------- ****/




#include "opencv2/opencv.hpp"

#include <iostream>
#include <chrono>
#include <thread>        
#include <mutex>   

#include <penguinpi.h>


#define STREAM_PORT     "udp://0.0.0.0:5000"
#define SLEEP_BETWEEN_FRAME_FETCH_US    (10)

using namespace std::chrono;



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


int main(){

  PenguinPi::Localiser localiser;

  std::cout << "Waiting for stream at " << STREAM_PORT << std::endl;

  cv::VideoCapture stream(STREAM_PORT); 
    
  // Check if camera opened successfully
  if(!stream.isOpened()){
    std::cout << "Error opening video stream or file" << std::endl;
    return -1;
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


 while(1){


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

    cv::imshow( "Frame", frame_gray );
    cv::waitKey(IMGPROC_WAIT_MS);

  }

  // Closes all the frames
  cv::destroyAllWindows();
     
  return 0;
}