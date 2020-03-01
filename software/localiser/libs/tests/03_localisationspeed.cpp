#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>
#include <penguinpi.h>

#define STREAM_PORT     ("udp://0.0.0.0:5000")
#define MAX_FRAMES      (1000)
#define FRAME_WAIT      (5)
 
using namespace std;
using namespace cv;
using namespace std::chrono;

// To test, run raspivid on a raspberry pi 
// eg 
// $ raspivid -t 0 -w 640 -fps 90 -h 480 -o - | nc -p 1904 -u LOCALHOST 5000

int main(){

  PenguinPi::Localiser localiser;

  cout << "Waiting for stream at " << STREAM_PORT << endl;
  VideoCapture stream(STREAM_PORT); 
    
  // Check if camera opened successfully
  if(!stream.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  int frames_decoded = 0;

  //while(frames_decoded < MAX_FRAMES){
 while(1){

    PenguinPi::Pose2D pose;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    Mat frame_bgr, frame_gray;
    stream >> frame_bgr;

    if (frame_bgr.empty()) { 
        break;
    }
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    frames_decoded++;

    cv::cvtColor(frame_bgr, frame_gray, CV_BGR2GRAY);
    localiser.compute_pose(&frame_gray, &pose);

    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    localiser.draw_pose(frame_gray, &pose);

    duration<double> capture_time = duration_cast<duration<double>>(t1 - t0);
    duration<double> compute_time = duration_cast<duration<double>>(t2 - t1);
    double capture_time_ms = capture_time.count()*1e3;
    double compute_time_ms = compute_time.count()*1e3;
    cout << "Frame grab time: " << capture_time_ms << " ms" << endl;
    cout << "Image processing time: " << compute_time_ms << " ms" << endl << endl;

    imshow( "Frame", frame_gray );
    waitKey(FRAME_WAIT);
  }

  stream.release();
 
  // Closes all the frames
  destroyAllWindows();
     
  return 0;
}