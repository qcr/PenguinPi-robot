#include "opencv2/opencv.hpp"
#include <iostream>
#include <chrono>

#define MAX_FRAMES      (1000)
#define WAIT_TIME_MS  (5)
 
using namespace std;
using namespace cv;
using namespace std::chrono;
 
int main(){
 
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap("udp://0.0.0.0:5000"); 
    
  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  int frames_decoded = 0;

high_resolution_clock::time_point t1 = high_resolution_clock::now();

  while(frames_decoded < MAX_FRAMES){
 
    Mat frame;

    cap >> frame;

    if (frame.empty()) { 
        break;
    }
      frames_decoded++;
    imshow( "Frame", frame );
    waitKey(WAIT_TIME_MS);
  }

   high_resolution_clock::time_point t2 = high_resolution_clock::now();

   duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

    double exec_time_seconds = time_span.count();
    double fps = MAX_FRAMES/exec_time_seconds;
    double frame_period_ms = exec_time_seconds/MAX_FRAMES*1e3;

  cout << "Decoded " << MAX_FRAMES << " frames in " << exec_time_seconds << " seconds. " << endl;
  cout << "Frame rate: " << fps << " fps, " << frame_period_ms << " milliseconds per frame" << endl;

  cap.release();
 
  // Closes all the frames
  destroyAllWindows();
     
  return 0;
}