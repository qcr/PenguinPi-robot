#include "opencv2/opencv.hpp"
#include <iostream>

#include <penguinpi.h>


/*
 * Open a webcam directly using cv videocapture and display frames.
 * 
 */

#define WAIT_TIME_MS  (5)

using namespace std;
using namespace cv;
 
int main(){
 
  // Create a VideoCapture object and open the input file
  // If the input is the web camera, pass 0 instead of the video file name
  VideoCapture cap("/dev/video0"); 

  // Check if camera opened successfully
  if(!cap.isOpened()){
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  std::cout << "Opened video with backend " << cap.getBackendName() << " at " << cap.get(CAP_PROP_FPS) << " fps " << std::endl;
     
  while(1){
 
    Mat frame;
    // Capture frame-by-frame
    cap >> frame;
  
    // If the frame is empty, break immediately
    if (frame.empty())
      break;
 
    // Display the resulting frame
    imshow( "Frame", frame );

    // Press  ESC on keyboard to exit
    waitKey(WAIT_TIME_MS);

  }
  
  // When everything done, release the video capture object
  cap.release();
 
  // Closes all the frames
  destroyAllWindows();
     
  return 0;
}
   