#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <stdint.h>
#include <math.h> 
#include <iostream>
#include <vector>

#ifndef DESKTOP
#include <raspicam/raspicam_cv.h>
#endif 

#include <ctime>

#include "pose.h"

using namespace cv; 

#define INIT_VAL  (10000)   // A large value for initialising a search for an index

class Localiser {
    private:

      #ifndef DESKTOP
      raspicam::RaspiCam_Cv camera;
      #ifndef DESKTOP

      Mat camera_image;
      std::vector<Point2f> srcPoints;
      std::vector<Point2f> dstPoints;
      Mat homography;
      Size size; 
      int upper_bound;
      int lower_bound;
      int flipCode;
      uint8_t camera_save_timer;
    public:

        #ifndef DESKTOP
        Localiser ();
        #else 
        Localiser (const char * img_file);
        #endif
        int update_camera_img(void);
        int compute_pose(Pose2D * result);
        friend std::ostream & operator<<(std::ostream& os, const Localiser& localiser);
        ~Localiser ();
};
