
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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

#ifdef CAMERA
#include <raspicam/raspicam_cv.h>
#endif 

#include "pose.h"

using namespace cv; 

#define INIT_VAL    (10000)   // A large value for initialising a search for an index
#define WIDTH       (640)
#define HEIGHT      (480)

class Localiser {
    private:

      #ifdef CAMERA
      raspicam::RaspiCam_Cv camera;
      #endif

      Mat camera_image;
      Mat pose_image;
      std::vector<Point2f> srcPoints;
      std::vector<Point2f> dstPoints;
      Mat homography;
      Size cartesian_size; 
      int upper_bound;
      int lower_bound;
      int flipCode;
    public:

        
        Localiser ();
         
        //Localiser (const char * img_file);
        int update_camera_img(void);
        int save_pose_img(void);
        int compute_pose(Pose2D * result);
        friend std::ostream & operator<<(std::ostream& os, const Localiser& localiser);
        ~Localiser ();
};
