
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>


#include <stdint.h>
#include <iostream>
#include <vector>
#ifdef CAMERA
#include <raspicam/raspicam_cv.h>
#endif 

#include "pose.h"
#include "vision_utils.h"

using namespace cv; 

namespace PenguinPi {

#define MASK_LOWER_BOUND            (220)
#define MASK_UPPER_BOUND            (255)
#define MINIMUM_BLOB_AREA           (5)
#define ARROW_LENGTH                (35)
#define ARROW_THICKNESS             (2)
#define ARROW_INTENSITY             (200)

class Localiser {
    private:

      #ifdef CAMERA
      raspicam::RaspiCam_Cv camera;
      #endif

      Mat camera_image;
      Mat pose_image;
      std::vector<Point2f> tiepoint_src;
      std::vector<Point2f> tiepoint_dest;
      Mat homography;
      Size cartesian_size; 
    public:

        Localiser ();
        int update_camera_img(void);
        int save_pose_img(void);
        int compute_pose(Pose2D * result);
        friend std::ostream & operator<<(std::ostream& os, const Localiser& localiser);
        ~Localiser ();
};

}