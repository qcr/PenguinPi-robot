
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
#include "sock_srvr.h"
#include "vision_utils.h"

using namespace cv; 

namespace PenguinPi {

#define MASK_LOWER_BOUND            (220)
#define MASK_UPPER_BOUND            (255)
#define MINIMUM_BLOB_AREA           (5)
#define ARROW_LENGTH                (35)
#define ARROW_THICKNESS             (2)
#define ARROW_INTENSITY             (200)

#define LOC_MSG_LEN                 (256)

#define LOC_REQ_TYPE_OFFSET         (0)
#define LOC_REQ_TYPE_LEN            (1)
#define LOC_REQ_CORNER_OFFSET       (1)
#define LOC_REQ_CORNER_LEN          (1)
#define LOC_REQ_COORD_LEN           (4)

#define LOC_REQ_GET_POSE            (0)
#define LOC_REQ_SAVE_POSE_IMG       (1)
#define LOC_REQ_SAVE_CAM_IMG        (2)
#define LOC_REQ_GET_TIEPOINT        (3)
#define LOC_REQ_POST_TIEPOINT       (4)

#define TIEPOINT_NW                 (0)
#define TIEPOINT_NE                 (1)
#define TIEPOINT_SE                 (2)
#define TIEPOINT_SW                 (3)
#define TIE_POINT_WIDTH             (4) // number of characters per tie point in message


class Localiser {
    private:

      #ifdef CAMERA
      raspicam::RaspiCam_Cv camera;
      #endif

      Mat camera_image;
      Mat pose_image;
      SocketServer sock;
      std::vector<Point> tiepoint_src;
      std::vector<Point> tiepoint_dest;
      Mat homography;
      Size cartesian_size; 
    public:

        Localiser ();
        int init_networking(void);
        int listen(void);
        int update_camera_img(void);
        int save_pose_img(void);
        int save_camera_img(void);
        int compute_pose(Pose2D * result);
        int send_pose(void);
        int send_tie_points(void);
        int update_tie_point(void);
        friend std::ostream & operator<<(std::ostream& os, const Localiser& localiser);
        ~Localiser ();
};

}