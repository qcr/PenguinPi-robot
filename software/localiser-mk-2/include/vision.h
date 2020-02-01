#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

#include <stdint.h>
#include <iostream>
#include <vector>

#include "pose.h"

using namespace cv; 

class Localiser {
    private:
      std::vector<Point2f> srcPoints;
      std::vector<Point2f> dstPoints;
      Mat homography;
      Size size; 
      int upper_bound;
      int lower_bound;
      int flipCode;


    public:
        Localiser ();
        int compute_pose(Mat img, Pose2D * result);
        friend std::ostream & operator<<(std::ostream& os, const Localiser& localiser);
};