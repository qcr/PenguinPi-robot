#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <stdint.h>
#include "pose.h"

using namespace cv; 

class Box {
  public:
    double x, y, w, h, cx, cy; 
    Box (double x, double y, double w, double h) : x(x), y(y), w(w), h(h), cx(x + w/2.0), cy (y + h/2.0) { }
};

// TODO contour class

class Localiser {
    private:
      const uint16_t srcPoints[4][2] {{558, 6},{107, 5},{77, 473}, {580, 474}};
      const uint16_t dstPoints[4][2] {{0,0},{500,0},{500,500}, {0,500}};
    public:
        Localiser ();
        int compute_pose(Mat img, Pose2D * result);
};