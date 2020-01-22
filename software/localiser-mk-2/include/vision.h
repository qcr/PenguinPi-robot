#include <opencv2/core/core.hpp>
#include "pose.h"

using namespace cv; 

class Box {
  public:
    double x, y, w, h, cx, cy; 
    Box (double x, double y, double w, double h) : x(x), y(y), w(w), h(h), cx(x + w/2.0), cy (y + h/2.0) { }
};

// TODO contour class

class Localiser {
    public:
        Localiser ();
        int compute_pose(Mat img, Pose2D * result);
};