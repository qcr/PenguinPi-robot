
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
     
#include <math.h> 
#include <unistd.h>


namespace PenguinPi {

typedef uint pixels;
typedef uint pixel_area;
typedef uint pixel_coord;
typedef uint tiepoint_id;
typedef float eucl_dist;
typedef float radians;
typedef float degrees;
typedef float cartesian_coord;

struct Pose2D {

  Pose2D() : x(0), y(0), theta(0) {};
  cartesian_coord x;
  cartesian_coord y;
  degrees theta;
};

class Localiser {

    private:
      cv::Mat video_frame_;
      std::vector<cv::Point> tiepoint_src_;
      std::vector<cv::Point> tiepoint_dest_;
      cv::Mat homography;
      cv::Size cartesian_size; 

    public:
        Localiser ();
        int compute_pose(const cv::Mat * src, Pose2D * result);
        int draw_pose(cv::Mat & frame, Pose2D * result);
        friend std::ostream & operator<<(std::ostream& os, const Localiser& localiser);
        ~Localiser ();
};

class Led {

  public:
    Led (cv::Rect rect, cv::Mat cont);

    cv::Point centroid;
    cv::Mat contour;
    pixels dist_to_center_led;

    friend std::ostream & operator<<(std::ostream& os, const Led& led);

};



bool operator <(const Led & led1, const Led & led2);

int filter_contours(const std::vector<cv::Mat> contours, std::vector<PenguinPi::Led> &dest, const pixel_area minimum_area);

int find_led_bounds(std::vector<PenguinPi::Led> leds, cv::Point &min, cv::Point &max);

size_t find_center_led(const std::vector<PenguinPi::Led> leds, const cv::Point min, const cv::Point max);

int get_distances_to_center_led(std::vector<PenguinPi::Led> &leds, const std::size_t center_led_index);

}