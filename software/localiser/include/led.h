
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <stdio.h>
#include <iostream>

#include "types.h"

namespace PenguinPi{

class Led {

  public:
    Led (cv::Rect rect, cv::Mat cont);

    cv::Point centroid;
    cv::Mat contour;
    pixels dist_to_center_led;

    friend std::ostream & operator<<(std::ostream& os, const Led& led);

};

bool operator <(const Led & led1, const Led & led2);

}