#include "led.h"

namespace PenguinPi {

Led :: Led(cv::Rect rect, cv::Mat cont) : centroid((rect.x + rect.width/2.0), (rect.y + rect.height/2.0)), dist_to_center_led() { 
      contour = cont.clone();
}

std::ostream & operator<<(std::ostream & os, const Led & led)
{
    os << "Led at " << led.centroid.x << "," << led.centroid.y << "- " << led.dist_to_center_led << " from center";
    return os;

}
bool operator <(const Led & led1, const Led & led2){
    return led1.dist_to_center_led < led2.dist_to_center_led;
}

}