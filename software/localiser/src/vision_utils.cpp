#include "vision_utils.h"


namespace PenguinPi {

int filter_contours(const std::vector<cv::Mat> contours, std::vector<PenguinPi::Led> &dest, const pixel_area minimum_area){

    for (auto contour: contours){
        
        cv::Rect rectangle = cv::boundingRect(contour);
        pixel_area area = rectangle.width * rectangle.height;

        if (area > minimum_area){
            Led led(rectangle, contour);
            dest.push_back(led);
        }
    }
    return 0;
}

size_t find_center_led(const std::vector<PenguinPi::Led> leds, const cv::Point min, const cv::Point max){
    size_t center_led_index = INDEX_NOT_FOUND;
    for(std::size_t i=0; i<leds.size(); i++){
        if (leds[i].centroid.x < max.x && leds[i].centroid.x > min.x && leds[i].centroid.y < max.y && leds[i].centroid.y > min.y){
            center_led_index = i;
        }
    }
    return center_led_index;
}

int get_distances_to_center_led(std::vector<PenguinPi::Led> &leds, const std::size_t center_led_index){
        for(std::size_t i=0; i<leds.size(); i++){
            if (center_led_index == i) {
                leds[i].dist_to_center_led = 0;
            } else {
                pixels diff_x = abs((int)leds[center_led_index].centroid.x - (int)leds[i].centroid.x);
                pixels diff_y = abs((int)leds[center_led_index].centroid.y - (int)leds[i].centroid.y);
                eucl_dist dist_led_float = sqrt( (eucl_dist)pow(diff_x,2) + (eucl_dist)pow(diff_y,2));
                pixels dist_led = (pixels) round(dist_led_float);
                leds[i].dist_to_center_led = dist_led;
            }
        }
    return 0;
}

int find_led_bounds(std::vector<PenguinPi::Led> const leds, cv::Point &min, cv::Point &max){

    min.x = ARENA_WIDTH_PIXELS;
    max.x = 0;
    min.y = ARENA_HEIGHT_PIXELS;
    max.y = 0;

    for (auto led : leds){
        if (led.centroid.x < min.x){
            min.x = led.centroid.x;
        }
        if (led.centroid.x > max.x){
            max.x = led.centroid.x;
        }
        if (led.centroid.y < min.y){
            min.y = led.centroid.y;
        }
        if (led.centroid.y > max.y){
            max.y = led.centroid.y;
        }
    }
    return 0;
}

} // end ns

