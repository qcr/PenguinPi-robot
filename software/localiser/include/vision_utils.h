#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h> 

#include "types.h"
#include "led.h"

#define PICAM_IMG_WIDTH             (640)
#define PICAM_IMG_HEIGHT            (480)
#define ARENA_WIDTH_PIXELS          (500)
#define ARENA_HEIGHT_PIXELS         (500)
#define ARENA_WIDTH_M               (2)
#define ARENA_HEIGHT_M              (2)
#define DEG_PER_RAD                 (180.0/M_PI)

#define CENTER                      (0)
#define CLOSEST_TO_CENTER           (1)
#define SECOND_CLOSEST_TO_CENTER    (2)
#define INDEX_NOT_FOUND             (100)

namespace PenguinPi {

int filter_contours(const std::vector<cv::Mat> contours, std::vector<PenguinPi::Led> &dest, const pixel_area minimum_area);
int find_led_bounds(std::vector<PenguinPi::Led> leds, cv::Point &min, cv::Point &max);
size_t find_center_led(const std::vector<PenguinPi::Led> leds, const cv::Point min, const cv::Point max);
int get_distances_to_center_led(std::vector<PenguinPi::Led> &leds, const std::size_t center_led_index);

}