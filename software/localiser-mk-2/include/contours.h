
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include "led.h"

using namespace cv;

class Contours {
    public:
        Contours ();
        void get_contours(std::vector<Mat> contours); 
        std::vector<Led> get_leds();
    private: 
        std::vector<Mat> list_contours_;
        std::vector<Led> list_leds_; 
        int minimum_area_;
};


