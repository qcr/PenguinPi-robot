
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include "box.h"

using namespace cv;

class Contours {
    public:
        Contours ();
        void get_contours(std::vector<Mat> contours); 
        std::vector<Box> get_boxes();
    private: 
        std::vector<Mat> list_contours_;
        std::vector<Box> list_boxes_; 
        int minimum_area_;
};


