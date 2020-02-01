# include "contours.h"

Contours :: Contours () : minimum_area_(5)  {}

void Contours :: get_contours(std::vector<Mat> contours){

    for (auto contour: contours){
        
        Rect rectangle = boundingRect(contour);
        int area = rectangle.width * rectangle.height;

        if (area > minimum_area_){
            list_contours_.push_back(contour);
            Box box(rectangle.x, rectangle.y, rectangle.width, rectangle.height);
            list_boxes_.push_back(box);
        }
    }
    return;
}

std::vector<Box> Contours :: get_boxes(){
    return list_boxes_;
}