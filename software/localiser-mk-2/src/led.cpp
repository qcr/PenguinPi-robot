#include "led.h"

std::ostream & operator<<(std::ostream & os, const Led & led)
{
    os << "Led at " << led.x << "," << led.y << std::endl;
    os << "Width, height in pixels: " << led.w << "," << led.h << std::endl;
    os << "Centre coordinates: " << led.cx << "," << led.cy << std::endl;
    return os;
}

