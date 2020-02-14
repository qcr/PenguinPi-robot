

#include <stdio.h>
#include <iostream>

class Led {
  public:
    int x, y, w, h, cx, cy; 
    Led (int x, int y, int w, int h) : x(x), y(y), w(w), h(h), cx(x + w/2.0), cy (y + h/2.0) { }
    friend std::ostream & operator<<(std::ostream& os, const Led& led);
};