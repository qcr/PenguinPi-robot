

class Box {
  public:
    double x, y, w, h, cx, cy; 
    Box (double x, double y, double w, double h) : x(x), y(y), w(w), h(h), cx(x + w/2.0), cy (y + h/2.0) { }
};