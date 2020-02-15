#include "types.h"

namespace PenguinPi
{

struct Pose2D {

  Pose2D() : x(0), y(0), theta(0) {};
  cartesian_coord x;
  cartesian_coord y;
  degrees theta;
};

}
