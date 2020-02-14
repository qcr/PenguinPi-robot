#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <mutex>          // std::mutex



struct Pose2D {

  Pose2D() : x(0), y(0), theta(0) {};
  float x;
  float y;
  float theta;
} ;


struct SharedPose
{
   //Mutex to protect access to the queue
  std::mutex mutex;

  Pose2D pose;
};

