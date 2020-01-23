#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <mutex>          // std::mutex

struct Pose2D {

  Pose2D() : x(0), y(0), theta(0) {};
  double x;
  double y;
  double theta;
} ;


struct SharedPose
{
  //  enum { NumItems = 100 };
  //  enum { LineSize = 100 };

  //  SharedPose()
  //     :  current_line(0)
  //     ,  end_a(false)
  //     ,  end_b(false)
  //  {}

   //Mutex to protect access to the queue
   std::mutex mutex;

  Pose2D pose;
};

