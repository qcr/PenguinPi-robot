#include <boost/interprocess/sync/interprocess_mutex.hpp>

struct Pose2D {
  double x;
  double y;
  double theta;
} ;

struct shared_memory_log
{
  //  enum { NumItems = 100 };
  //  enum { LineSize = 100 };

  //  shared_memory_log()
  //     :  current_line(0)
  //     ,  end_a(false)
  //     ,  end_b(false)
  //  {}

   //Mutex to protect access to the queue
   boost::interprocess::interprocess_mutex mutex;

   //Items to fill
  Pose2D   pose;
};

