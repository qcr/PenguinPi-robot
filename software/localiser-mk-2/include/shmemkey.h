#include <mutex>          // std::mutex
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#define SHARED_MEMORY_KEY "sharedposememory"