cmake_minimum_required(VERSION 3.5)
project(Localiser)

set(raspicam_DIR "/usr/local/lib/cmake")
set(PROJECT_VERSION 1.0)

option(DEBUG "Verbose" FALSE) #OFF by default
option(PROFILE "Time profiling" FALSE)

if(DEBUG)
    add_definitions(-DDEBUG=1)
endif(DEBUG)

add_subdirectory(libs)
add_subdirectory(app)


find_package( OpenCV REQUIRED)

if (PROFILE)
    add_definitions(-DPROFILE=1)
endif()



unset(DEBUG CACHE)
unset(PROFILE CACHE)
