cmake_minimum_required(VERSION 3.5)
project(Localiser)

set(PROJECT_VERSION 1.0)

option(DEBUG "Verbose" FALSE) #OFF by default
option(HEADLESS "If true, do not open cv windows" FALSE)
option(PROFILE "Time profiling" FALSE)

find_package( OpenCV REQUIRED)

if(DEBUG)
    add_definitions(-DDEBUG=1)
endif(DEBUG)

if(HEADLESS)
    add_definitions(-DHEADLESS=1)
endif(HEADLESS)

if(PROFILE)
    add_definitions(-DPROFILE=1)
endif()

add_subdirectory(libs)
add_subdirectory(app)

unset(DEBUG CACHE)
unset(HEADLESS CACHE)
unset(PROFILE CACHE)
