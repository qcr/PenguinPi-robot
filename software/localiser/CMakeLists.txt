cmake_minimum_required(VERSION 3.5)
project(Localiser)

set(raspicam_DIR "/usr/local/lib/cmake")
set(PROJECT_VERSION 1.0)

option(CAMERA "Compile for raspberry pi target" TRUE) # on by default
option(DEBUG "Verbose" FALSE) #OFF by default
option(PROFILE "Time profiling" FALSE)

if(DEBUG)
    add_definitions(-DDEBUG=1)
endif(DEBUG)

find_package( OpenCV REQUIRED)

#if (CAMERA)
#    find_package(raspicam REQUIRED)
#endif()

if (PROFILE)
    add_definitions(-DPROFILE=1)
endif()

add_executable(localiser 
    src/main.cpp 
    src/vision_utils.cpp
    src/localiser.cpp
    src/sock_srvr.cpp
    src/led.cpp
)

target_link_libraries(localiser
    pthread
    rt
    ${OpenCV_LIBS}
)

if(CAMERA)
    #target_link_libraries(localiser "${raspicam_CV_LIBS}")
    target_compile_definitions(localiser PRIVATE CAMERA=1)
endif(CAMERA)

if(DEBUG)
    target_compile_definitions(localiser PRIVATE DEBUG=1)
endif(DEBUG)


include_directories(localiser 
    ${PROJECT_SOURCE_DIR}/include 
)

unset(CAMERA CACHE)
unset(DEBUG CACHE)
unset(PROFILE CACHE)

install(TARGETS localiser
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)


