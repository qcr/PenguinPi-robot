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

if (PROFILE)
    add_definitions(-DPROFILE=1)
endif()

add_library(localiser 
    src/localiser.cpp
    src/vision_utils.cpp
    src/sock_srvr.cpp
    src/led.cpp
)

set_target_properties(localiser PROPERTIES PUBLIC_HEADER include/localiser.h)

target_link_libraries(localiser
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(locclient 
    src/main.cpp 
)

target_link_libraries(locclient
    pthread
    localiser
    rt
    ${OpenCV_LIBS}
)

if(CAMERA)
    target_compile_definitions(localiser PRIVATE CAMERA=1)
endif(CAMERA)

if(DEBUG)
    target_compile_definitions(localiser PRIVATE DEBUG=1)
endif(DEBUG)

include_directories(localiser 
    ${PROJECT_SOURCE_DIR}/include 
)

add_executable(teststream 
    testing/teststream.cpp
)
target_link_libraries(teststream
    localiser
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(testrealtime
    testing/testrealtime.cpp
)

target_link_libraries(testrealtime
    pthread
    rt
    ${OpenCV_LIBS}
)

install(TARGETS locclient localiser
        RUNTIME DESTINATION bin
	PUBLIC_HEADER DESTINATION include
	ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

unset(CAMERA CACHE)
unset(DEBUG CACHE)
unset(PROFILE CACHE)
