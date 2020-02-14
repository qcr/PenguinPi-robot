cmake_minimum_required(VERSION 3.5)
project(Localiser)

set(raspicam_DIR "/usr/local/lib/cmake")

# Compile without rpi libs if on desktop

option(CAMERA "Compile for raspberry pi target" TRUE) # on by default
option(DEBUG "Verbose" FALSE) #OFF by default

if(DEBUG)
    add_definitions(-DDEBUG=1)
else()
    #add some other compilation flags
endif(DEBUG)

find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED)

if (NO_CAMERA==FALSE)
    find_package(raspicam REQUIRED)
endif()

add_executable(localiser 
    src/main.cpp 
    src/vision.cpp
    src/contours.cpp
)


target_link_libraries(localiser
    ${OpenCV_LIBS}
)

target_link_libraries(localiser
    rt
    pthread
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
)

if(NO_CAMERA)
    target_compile_definitions(localiser PRIVATE NO_CAMERA=1)
else()
    target_link_libraries(localiser "${raspicam_CV_LIBS}")
endif(NO_CAMERA)

if(DEBUG)
    target_compile_definitions(localiser PRIVATE DEBUG=1)
else()
    #add some other compilation flags
endif(DEBUG)

include_directories(localiser 
    ${PROJECT_SOURCE_DIR}/include 
    ${Boost_INCLUDE_DIR}
)


add_executable(cgi_app src/cgi.cpp)
target_link_libraries(cgi_app 
    rt 
    fcgi++ 
    fcgi 
    ${Boost_LIBRARIES}
)

unset(NO_CAMERA CACHE)
unset(DEBUG CACHE)

