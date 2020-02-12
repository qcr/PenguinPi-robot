cmake_minimum_required(VERSION 3.5)
project(Localiser)

set(raspicam_DIR "/usr/local/lib/cmake")

# Compile without rpi libs if on desktop

option(DESKTOP "Compile for desktop testing" OFF) #OFF by default
if(DESKTOP)
    add_definitions(-DDESKTOP=1)
else()
    #add some other compilation flags
endif(DESKTOP)

option(DEBUG "Compile for desktop testing" OFF) #OFF by default
if(DEBUG)
    add_definitions(-DDEBUG=1)
else()
    #add some other compilation flags
endif(DEBUG)

find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED)
find_package(raspicam REQUIRED)

add_executable(localiser 
    src/main.cpp 
    src/vision.cpp
    src/contours.cpp
)

include_directories(localiser 
    ${PROJECT_SOURCE_DIR}/include 
    ${Boost_INCLUDE_DIR}
)
target_link_libraries(localiser 
    rt
    pthread
    #${raspicam_LIBS}
    ${raspicam_CV_LIBS}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}   
)

add_executable(cgi_app src/cgi.cpp)
target_link_libraries(cgi_app 
    rt 
    fcgi++ 
    fcgi 
    ${Boost_LIBRARIES}
)

unset(DESKTOP CACHE)
unset(DEBUG CACHE)

