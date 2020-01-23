cmake_minimum_required(VERSION 3.5)
project(Localiser)
find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED)

#MESSAGE(STATUS "Include dirs ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}")
MESSAGE(STATUS "LINK LIBRARIES ${BOOST_SYSTEM_LIBRARY}")# ${OpenCV_LIBS} ${Boost_LIBRARIES}")


add_executable(localiser 
    src/main.cpp 
    src/vision.cpp
)


include_directories(localiser 
    ${PROJECT_SOURCE_DIR}/include 
    ${Boost_INCLUDE_DIR}
)
target_link_libraries(localiser 
    rt
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
