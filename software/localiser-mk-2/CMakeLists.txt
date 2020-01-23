cmake_minimum_required(VERSION 3.5)
project(Localiser)
find_package( OpenCV REQUIRED )

MESSAGE(STATUS "Include dirs ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "LINK LIBRARIES ${OpenCV_LIBS}")
include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(localiser src/main.cpp src/vision.cpp)
target_link_libraries(localiser ${OpenCV_LIBS})

add_executable(cgi_app src/cgi.cpp)
target_link_libraries(cgi_app fcgi++ fcgi)
