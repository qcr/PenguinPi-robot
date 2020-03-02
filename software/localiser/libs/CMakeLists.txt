find_package( OpenCV REQUIRED)

add_library(penguinpi 
    src/localiser.cpp
    src/led.cpp
    src/tcp_srvr.cpp
)

set_target_properties(penguinpi PROPERTIES PUBLIC_HEADER 
    "include/penguinpi.h"
)

include_directories(penguinpi 
    ${PROJECT_SOURCE_DIR}/include 
    include
)


if(DEBUG)
    target_compile_definitions(penguinpi PRIVATE DEBUG=1)
endif(DEBUG)
if(PROFILE)
    target_compile_definitions(penguinpi PRIVATE PROFILE=1)
endif(PROFILE)
if(HEADLESS)
    target_compile_definitions(penguinpi PRIVATE HEADLESS=1)
endif(HEADLESS)

target_link_libraries(penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

install(
        TARGETS penguinpi
        RUNTIME DESTINATION bin
	    PUBLIC_HEADER DESTINATION include
	    ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
    )


# -------------------------- BUILD TESTS (won't be installed) ------------------------------ #


add_executable(01_test_stream 
    tests/01_stream.cpp
)
target_link_libraries(01_test_stream
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(02_test_realtime
    tests/02_realtime.cpp
)

target_link_libraries(02_test_realtime
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(03_test_localisationspeed
    tests/03_localisationspeed.cpp
)

target_link_libraries(03_test_localisationspeed
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(04_test_frame_decoding 
    tests/04_framedecoding.cpp
)
target_link_libraries(04_test_frame_decoding
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(05_simpleudpserver
    tests/05_simpleudpserver.cpp 
)
add_executable(06_simpletcpserver
    tests/06_simpletcpserver.cpp 
)
add_executable(07_tcpclass
    tests/07_tcpclass.cpp 
)
target_link_libraries(07_tcpclass
    penguinpi
)

add_executable(08_tcpclient
    tests/08_tcpclient.cpp 
)

add_executable(09_cvopencamera
    tests/09_cvopencamera.cpp 
)
target_link_libraries(09_cvopencamera
    penguinpi
)

add_executable(10_servestreamtcp
    tests/10_servestreamtcp.cpp)
target_link_libraries(10_servestreamtcp
    penguinpi
)

add_executable(11_localiser_and_stream_thread
    tests/11_localiser_and_stream_thread.cpp)
target_link_libraries(11_localiser_and_stream_thread
    penguinpi
)

add_executable(12_parse_request
    tests/12_parse_request.cpp)
target_link_libraries(12_parse_request
    penguinpi
)