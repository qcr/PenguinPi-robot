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

target_compile_definitions(penguinpi PRIVATE)

if(DEBUG)
    target_compile_definitions(penguinpi PRIVATE DEBUG=1)
endif(DEBUG)

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


add_executable(test_stream 
    tests/stream.cpp
)
target_link_libraries(test_stream
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(test_realtime
    tests/realtime.cpp
)

target_link_libraries(test_realtime
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(test_localisationspeed
    tests/localisationspeed.cpp
)

target_link_libraries(test_localisationspeed
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(test_frame_decoding 
    tests/framedecoding.cpp
)
target_link_libraries(test_frame_decoding
    penguinpi
    pthread
    rt
    ${OpenCV_LIBS}
)

add_executable(simpleudpserver
    tests/simpleudpserver.cpp 
)
add_executable(simpletcpserver
    tests/simpletcpserver.cpp 
)
add_executable(tcpclass
    tests/tcpclass.cpp 
)
target_link_libraries(tcpclass
    penguinpi
)

add_executable(servestreamtcp
    tests/servestreamtcp.cpp 
)
target_link_libraries(servestreamtcp
    penguinpi
)