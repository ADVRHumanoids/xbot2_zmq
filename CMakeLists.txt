cmake_minimum_required(VERSION 3.8)
project(xbot2_zmq VERSION 1.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 20)

find_package(xbot2 REQUIRED)
find_package(cppzmq REQUIRED)


##
add_xbot2_plugin(zmq_io src/zmq_io.cpp)

target_link_libraries(zmq_io
    PRIVATE
    cppzmq
    )

target_include_directories(zmq_io
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

##
add_xbot2_device(zmq_hal src/zmq_hal.cpp)

target_link_libraries(zmq_hal
    PRIVATE
    cppzmq
    # zmq_io_protoc
    # protobuf::libprotobuf
    )

target_include_directories(zmq_hal
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

install(TARGETS zmq_io zmq_hal
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# install(DIRECTORY proto
#     DESTINATION share/${PROJECT_NAME}
#     FILES_MATCHING PATTERN "*.proto")
