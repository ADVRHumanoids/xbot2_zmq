cmake_minimum_required(VERSION 3.20)
project(xbot2_zmq VERSION 1.0.0)

find_package(xbot2 REQUIRED)
find_package(cppzmq REQUIRED)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(
    PROTO_SRCS PROTO_HDRS
    proto/generic_rx_msg.proto
    proto/jointstate.proto
    proto/imu.proto
    proto/jointcmd.proto
    )

message(STATUS "PROTO_HDRS: ${PROTO_HDRS}"
)
add_xbot2_plugin(zmq_io src/zmq_io.cpp ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(zmq_io
    PRIVATE
    cppzmq
    protobuf::libprotobuf
    )

target_include_directories(zmq_io
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

install(TARGETS zmq_io
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY proto
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.proto")
