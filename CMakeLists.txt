cmake_minimum_required(VERSION 3.20)
project(xbot2_zmq VERSION 1.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

##
add_library(zmq_io_protoc SHARED 
    ${PROTO_SRCS} ${PROTO_HDRS}
)

target_link_libraries(zmq_io_protoc 
    PRIVATE 
    protobuf::libprotobuf
    -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/zmq_io.ld
    )

set_target_properties(zmq_io_protoc PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/zmq_io.ld)

target_include_directories(zmq_io_protoc
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

# target_compile_options(zmq_io_protoc
#     PUBLIC
#     PRIVATE
#     -fvisibility-inlines-hidden
#     -fvisibility=hidden
#     )

##
add_xbot2_plugin(zmq_io src/zmq_io.cpp)

# target_compile_options(zmq_io
#     PUBLIC
#     PRIVATE
#     -fvisibility-inlines-hidden
#     -fvisibility=hidden)

target_link_libraries(zmq_io
    PRIVATE
    cppzmq
    zmq_io_protoc
    protobuf::libprotobuf
    )

target_include_directories(zmq_io
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

install(TARGETS zmq_io zmq_io_protoc
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY proto
    DESTINATION share/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.proto")
