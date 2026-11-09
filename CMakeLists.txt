cmake_minimum_required(VERSION 3.8)
project(xbot2_zmq VERSION 1.0.0)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 20)

option(BUILD_SERVER "Build the zmq_io XBot2 plugin and the zmq_hal device" ON)
option(BUILD_CLIENT "Build the C++ client library (needs no XBot2)" OFF)
option(BUILD_CLIENT_PYTHON "Build the Python bindings of the C++ client" OFF)

if(BUILD_SERVER)
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
endif()

if(BUILD_CLIENT)
    add_subdirectory(cpp_client)
endif()

# install(DIRECTORY proto
#     DESTINATION share/${PROJECT_NAME}
#     FILES_MATCHING PATTERN "*.proto")
