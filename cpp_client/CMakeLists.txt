include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(cppzmq REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(xbot2_zmq_client SHARED src/client.cpp)
add_library(xbot2_zmq::xbot2_zmq_client ALIAS xbot2_zmq_client)
target_compile_features(xbot2_zmq_client PUBLIC cxx_std_17)
target_include_directories(
  xbot2_zmq_client
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(xbot2_zmq_client PRIVATE cppzmq
  $<IF:$<TARGET_EXISTS:yaml-cpp::yaml-cpp>,yaml-cpp::yaml-cpp,yaml-cpp>)

install(TARGETS xbot2_zmq_client EXPORT xbot2_zmq_client-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
set(XBOT2_ZMQ_CLIENT_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/xbot2_zmq_client)
install(EXPORT xbot2_zmq_client-targets NAMESPACE xbot2_zmq::
        DESTINATION ${XBOT2_ZMQ_CLIENT_CMAKE_DIR})
configure_package_config_file(
  cmake/xbot2_zmq_client-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/xbot2_zmq_client-config.cmake
  INSTALL_DESTINATION ${XBOT2_ZMQ_CLIENT_CMAKE_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/xbot2_zmq_client-config.cmake
        DESTINATION ${XBOT2_ZMQ_CLIENT_CMAKE_DIR})

if(BUILD_CLIENT_PYTHON)
  find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import nanobind; print(nanobind.cmake_dir())"
    OUTPUT_VARIABLE nanobind_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(xbot2_zmq_client_py python/module.cpp)
  set_target_properties(xbot2_zmq_client_py PROPERTIES OUTPUT_NAME xbot2_zmq_client)
  target_link_libraries(xbot2_zmq_client_py PRIVATE xbot2_zmq_client)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import sys; print(f'python{sys.version_info[0]}.{sys.version_info[1]}')"
    OUTPUT_VARIABLE XBOT2_ZMQ_PYTHON_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
  install(TARGETS xbot2_zmq_client_py
          LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/${XBOT2_ZMQ_PYTHON_DIR}/site-packages)
endif()
