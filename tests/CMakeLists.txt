cmake_minimum_required(VERSION 3.16)
project(CameraGeometryTests LANGUAGES CXX)

add_executable(camera_geometry_test camera_geometry_test.cpp)

target_include_directories(camera_geometry_test
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(camera_geometry_test PRIVATE xr)
target_compile_features(camera_geometry_test PRIVATE cxx_std_20)

add_test(NAME camera_geometry_test COMMAND camera_geometry_test)
