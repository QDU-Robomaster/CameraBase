cmake_minimum_required(VERSION 3.16)
project(CameraGeometryTests LANGUAGES CXX)

find_package(Threads REQUIRED)

add_executable(camera_geometry_test camera_geometry_test.cpp)
add_executable(camera_image_sink_test camera_image_sink_test.cpp)
add_executable(camera_intrinsic_sanity_test camera_intrinsic_sanity_test.cpp)
add_executable(camera_replay_adapter_test camera_replay_adapter_test.cpp)

target_include_directories(camera_geometry_test
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_include_directories(camera_image_sink_test
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_include_directories(camera_intrinsic_sanity_test
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_include_directories(camera_replay_adapter_test
                           PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_link_libraries(camera_geometry_test PRIVATE xr)
target_link_libraries(camera_image_sink_test PRIVATE xr Threads::Threads)
target_link_libraries(camera_intrinsic_sanity_test PRIVATE xr)
target_compile_features(camera_geometry_test PRIVATE cxx_std_20)
target_compile_features(camera_image_sink_test PRIVATE cxx_std_20)
target_compile_features(camera_intrinsic_sanity_test PRIVATE cxx_std_20)
target_compile_features(camera_replay_adapter_test PRIVATE cxx_std_20)

add_test(NAME camera_geometry_test COMMAND camera_geometry_test)
add_test(NAME camera_image_sink_test COMMAND camera_image_sink_test)
add_test(NAME camera_intrinsic_sanity_test COMMAND camera_intrinsic_sanity_test)
add_test(NAME camera_replay_adapter_test COMMAND camera_replay_adapter_test)
set_tests_properties(camera_image_sink_test PROPERTIES TIMEOUT 5)
