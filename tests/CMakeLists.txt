cmake_minimum_required(VERSION 3.16)
project(CameraBaseTests LANGUAGES CXX)

find_package(Threads REQUIRED)

add_executable(camera_types_test camera_types_test.cpp)
add_executable(camera_base_test camera_base_test.cpp)

foreach(test camera_types_test camera_base_test)
  target_include_directories(${test} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
  target_link_libraries(${test} PRIVATE xr Threads::Threads)
  target_compile_features(${test} PRIVATE cxx_std_20)
  add_test(NAME ${test} COMMAND ${test})
endforeach()
set_tests_properties(camera_base_test PROPERTIES TIMEOUT 20)
