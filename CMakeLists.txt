# CameraBase 模块 CMake 配置

# 导出本模块头文件。
target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# 自动纳入本模块源文件。
file(
  GLOB
  MODULE_CAMERABASE_SRC
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/*.cxx"
  "${CMAKE_CURRENT_LIST_DIR}/*.cc"
  "${CMAKE_CURRENT_LIST_DIR}/*.c")

target_sources(xr PRIVATE ${MODULE_CAMERABASE_SRC})

# 模块依赖壳。
get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)

# target_compile_definitions(${_DEPS_TARGET} INTERFACE # FOO=1 )
# target_compile_options(${_DEPS_TARGET} INTERFACE # -Wall )

# 注册到全局模块依赖集合。
set_property(GLOBAL APPEND PROPERTY XR_MODULE_DEPS ${_DEPS_TARGET})

if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests
                   ${CMAKE_CURRENT_BINARY_DIR}/CameraBase_tests)
endif()
