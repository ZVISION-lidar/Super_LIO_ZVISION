cmake_minimum_required(VERSION 3.16)
project(super_lio VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE STRING "Suppress developer warnings" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions -flto=auto")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -fPIC")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(Glog REQUIRED)
find_package(basic REQUIRED)

# ============================================================
# GTSAM 链接目标
# ============================================================
# GTSAM 4.x 用的是 CMake exported targets 机制:
#     install(TARGETS gtsam EXPORT GTSAM-exports ...)
#     install(EXPORT GTSAM-exports DESTINATION lib/cmake/GTSAM)   (无 NAMESPACE)
#
# 关键点: 上面这个 install(EXPORT) 【没有写 NAMESPACE】,
#         所以导入的 imported target 名就是裸的 "gtsam", 而不是 "GTSAM::gtsam".
#
# 另外, GTSAM 4.2 的 GTSAMConfig.cmake 只定义了 GTSAM_INCLUDE_DIR,
# 根本没有 GTSAM_LIBRARIES 变量 -> 用 ${GTSAM_LIBRARIES} 会拿到空字符串
# -> 链接期报一堆 undefined reference to gtsam::...
#
# 这里做多级兼容, 任意一种安装形态都能命中:
#   1. imported target  GTSAM::gtsam / Gtsam::gtsam  (带命名空间的版本)
#   2. imported target  gtsam                          (GTSAM 4.2 实际情况)
#   3. GTSAM_LIBRARIES                                  (GTSAM 3.x)
#   4. find_library 直接找 libgtsam.so                  (兜底)
# ============================================================
find_package(GTSAM REQUIRED QUIET)
if(TARGET GTSAM::gtsam)
  set(GTSAM_LINK_TARGETS GTSAM::gtsam)
  message(STATUS " ---> GTSAM: using imported target GTSAM::gtsam")
elseif(TARGET Gtsam::gtsam)
  set(GTSAM_LINK_TARGETS Gtsam::gtsam)
  message(STATUS " ---> GTSAM: using imported target Gtsam::gtsam")
elseif(TARGET gtsam)
  # GTSAM 4.2 实际情况: install(EXPORT GTSAM-exports ...) 没有写 NAMESPACE,
  # 所以导入的 imported target 名就是裸的 "gtsam".
  set(GTSAM_LINK_TARGETS gtsam)
  message(STATUS " ---> GTSAM: using imported target gtsam")
elseif(TARGET GTSAM::gtsam_shared)
  set(GTSAM_LINK_TARGETS GTSAM::gtsam_shared)
  message(STATUS " ---> GTSAM: using imported target GTSAM::gtsam_shared")
elseif(DEFINED GTSAM_LIBRARIES AND NOT "${GTSAM_LIBRARIES}" STREQUAL "")
  set(GTSAM_LINK_TARGETS ${GTSAM_LIBRARIES})
  message(STATUS " ---> GTSAM: using GTSAM_LIBRARIES = ${GTSAM_LIBRARIES}")
else()
  # 兜底: 直接找库文件 (与 fastlio_sam_sc 的做法一致).
  find_library(GTSAM_LIBRARY NAMES gtsam
               PATHS ${GTSAM_LIBRARY_DIRS}
                     /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
               NO_DEFAULT_PATH)
  if(NOT GTSAM_LIBRARY)
    # 再放开默认搜索路径试一次.
    find_library(GTSAM_LIBRARY NAMES gtsam)
  endif()
  if(NOT GTSAM_LIBRARY)
    message(FATAL_ERROR
      "GTSAM not found! Install GTSAM 4.x, or set the search path, e.g. "
      "cmake -DGTSAM_DIR=/usr/local/lib/cmake/GTSAM")
  endif()
  set(GTSAM_LINK_TARGETS ${GTSAM_LIBRARY})
  message(STATUS " ---> GTSAM: using library ${GTSAM_LIBRARY}")
endif()

# 保证裸名 "gtsam" 也能被链接器找到 (与 fastlio_sam_sc 保持一致).
if(GTSAM_LIBRARY_DIRS)
  link_directories(${GTSAM_LIBRARY_DIRS})
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CloudPose.msg
  msg/CloudPose2.msg
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

include_directories(
  include
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Glog_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp" "src/ros/*.cpp")
file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h")

add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS})

ament_target_dependencies(lio PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  pcl_ros
  basic
  rosidl_default_runtime
  Boost
)

target_link_libraries(lio PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog
  pthread
  ${cpp_typesupport_target}
  ${GTSAM_LINK_TARGETS}
)

add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node
  lio
  ${cpp_typesupport_target}
)

add_executable(relocation_node src/apps/relocation_node.cpp)
target_link_libraries(relocation_node
  lio
  ${cpp_typesupport_target}
)

install(TARGETS
  lio
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(TARGETS
  super_lio_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  relocation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  3rdparty/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()

