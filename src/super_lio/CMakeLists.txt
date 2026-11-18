cmake_minimum_required(VERSION 3.16)
project(super_lio)


#DEBUG CONFIGURATION
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g -pthread -fsanitize=address")

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -pthread -fexceptions -flto=auto")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Find packages
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  basic
  pcl_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav_msgs
  message_generation
)
find_package(PythonLibs REQUIRED)

# ============================================================
# GTSAM 链接目标
# ============================================================
# GTSAM 4.x 用的是 CMake exported targets 机制:
#     install(TARGETS gtsam EXPORT GTSAM-exports ...)
#     install(EXPORT GTSAM-exports DESTINATION lib/cmake/GTSAM)   (无 NAMESPACE)
#
# 关键点: 上面这个 install(EXPORT) 【没有写 NAMESPACE】,
#         所以导入的 imported target 名就是裸的 "gtsam", 而不是 "GTSAM::gtsam".
#         这正是 fastlio_sam_sc 直接链裸名 gtsam 能成功的原因.
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
if(TARGET GTSAM::gtsam)
  set(GTSAM_LINK_TARGETS GTSAM::gtsam)
  message(STATUS " ---> GTSAM: using imported target GTSAM::gtsam")
elseif(TARGET Gtsam::gtsam)
  set(GTSAM_LINK_TARGETS Gtsam::gtsam)
  message(STATUS " ---> GTSAM: using imported target Gtsam::gtsam")
elseif(TARGET gtsam)
  # GTSAM 4.2 实际情况: install(EXPORT GTSAM-exports ...) 没有写 NAMESPACE,
  # 所以导入的 imported target 名就是裸的 "gtsam".
  # (fastlio_sam_sc 就是直接链裸名 "gtsam" 工作的)
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

add_message_files(
  FILES
  CloudPose.msg
  CloudPose2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Include directories for libraries
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  include
  3rdparty
)

# Package setup for catkin
# 注意: PCL 的 find_package 是在 basic 包里做的, 本包没有 find_package(PCL),
#       所以这里不能写 "DEPENDS ... PCL" (会报 PCL_INCLUDE_DIRS 未定义的警告).
#       PCL 的头文件/库由 ${catkin_LIBRARIES} 经 basic 包传递进来.
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3
  INCLUDE_DIRS include
)

# LIO library source files
file(GLOB_RECURSE LIO_LIB_SRC "src/lio/*.cpp" "src/ros/*.cpp")

file(GLOB_RECURSE LIO_LIB_HEADERS "include/lio/*.hpp" "include/lio/*.h")

# library build
add_library(lio ${LIO_LIB_SRC} ${LIO_LIB_HEADERS})

# Target specific includes
target_include_directories(lio PUBLIC
  include
  3rdparty
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Linking libraries
target_link_libraries(lio
  ${EIGEN3_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PYTHON_LIBRARIES}
  ${GTSAM_LINK_TARGETS}
  tbb
  gflags
  glog
)


add_executable(super_lio_node src/apps/super_lio_node.cpp)
target_link_libraries(super_lio_node
  lio
  ${catkin_LIBRARIES}
  ${GTSAM_LINK_TARGETS}
)


add_executable(relocation_node src/apps/relocation_node.cpp)
target_link_libraries(relocation_node
  lio
  ${catkin_LIBRARIES}
  ${GTSAM_LINK_TARGETS}
)
