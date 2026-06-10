// ROS2-compatible Livox CustomPoint message header
// This is a standalone implementation to avoid dependency on livox_ros_driver2

#ifndef LIVOX_ROS_DRIVER2_MSG_CUSTOM_POINT_HPP_
#define LIVOX_ROS_DRIVER2_MSG_CUSTOM_POINT_HPP_

#include <cstdint>
#include <cmath>
#include <memory>
#include <ostream>

namespace livox_ros_driver2
{
namespace msg
{

template<class ContainerAllocator>
struct CustomPoint_
{
  using Allocator = ContainerAllocator;

  CustomPoint_()
    : offset_time(0)
    , x(0.0f)
    , y(0.0f)
    , z(0.0f)
    , reflectivity(0)
    , tag(0)
    , line(0) {}

  CustomPoint_(const ContainerAllocator&)
    : offset_time(0)
    , x(0.0f)
    , y(0.0f)
    , z(0.0f)
    , reflectivity(0)
    , tag(0)
    , line(0) {}

  using NoAllocator = CustomPoint_<std::allocator<void>>;

  uint32_t offset_time;
  float x;
  float y;
  float z;
  uint8_t reflectivity;
  uint8_t tag;
  uint8_t line;

  // Padding to match ROS2 message alignment
  uint8_t __padding[1];

  typedef std::shared_ptr<CustomPoint_<ContainerAllocator>> SharedPtr;
  typedef std::shared_ptr<CustomPoint_<ContainerAllocator> const> ConstSharedPtr;
  typedef std::shared_ptr<CustomPoint_<ContainerAllocator>> UniquePtr;
  typedef std::shared_ptr<CustomPoint_<ContainerAllocator> const> ConstUniquePtr;
};

typedef CustomPoint_<std::allocator<void>> CustomPoint;

typedef std::shared_ptr<CustomPoint> CustomPointPtr;
typedef std::shared_ptr<CustomPoint const> CustomPointConstPtr;
typedef std::shared_ptr<CustomPoint_<std::allocator<void>>> CustomPointUniquePtr;
typedef std::shared_ptr<CustomPoint_<std::allocator<void>> const> CustomPointConstUniquePtr;

}  // namespace msg
}  // namespace livox_ros_driver2

// Comparison operators
template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const livox_ros_driver2::msg::CustomPoint_<ContainerAllocator1>& lhs,
                const livox_ros_driver2::msg::CustomPoint_<ContainerAllocator2>& rhs)
{
  return lhs.offset_time == rhs.offset_time &&
         lhs.x == rhs.x &&
         lhs.y == rhs.y &&
         lhs.z == rhs.z &&
         lhs.reflectivity == rhs.reflectivity &&
         lhs.tag == rhs.tag &&
         lhs.line == rhs.line;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const livox_ros_driver2::msg::CustomPoint_<ContainerAllocator1>& lhs,
                const livox_ros_driver2::msg::CustomPoint_<ContainerAllocator2>& rhs)
{
  return !(lhs == rhs);
}

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const livox_ros_driver2::msg::CustomPoint_<ContainerAllocator>& v)
{
  s << "CustomPoint(";
  s << "offset_time: " << v.offset_time << ", ";
  s << "x: " << v.x << ", ";
  s << "y: " << v.y << ", ";
  s << "z: " << v.z << ", ";
  s << "reflectivity: " << (int)v.reflectivity << ", ";
  s << "tag: " << (int)v.tag << ", ";
  s << "line: " << (int)v.line << ")";
  return s;
}

#endif  // LIVOX_ROS_DRIVER2_MSG_CUSTOM_POINT_HPP_

