// ROS2-compatible Livox CustomMsg message header
// This is a standalone implementation to avoid dependency on livox_ros_driver2

#ifndef LIVOX_ROS_DRIVER2_MSG_CUSTOM_MSG_HPP_
#define LIVOX_ROS_DRIVER2_MSG_CUSTOM_MSG_HPP_

#include <vector>
#include <memory>
#include <cstdint>
#include <ostream>

#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/header.hpp>

#include "custom_point.hpp"

namespace livox_ros_driver2
{
namespace msg
{

template<class ContainerAllocator>
struct CustomMsg_
{
  using Allocator = ContainerAllocator;

  CustomMsg_()
    : timebase(0)
    , point_num(0)
    , lidar_id(0)
    , rsvd{0, 0, 0}
    , points() {}

  CustomMsg_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , timebase(0)
    , point_num(0)
    , lidar_id(0)
    , rsvd{0, 0, 0}
    , points(_alloc) {}

  using NoAllocator = CustomMsg_<std::allocator<void>>;

  std_msgs::msg::Header_<ContainerAllocator> header;
  uint64_t timebase;
  uint32_t point_num;
  uint8_t lidar_id;
  uint8_t rsvd[3];
  std::vector<CustomPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<CustomPoint_<ContainerAllocator>>> points;

  typedef std::shared_ptr<CustomMsg_<ContainerAllocator>> SharedPtr;
  typedef std::shared_ptr<CustomMsg_<ContainerAllocator> const> ConstSharedPtr;
  typedef std::shared_ptr<CustomMsg_<ContainerAllocator>> UniquePtr;
  typedef std::shared_ptr<CustomMsg_<ContainerAllocator> const> ConstUniquePtr;
};

typedef CustomMsg_<std::allocator<void>> CustomMsg;

typedef std::shared_ptr<CustomMsg> CustomMsgPtr;
typedef std::shared_ptr<CustomMsg const> CustomMsgConstPtr;
typedef std::shared_ptr<CustomMsg_<std::allocator<void>>> CustomMsgUniquePtr;
typedef std::shared_ptr<CustomMsg_<std::allocator<void>> const> CustomMsgConstUniquePtr;

}  // namespace msg
}  // namespace livox_ros_driver2

// Comparison operators
template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const livox_ros_driver2::msg::CustomMsg_<ContainerAllocator1>& lhs,
                const livox_ros_driver2::msg::CustomMsg_<ContainerAllocator2>& rhs)
{
  return lhs.header == rhs.header &&
         lhs.timebase == rhs.timebase &&
         lhs.point_num == rhs.point_num &&
         lhs.lidar_id == rhs.lidar_id &&
         lhs.rsvd[0] == rhs.rsvd[0] &&
         lhs.rsvd[1] == rhs.rsvd[1] &&
         lhs.rsvd[2] == rhs.rsvd[2] &&
         lhs.points == rhs.points;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const livox_ros_driver2::msg::CustomMsg_<ContainerAllocator1>& lhs,
                const livox_ros_driver2::msg::CustomMsg_<ContainerAllocator2>& rhs)
{
  return !(lhs == rhs);
}

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const livox_ros_driver2::msg::CustomMsg_<ContainerAllocator>& v)
{
  s << "CustomMsg(";
  s << "header: " << v.header << ", ";
  s << "timebase: " << v.timebase << ", ";
  s << "point_num: " << v.point_num << ", ";
  s << "lidar_id: " << (int)v.lidar_id << ", ";
  s << "points: [" << v.points.size() << " points])";
  return s;
}

#endif  // LIVOX_ROS_DRIVER2_MSG_CUSTOM_MSG_HPP_

