// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_camera_service_interfaces:srv/ImageCapture.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__STRUCT_HPP_
#define TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Request __declspec(deprecated)
#endif

namespace turtlebot3_camera_service_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ImageCapture_Request_
{
  using Type = ImageCapture_Request_<ContainerAllocator>;

  explicit ImageCapture_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filename = "";
    }
  }

  explicit ImageCapture_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : filename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->filename = "";
    }
  }

  // field types and members
  using _filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _filename_type filename;

  // setters for named parameter idiom
  Type & set__filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->filename = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Request
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Request
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImageCapture_Request_ & other) const
  {
    if (this->filename != other.filename) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImageCapture_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImageCapture_Request_

// alias to use template instance with default allocator
using ImageCapture_Request =
  turtlebot3_camera_service_interfaces::srv::ImageCapture_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_camera_service_interfaces


#ifndef _WIN32
# define DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Response __declspec(deprecated)
#endif

namespace turtlebot3_camera_service_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ImageCapture_Response_
{
  using Type = ImageCapture_Response_<ContainerAllocator>;

  explicit ImageCapture_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ImageCapture_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Response
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_camera_service_interfaces__srv__ImageCapture_Response
    std::shared_ptr<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ImageCapture_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ImageCapture_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ImageCapture_Response_

// alias to use template instance with default allocator
using ImageCapture_Response =
  turtlebot3_camera_service_interfaces::srv::ImageCapture_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_camera_service_interfaces

namespace turtlebot3_camera_service_interfaces
{

namespace srv
{

struct ImageCapture
{
  using Request = turtlebot3_camera_service_interfaces::srv::ImageCapture_Request;
  using Response = turtlebot3_camera_service_interfaces::srv::ImageCapture_Response;
};

}  // namespace srv

}  // namespace turtlebot3_camera_service_interfaces

#endif  // TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__STRUCT_HPP_
