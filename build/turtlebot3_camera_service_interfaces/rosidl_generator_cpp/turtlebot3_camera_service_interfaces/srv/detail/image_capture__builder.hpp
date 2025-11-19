// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from turtlebot3_camera_service_interfaces:srv/ImageCapture.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__BUILDER_HPP_
#define TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "turtlebot3_camera_service_interfaces/srv/detail/image_capture__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace turtlebot3_camera_service_interfaces
{

namespace srv
{

namespace builder
{

class Init_ImageCapture_Request_filename
{
public:
  Init_ImageCapture_Request_filename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::turtlebot3_camera_service_interfaces::srv::ImageCapture_Request filename(::turtlebot3_camera_service_interfaces::srv::ImageCapture_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_camera_service_interfaces::srv::ImageCapture_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>()
{
  return turtlebot3_camera_service_interfaces::srv::builder::Init_ImageCapture_Request_filename();
}

}  // namespace turtlebot3_camera_service_interfaces


namespace turtlebot3_camera_service_interfaces
{

namespace srv
{

namespace builder
{

class Init_ImageCapture_Response_message
{
public:
  explicit Init_ImageCapture_Response_message(::turtlebot3_camera_service_interfaces::srv::ImageCapture_Response & msg)
  : msg_(msg)
  {}
  ::turtlebot3_camera_service_interfaces::srv::ImageCapture_Response message(::turtlebot3_camera_service_interfaces::srv::ImageCapture_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::turtlebot3_camera_service_interfaces::srv::ImageCapture_Response msg_;
};

class Init_ImageCapture_Response_success
{
public:
  Init_ImageCapture_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImageCapture_Response_message success(::turtlebot3_camera_service_interfaces::srv::ImageCapture_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ImageCapture_Response_message(msg_);
  }

private:
  ::turtlebot3_camera_service_interfaces::srv::ImageCapture_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>()
{
  return turtlebot3_camera_service_interfaces::srv::builder::Init_ImageCapture_Response_success();
}

}  // namespace turtlebot3_camera_service_interfaces

#endif  // TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__BUILDER_HPP_
