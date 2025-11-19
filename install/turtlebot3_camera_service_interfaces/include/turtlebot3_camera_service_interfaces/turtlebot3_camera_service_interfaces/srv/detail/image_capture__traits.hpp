// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from turtlebot3_camera_service_interfaces:srv/ImageCapture.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__TRAITS_HPP_
#define TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "turtlebot3_camera_service_interfaces/srv/detail/image_capture__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace turtlebot3_camera_service_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ImageCapture_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: filename
  {
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImageCapture_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImageCapture_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_camera_service_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_camera_service_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_camera_service_interfaces::srv::ImageCapture_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_camera_service_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_camera_service_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_camera_service_interfaces::srv::ImageCapture_Request & msg)
{
  return turtlebot3_camera_service_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>()
{
  return "turtlebot3_camera_service_interfaces::srv::ImageCapture_Request";
}

template<>
inline const char * name<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>()
{
  return "turtlebot3_camera_service_interfaces/srv/ImageCapture_Request";
}

template<>
struct has_fixed_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace turtlebot3_camera_service_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ImageCapture_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImageCapture_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImageCapture_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace turtlebot3_camera_service_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use turtlebot3_camera_service_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const turtlebot3_camera_service_interfaces::srv::ImageCapture_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  turtlebot3_camera_service_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use turtlebot3_camera_service_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const turtlebot3_camera_service_interfaces::srv::ImageCapture_Response & msg)
{
  return turtlebot3_camera_service_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>()
{
  return "turtlebot3_camera_service_interfaces::srv::ImageCapture_Response";
}

template<>
inline const char * name<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>()
{
  return "turtlebot3_camera_service_interfaces/srv/ImageCapture_Response";
}

template<>
struct has_fixed_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<turtlebot3_camera_service_interfaces::srv::ImageCapture>()
{
  return "turtlebot3_camera_service_interfaces::srv::ImageCapture";
}

template<>
inline const char * name<turtlebot3_camera_service_interfaces::srv::ImageCapture>()
{
  return "turtlebot3_camera_service_interfaces/srv/ImageCapture";
}

template<>
struct has_fixed_size<turtlebot3_camera_service_interfaces::srv::ImageCapture>
  : std::integral_constant<
    bool,
    has_fixed_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>::value &&
    has_fixed_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>::value
  >
{
};

template<>
struct has_bounded_size<turtlebot3_camera_service_interfaces::srv::ImageCapture>
  : std::integral_constant<
    bool,
    has_bounded_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>::value &&
    has_bounded_size<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>::value
  >
{
};

template<>
struct is_service<turtlebot3_camera_service_interfaces::srv::ImageCapture>
  : std::true_type
{
};

template<>
struct is_service_request<turtlebot3_camera_service_interfaces::srv::ImageCapture_Request>
  : std::true_type
{
};

template<>
struct is_service_response<turtlebot3_camera_service_interfaces::srv::ImageCapture_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__TRAITS_HPP_
