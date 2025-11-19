// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from turtlebot3_camera_service_interfaces:srv/ImageCapture.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__STRUCT_H_
#define TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'filename'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ImageCapture in the package turtlebot3_camera_service_interfaces.
typedef struct turtlebot3_camera_service_interfaces__srv__ImageCapture_Request
{
  rosidl_runtime_c__String filename;
} turtlebot3_camera_service_interfaces__srv__ImageCapture_Request;

// Struct for a sequence of turtlebot3_camera_service_interfaces__srv__ImageCapture_Request.
typedef struct turtlebot3_camera_service_interfaces__srv__ImageCapture_Request__Sequence
{
  turtlebot3_camera_service_interfaces__srv__ImageCapture_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_camera_service_interfaces__srv__ImageCapture_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ImageCapture in the package turtlebot3_camera_service_interfaces.
typedef struct turtlebot3_camera_service_interfaces__srv__ImageCapture_Response
{
  bool success;
  rosidl_runtime_c__String message;
} turtlebot3_camera_service_interfaces__srv__ImageCapture_Response;

// Struct for a sequence of turtlebot3_camera_service_interfaces__srv__ImageCapture_Response.
typedef struct turtlebot3_camera_service_interfaces__srv__ImageCapture_Response__Sequence
{
  turtlebot3_camera_service_interfaces__srv__ImageCapture_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} turtlebot3_camera_service_interfaces__srv__ImageCapture_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TURTLEBOT3_CAMERA_SERVICE_INTERFACES__SRV__DETAIL__IMAGE_CAPTURE__STRUCT_H_
