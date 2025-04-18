// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from realsense2_camera_msgs:srv/CalibConfigRead.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_READ__FUNCTIONS_H_
#define REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_READ__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "realsense2_camera_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "realsense2_camera_msgs/srv/detail/calib_config_read__struct.h"

/// Initialize srv/CalibConfigRead message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * realsense2_camera_msgs__srv__CalibConfigRead_Request
 * )) before or use
 * realsense2_camera_msgs__srv__CalibConfigRead_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Request__init(realsense2_camera_msgs__srv__CalibConfigRead_Request * msg);

/// Finalize srv/CalibConfigRead message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Request__fini(realsense2_camera_msgs__srv__CalibConfigRead_Request * msg);

/// Create srv/CalibConfigRead message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
realsense2_camera_msgs__srv__CalibConfigRead_Request *
realsense2_camera_msgs__srv__CalibConfigRead_Request__create();

/// Destroy srv/CalibConfigRead message.
/**
 * It calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Request__destroy(realsense2_camera_msgs__srv__CalibConfigRead_Request * msg);

/// Check for srv/CalibConfigRead message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Request__are_equal(const realsense2_camera_msgs__srv__CalibConfigRead_Request * lhs, const realsense2_camera_msgs__srv__CalibConfigRead_Request * rhs);

/// Copy a srv/CalibConfigRead message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Request__copy(
  const realsense2_camera_msgs__srv__CalibConfigRead_Request * input,
  realsense2_camera_msgs__srv__CalibConfigRead_Request * output);

/// Initialize array of srv/CalibConfigRead messages.
/**
 * It allocates the memory for the number of elements and calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__init(realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence * array, size_t size);

/// Finalize array of srv/CalibConfigRead messages.
/**
 * It calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__fini(realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence * array);

/// Create array of srv/CalibConfigRead messages.
/**
 * It allocates the memory for the array and calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence *
realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__create(size_t size);

/// Destroy array of srv/CalibConfigRead messages.
/**
 * It calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__destroy(realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence * array);

/// Check for srv/CalibConfigRead message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__are_equal(const realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence * lhs, const realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence * rhs);

/// Copy an array of srv/CalibConfigRead messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence__copy(
  const realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence * input,
  realsense2_camera_msgs__srv__CalibConfigRead_Request__Sequence * output);

/// Initialize srv/CalibConfigRead message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * realsense2_camera_msgs__srv__CalibConfigRead_Response
 * )) before or use
 * realsense2_camera_msgs__srv__CalibConfigRead_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Response__init(realsense2_camera_msgs__srv__CalibConfigRead_Response * msg);

/// Finalize srv/CalibConfigRead message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Response__fini(realsense2_camera_msgs__srv__CalibConfigRead_Response * msg);

/// Create srv/CalibConfigRead message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
realsense2_camera_msgs__srv__CalibConfigRead_Response *
realsense2_camera_msgs__srv__CalibConfigRead_Response__create();

/// Destroy srv/CalibConfigRead message.
/**
 * It calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Response__destroy(realsense2_camera_msgs__srv__CalibConfigRead_Response * msg);

/// Check for srv/CalibConfigRead message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Response__are_equal(const realsense2_camera_msgs__srv__CalibConfigRead_Response * lhs, const realsense2_camera_msgs__srv__CalibConfigRead_Response * rhs);

/// Copy a srv/CalibConfigRead message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Response__copy(
  const realsense2_camera_msgs__srv__CalibConfigRead_Response * input,
  realsense2_camera_msgs__srv__CalibConfigRead_Response * output);

/// Initialize array of srv/CalibConfigRead messages.
/**
 * It allocates the memory for the number of elements and calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__init(realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence * array, size_t size);

/// Finalize array of srv/CalibConfigRead messages.
/**
 * It calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__fini(realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence * array);

/// Create array of srv/CalibConfigRead messages.
/**
 * It allocates the memory for the array and calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence *
realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__create(size_t size);

/// Destroy array of srv/CalibConfigRead messages.
/**
 * It calls
 * realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
void
realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__destroy(realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence * array);

/// Check for srv/CalibConfigRead message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__are_equal(const realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence * lhs, const realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence * rhs);

/// Copy an array of srv/CalibConfigRead messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_realsense2_camera_msgs
bool
realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence__copy(
  const realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence * input,
  realsense2_camera_msgs__srv__CalibConfigRead_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // REALSENSE2_CAMERA_MSGS__SRV__DETAIL__CALIB_CONFIG_READ__FUNCTIONS_H_
