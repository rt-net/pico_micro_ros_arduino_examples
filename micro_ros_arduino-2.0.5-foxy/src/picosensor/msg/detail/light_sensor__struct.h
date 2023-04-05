// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from picosensor:msg/LightSensor.idl
// generated code does not contain a copyright notice

#ifndef PICOSENSOR__MSG__DETAIL__LIGHT_SENSOR__STRUCT_H_
#define PICOSENSOR__MSG__DETAIL__LIGHT_SENSOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/LightSensor in the package picosensor.
typedef struct picosensor__msg__LightSensor
{
  int16_t right_forward;
  int16_t right_side;
  int16_t left_side;
  int16_t left_forward;
  int16_t battery;
} picosensor__msg__LightSensor;

// Struct for a sequence of picosensor__msg__LightSensor.
typedef struct picosensor__msg__LightSensor__Sequence
{
  picosensor__msg__LightSensor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} picosensor__msg__LightSensor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PICOSENSOR__MSG__DETAIL__LIGHT_SENSOR__STRUCT_H_
