// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "teachbot_interfaces/msg/detail/teachbot_state__struct.h"
#include "teachbot_interfaces/msg/detail/teachbot_state__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool teachbot_interfaces__msg__teachbot_pistol_state__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * teachbot_interfaces__msg__teachbot_pistol_state__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool teachbot_interfaces__msg__teachbot_state__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("teachbot_interfaces.msg._teachbot_state.TeachbotState", full_classname_dest, 53) == 0);
  }
  teachbot_interfaces__msg__TeachbotState * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // joint_angles_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "joint_angles_deg");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 6;
      double * dest = ros_message->joint_angles_deg;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // tcp_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "tcp_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tcp_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tcp_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "tcp_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tcp_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tcp_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "tcp_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tcp_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tcp_rx
    PyObject * field = PyObject_GetAttrString(_pymsg, "tcp_rx");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tcp_rx = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tcp_ry
    PyObject * field = PyObject_GetAttrString(_pymsg, "tcp_ry");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tcp_ry = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // tcp_rz
    PyObject * field = PyObject_GetAttrString(_pymsg, "tcp_rz");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->tcp_rz = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pistol
    PyObject * field = PyObject_GetAttrString(_pymsg, "pistol");
    if (!field) {
      return false;
    }
    if (!teachbot_interfaces__msg__teachbot_pistol_state__convert_from_py(field, &ros_message->pistol)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // encoder_errors
    PyObject * field = PyObject_GetAttrString(_pymsg, "encoder_errors");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'encoder_errors'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = 6;
      bool * dest = ros_message->encoder_errors;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyBool_Check(item));
        bool tmp = (item == Py_True);
        memcpy(&dest[i], &tmp, sizeof(bool));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // encoder_warnings
    PyObject * field = PyObject_GetAttrString(_pymsg, "encoder_warnings");
    if (!field) {
      return false;
    }
    {
      PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'encoder_warnings'");
      if (!seq_field) {
        Py_DECREF(field);
        return false;
      }
      Py_ssize_t size = 6;
      bool * dest = ros_message->encoder_warnings;
      for (Py_ssize_t i = 0; i < size; ++i) {
        PyObject * item = PySequence_Fast_GET_ITEM(seq_field, i);
        if (!item) {
          Py_DECREF(seq_field);
          Py_DECREF(field);
          return false;
        }
        assert(PyBool_Check(item));
        bool tmp = (item == Py_True);
        memcpy(&dest[i], &tmp, sizeof(bool));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // encoder_frequencies
    PyObject * field = PyObject_GetAttrString(_pymsg, "encoder_frequencies");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 6;
      double * dest = ros_message->encoder_frequencies;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // robot_model
    PyObject * field = PyObject_GetAttrString(_pymsg, "robot_model");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->robot_model, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * teachbot_interfaces__msg__teachbot_state__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TeachbotState */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("teachbot_interfaces.msg._teachbot_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TeachbotState");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  teachbot_interfaces__msg__TeachbotState * ros_message = (teachbot_interfaces__msg__TeachbotState *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // joint_angles_deg
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "joint_angles_deg");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->joint_angles_deg[0]);
    memcpy(dst, src, 6 * sizeof(double));
    Py_DECREF(field);
  }
  {  // tcp_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tcp_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tcp_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tcp_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tcp_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tcp_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tcp_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tcp_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tcp_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tcp_rx
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tcp_rx);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tcp_rx", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tcp_ry
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tcp_ry);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tcp_ry", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tcp_rz
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->tcp_rz);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tcp_rz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pistol
    PyObject * field = NULL;
    field = teachbot_interfaces__msg__teachbot_pistol_state__convert_to_py(&ros_message->pistol);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pistol", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // encoder_errors
    PyObject * field = NULL;
    size_t size = 6;
    bool * src = ros_message->encoder_errors;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "encoder_errors", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // encoder_warnings
    PyObject * field = NULL;
    size_t size = 6;
    bool * src = ros_message->encoder_warnings;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    for (size_t i = 0; i < size; ++i) {
      int rc = PyList_SetItem(field, i, PyBool_FromLong(src[i] ? 1 : 0));
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "encoder_warnings", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // encoder_frequencies
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "encoder_frequencies");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->encoder_frequencies[0]);
    memcpy(dst, src, 6 * sizeof(double));
    Py_DECREF(field);
  }
  {  // robot_model
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->robot_model.data,
      strlen(ros_message->robot_model.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "robot_model", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
