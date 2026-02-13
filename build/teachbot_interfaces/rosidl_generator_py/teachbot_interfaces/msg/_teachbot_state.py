# generated from rosidl_generator_py/resource/_idl.py.em
# with input from teachbot_interfaces:msg/TeachbotState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'joint_angles_deg'
# Member 'encoder_frequencies'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TeachbotState(type):
    """Metaclass of message 'TeachbotState'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('teachbot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'teachbot_interfaces.msg.TeachbotState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__teachbot_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__teachbot_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__teachbot_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__teachbot_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__teachbot_state

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from teachbot_interfaces.msg import TeachbotPistolState
            if TeachbotPistolState.__class__._TYPE_SUPPORT is None:
                TeachbotPistolState.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TeachbotState(metaclass=Metaclass_TeachbotState):
    """Message class 'TeachbotState'."""

    __slots__ = [
        '_header',
        '_joint_angles_deg',
        '_tcp_x',
        '_tcp_y',
        '_tcp_z',
        '_tcp_rx',
        '_tcp_ry',
        '_tcp_rz',
        '_pistol',
        '_encoder_errors',
        '_encoder_warnings',
        '_encoder_frequencies',
        '_robot_model',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'joint_angles_deg': 'double[6]',
        'tcp_x': 'double',
        'tcp_y': 'double',
        'tcp_z': 'double',
        'tcp_rx': 'double',
        'tcp_ry': 'double',
        'tcp_rz': 'double',
        'pistol': 'teachbot_interfaces/TeachbotPistolState',
        'encoder_errors': 'boolean[6]',
        'encoder_warnings': 'boolean[6]',
        'encoder_frequencies': 'double[6]',
        'robot_model': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['teachbot_interfaces', 'msg'], 'TeachbotPistolState'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 6),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        if 'joint_angles_deg' not in kwargs:
            self.joint_angles_deg = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.joint_angles_deg = kwargs.get('joint_angles_deg')
        self.tcp_x = kwargs.get('tcp_x', float())
        self.tcp_y = kwargs.get('tcp_y', float())
        self.tcp_z = kwargs.get('tcp_z', float())
        self.tcp_rx = kwargs.get('tcp_rx', float())
        self.tcp_ry = kwargs.get('tcp_ry', float())
        self.tcp_rz = kwargs.get('tcp_rz', float())
        from teachbot_interfaces.msg import TeachbotPistolState
        self.pistol = kwargs.get('pistol', TeachbotPistolState())
        self.encoder_errors = kwargs.get(
            'encoder_errors',
            [bool() for x in range(6)]
        )
        self.encoder_warnings = kwargs.get(
            'encoder_warnings',
            [bool() for x in range(6)]
        )
        if 'encoder_frequencies' not in kwargs:
            self.encoder_frequencies = numpy.zeros(6, dtype=numpy.float64)
        else:
            self.encoder_frequencies = kwargs.get('encoder_frequencies')
        self.robot_model = kwargs.get('robot_model', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if any(self.joint_angles_deg != other.joint_angles_deg):
            return False
        if self.tcp_x != other.tcp_x:
            return False
        if self.tcp_y != other.tcp_y:
            return False
        if self.tcp_z != other.tcp_z:
            return False
        if self.tcp_rx != other.tcp_rx:
            return False
        if self.tcp_ry != other.tcp_ry:
            return False
        if self.tcp_rz != other.tcp_rz:
            return False
        if self.pistol != other.pistol:
            return False
        if self.encoder_errors != other.encoder_errors:
            return False
        if self.encoder_warnings != other.encoder_warnings:
            return False
        if any(self.encoder_frequencies != other.encoder_frequencies):
            return False
        if self.robot_model != other.robot_model:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def joint_angles_deg(self):
        """Message field 'joint_angles_deg'."""
        return self._joint_angles_deg

    @joint_angles_deg.setter
    def joint_angles_deg(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'joint_angles_deg' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 6, \
                "The 'joint_angles_deg' numpy.ndarray() must have a size of 6"
            self._joint_angles_deg = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'joint_angles_deg' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._joint_angles_deg = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def tcp_x(self):
        """Message field 'tcp_x'."""
        return self._tcp_x

    @tcp_x.setter
    def tcp_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tcp_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tcp_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tcp_x = value

    @builtins.property
    def tcp_y(self):
        """Message field 'tcp_y'."""
        return self._tcp_y

    @tcp_y.setter
    def tcp_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tcp_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tcp_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tcp_y = value

    @builtins.property
    def tcp_z(self):
        """Message field 'tcp_z'."""
        return self._tcp_z

    @tcp_z.setter
    def tcp_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tcp_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tcp_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tcp_z = value

    @builtins.property
    def tcp_rx(self):
        """Message field 'tcp_rx'."""
        return self._tcp_rx

    @tcp_rx.setter
    def tcp_rx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tcp_rx' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tcp_rx' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tcp_rx = value

    @builtins.property
    def tcp_ry(self):
        """Message field 'tcp_ry'."""
        return self._tcp_ry

    @tcp_ry.setter
    def tcp_ry(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tcp_ry' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tcp_ry' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tcp_ry = value

    @builtins.property
    def tcp_rz(self):
        """Message field 'tcp_rz'."""
        return self._tcp_rz

    @tcp_rz.setter
    def tcp_rz(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tcp_rz' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tcp_rz' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tcp_rz = value

    @builtins.property
    def pistol(self):
        """Message field 'pistol'."""
        return self._pistol

    @pistol.setter
    def pistol(self, value):
        if __debug__:
            from teachbot_interfaces.msg import TeachbotPistolState
            assert \
                isinstance(value, TeachbotPistolState), \
                "The 'pistol' field must be a sub message of type 'TeachbotPistolState'"
        self._pistol = value

    @builtins.property
    def encoder_errors(self):
        """Message field 'encoder_errors'."""
        return self._encoder_errors

    @encoder_errors.setter
    def encoder_errors(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'encoder_errors' field must be a set or sequence with length 6 and each value of type 'bool'"
        self._encoder_errors = value

    @builtins.property
    def encoder_warnings(self):
        """Message field 'encoder_warnings'."""
        return self._encoder_warnings

    @encoder_warnings.setter
    def encoder_warnings(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'encoder_warnings' field must be a set or sequence with length 6 and each value of type 'bool'"
        self._encoder_warnings = value

    @builtins.property
    def encoder_frequencies(self):
        """Message field 'encoder_frequencies'."""
        return self._encoder_frequencies

    @encoder_frequencies.setter
    def encoder_frequencies(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'encoder_frequencies' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 6, \
                "The 'encoder_frequencies' numpy.ndarray() must have a size of 6"
            self._encoder_frequencies = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'encoder_frequencies' field must be a set or sequence with length 6 and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._encoder_frequencies = numpy.array(value, dtype=numpy.float64)

    @builtins.property
    def robot_model(self):
        """Message field 'robot_model'."""
        return self._robot_model

    @robot_model.setter
    def robot_model(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_model' field must be of type 'str'"
        self._robot_model = value
