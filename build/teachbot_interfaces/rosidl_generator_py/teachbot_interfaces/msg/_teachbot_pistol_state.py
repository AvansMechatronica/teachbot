# generated from rosidl_generator_py/resource/_idl.py.em
# with input from teachbot_interfaces:msg/TeachbotPistolState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TeachbotPistolState(type):
    """Metaclass of message 'TeachbotPistolState'."""

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
                'teachbot_interfaces.msg.TeachbotPistolState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__teachbot_pistol_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__teachbot_pistol_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__teachbot_pistol_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__teachbot_pistol_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__teachbot_pistol_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TeachbotPistolState(metaclass=Metaclass_TeachbotPistolState):
    """Message class 'TeachbotPistolState'."""

    __slots__ = [
        '_pot_raw',
        '_pot_percent',
        '_btn1',
        '_btn2',
    ]

    _fields_and_field_types = {
        'pot_raw': 'int32',
        'pot_percent': 'int32',
        'btn1': 'boolean',
        'btn2': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pot_raw = kwargs.get('pot_raw', int())
        self.pot_percent = kwargs.get('pot_percent', int())
        self.btn1 = kwargs.get('btn1', bool())
        self.btn2 = kwargs.get('btn2', bool())

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
        if self.pot_raw != other.pot_raw:
            return False
        if self.pot_percent != other.pot_percent:
            return False
        if self.btn1 != other.btn1:
            return False
        if self.btn2 != other.btn2:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pot_raw(self):
        """Message field 'pot_raw'."""
        return self._pot_raw

    @pot_raw.setter
    def pot_raw(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pot_raw' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'pot_raw' field must be an integer in [-2147483648, 2147483647]"
        self._pot_raw = value

    @builtins.property
    def pot_percent(self):
        """Message field 'pot_percent'."""
        return self._pot_percent

    @pot_percent.setter
    def pot_percent(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pot_percent' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'pot_percent' field must be an integer in [-2147483648, 2147483647]"
        self._pot_percent = value

    @builtins.property
    def btn1(self):
        """Message field 'btn1'."""
        return self._btn1

    @btn1.setter
    def btn1(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'btn1' field must be of type 'bool'"
        self._btn1 = value

    @builtins.property
    def btn2(self):
        """Message field 'btn2'."""
        return self._btn2

    @btn2.setter
    def btn2(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'btn2' field must be of type 'bool'"
        self._btn2 = value
