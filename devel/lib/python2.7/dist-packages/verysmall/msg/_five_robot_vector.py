# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from verysmall/five_robot_vector.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class five_robot_vector(genpy.Message):
  _md5sum = "46d2ecd06a68bed7c3b95711f5808fee"
  _type = "verysmall/five_robot_vector"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float64   robot_angle_vector_1
float64   robot_angle_vector_2
float64   robot_angle_vector_3
float64   robot_angle_vector_4
float64   robot_angle_vector_5"""
  __slots__ = ['robot_angle_vector_1','robot_angle_vector_2','robot_angle_vector_3','robot_angle_vector_4','robot_angle_vector_5']
  _slot_types = ['float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       robot_angle_vector_1,robot_angle_vector_2,robot_angle_vector_3,robot_angle_vector_4,robot_angle_vector_5

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(five_robot_vector, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.robot_angle_vector_1 is None:
        self.robot_angle_vector_1 = 0.
      if self.robot_angle_vector_2 is None:
        self.robot_angle_vector_2 = 0.
      if self.robot_angle_vector_3 is None:
        self.robot_angle_vector_3 = 0.
      if self.robot_angle_vector_4 is None:
        self.robot_angle_vector_4 = 0.
      if self.robot_angle_vector_5 is None:
        self.robot_angle_vector_5 = 0.
    else:
      self.robot_angle_vector_1 = 0.
      self.robot_angle_vector_2 = 0.
      self.robot_angle_vector_3 = 0.
      self.robot_angle_vector_4 = 0.
      self.robot_angle_vector_5 = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_5d().pack(_x.robot_angle_vector_1, _x.robot_angle_vector_2, _x.robot_angle_vector_3, _x.robot_angle_vector_4, _x.robot_angle_vector_5))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.robot_angle_vector_1, _x.robot_angle_vector_2, _x.robot_angle_vector_3, _x.robot_angle_vector_4, _x.robot_angle_vector_5,) = _get_struct_5d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_5d().pack(_x.robot_angle_vector_1, _x.robot_angle_vector_2, _x.robot_angle_vector_3, _x.robot_angle_vector_4, _x.robot_angle_vector_5))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 40
      (_x.robot_angle_vector_1, _x.robot_angle_vector_2, _x.robot_angle_vector_3, _x.robot_angle_vector_4, _x.robot_angle_vector_5,) = _get_struct_5d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_5d = None
def _get_struct_5d():
    global _struct_5d
    if _struct_5d is None:
        _struct_5d = struct.Struct("<5d")
    return _struct_5d
