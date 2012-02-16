"""autogenerated by genmsg_py from Lighting.msg. Do not edit."""
import roslib.message
import struct


class Lighting(roslib.message.Message):
  _md5sum = "8466f8703abc19a467a2579c653659e8"
  _type = "rover_msgs/Lighting"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 red_duty_cycle
float32 red_current
float32 green_duty_cycle
float32 green_current
float32 blue_duty_cycle
float32 blue_current

"""
  __slots__ = ['red_duty_cycle','red_current','green_duty_cycle','green_current','blue_duty_cycle','blue_current']
  _slot_types = ['float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       red_duty_cycle,red_current,green_duty_cycle,green_current,blue_duty_cycle,blue_current
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(Lighting, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.red_duty_cycle is None:
        self.red_duty_cycle = 0.
      if self.red_current is None:
        self.red_current = 0.
      if self.green_duty_cycle is None:
        self.green_duty_cycle = 0.
      if self.green_current is None:
        self.green_current = 0.
      if self.blue_duty_cycle is None:
        self.blue_duty_cycle = 0.
      if self.blue_current is None:
        self.blue_current = 0.
    else:
      self.red_duty_cycle = 0.
      self.red_current = 0.
      self.green_duty_cycle = 0.
      self.green_current = 0.
      self.blue_duty_cycle = 0.
      self.blue_current = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_6f.pack(_x.red_duty_cycle, _x.red_current, _x.green_duty_cycle, _x.green_current, _x.blue_duty_cycle, _x.blue_current))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.red_duty_cycle, _x.red_current, _x.green_duty_cycle, _x.green_current, _x.blue_duty_cycle, _x.blue_current,) = _struct_6f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_6f.pack(_x.red_duty_cycle, _x.red_current, _x.green_duty_cycle, _x.green_current, _x.blue_duty_cycle, _x.blue_current))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.red_duty_cycle, _x.red_current, _x.green_duty_cycle, _x.green_current, _x.blue_duty_cycle, _x.blue_current,) = _struct_6f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_6f = struct.Struct("<6f")