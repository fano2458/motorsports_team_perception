# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from zed_interfaces/ObjectsStamped.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg
import zed_interfaces.msg

class ObjectsStamped(genpy.Message):
  _md5sum = "b138979dc4c884a00915d2a416aa75c6"
  _type = "zed_interfaces/ObjectsStamped"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """# Standard Header
std_msgs/Header header

# Array of `object_stamped` topics
zed_interfaces/Object[] objects

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: zed_interfaces/Object
# Object label
string label

# Object label ID
int16 label_id

# Object instance ID
int16 instance_id

# Object sub
string sublabel

# Object confidence level (1-99)
float32 confidence

# Object centroid position
float32[3] position

# Position covariance
float32[6] position_covariance

# Object velocity
float32[3] velocity

# Tracking available
bool tracking_available

# Tracking state
# 0 -> OFF (object not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)
int8 tracking_state

# Action state
# 0 -> IDLE
# 2 -> MOVING
int8 action_state

# 2D Bounding box projected to Camera image
zed_interfaces/BoundingBox2Di bounding_box_2d

# 3D Bounding box in world frame
zed_interfaces/BoundingBox3D bounding_box_3d

# 3D dimensions (width, height, lenght)
float32[3] dimensions_3d

# Is skeleton available?
bool skeleton_available

# 2D Bounding box projected to Camera image of the person head
zed_interfaces/BoundingBox2Df head_bounding_box_2d

# 3D Bounding box in world frame of the person head
zed_interfaces/BoundingBox3D head_bounding_box_3d

# 3D position of the centroid of the person head
float32[3] head_position

# 2D Person skeleton projected to Camera image
zed_interfaces/Skeleton2D skeleton_2d

# 3D Person skeleton in world frame
zed_interfaces/Skeleton3D skeleton_3d

================================================================================
MSG: zed_interfaces/BoundingBox2Di
#      0 ------- 1
#      |         |
#      |         |
#      |         |
#      3 ------- 2
zed_interfaces/Keypoint2Di[4] corners

================================================================================
MSG: zed_interfaces/Keypoint2Di
uint32[2] kp

================================================================================
MSG: zed_interfaces/BoundingBox3D
#      1 ------- 2
#     /.        /|
#    0 ------- 3 |
#    | .       | |
#    | 5.......| 6
#    |.        |/
#    4 ------- 7
zed_interfaces/Keypoint3D[8] corners

================================================================================
MSG: zed_interfaces/Keypoint3D
float32[3] kp

================================================================================
MSG: zed_interfaces/BoundingBox2Df
#      0 ------- 1
#      |         |
#      |         |
#      |         |
#      3 ------- 2
zed_interfaces/Keypoint2Df[4] corners

================================================================================
MSG: zed_interfaces/Keypoint2Df
float32[2] kp

================================================================================
MSG: zed_interfaces/Skeleton2D
# Skeleton joints indices
#        16-14   15-17
#             \ /
#              0
#              |
#       2------1------5
#       |    |   |    |
#	    |    |   |    |
#       3    |   |    6
#       |    |   |    |
#       |    |   |    |
#       4    8   11   7
#            |   |
#            |   |
#            |   |
#            9   12
#            |   |
#            |   |
#            |   |
#           10   13
zed_interfaces/Keypoint2Df[18] keypoints

================================================================================
MSG: zed_interfaces/Skeleton3D
# Skeleton joints indices
#        16-14   15-17
#             \ /
#              0
#              |
#       2------1------5
#       |    |   |    |
#	    |    |   |    |
#       3    |   |    6
#       |    |   |    |
#       |    |   |    |
#       4    8   11   7
#            |   |
#            |   |
#            |   |
#            9   12
#            |   |
#            |   |
#            |   |
#           10   13
zed_interfaces/Keypoint3D[18] keypoints
"""
  __slots__ = ['header','objects']
  _slot_types = ['std_msgs/Header','zed_interfaces/Object[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,objects

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ObjectsStamped, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.objects is None:
        self.objects = []
    else:
      self.header = std_msgs.msg.Header()
      self.objects = []

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.objects:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2h().pack(_x.label_id, _x.instance_id))
        _x = val1.sublabel
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.confidence
        buff.write(_get_struct_f().pack(_x))
        buff.write(_get_struct_3f().pack(*val1.position))
        buff.write(_get_struct_6f().pack(*val1.position_covariance))
        buff.write(_get_struct_3f().pack(*val1.velocity))
        _x = val1
        buff.write(_get_struct_B2b().pack(_x.tracking_available, _x.tracking_state, _x.action_state))
        _v1 = val1.bounding_box_2d
        if len(_v1.corners) != 4:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (4, len(_v1.corners), '_v1.corners')))
        for val3 in _v1.corners:
          buff.write(_get_struct_2I().pack(*val3.kp))
        _v2 = val1.bounding_box_3d
        if len(_v2.corners) != 8:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (8, len(_v2.corners), '_v2.corners')))
        for val3 in _v2.corners:
          buff.write(_get_struct_3f().pack(*val3.kp))
        buff.write(_get_struct_3f().pack(*val1.dimensions_3d))
        _x = val1.skeleton_available
        buff.write(_get_struct_B().pack(_x))
        _v3 = val1.head_bounding_box_2d
        if len(_v3.corners) != 4:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (4, len(_v3.corners), '_v3.corners')))
        for val3 in _v3.corners:
          buff.write(_get_struct_2f().pack(*val3.kp))
        _v4 = val1.head_bounding_box_3d
        if len(_v4.corners) != 8:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (8, len(_v4.corners), '_v4.corners')))
        for val3 in _v4.corners:
          buff.write(_get_struct_3f().pack(*val3.kp))
        buff.write(_get_struct_3f().pack(*val1.head_position))
        _v5 = val1.skeleton_2d
        if len(_v5.keypoints) != 18:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (18, len(_v5.keypoints), '_v5.keypoints')))
        for val3 in _v5.keypoints:
          buff.write(_get_struct_2f().pack(*val3.kp))
        _v6 = val1.skeleton_3d
        if len(_v6.keypoints) != 18:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (18, len(_v6.keypoints), '_v6.keypoints')))
        for val3 in _v6.keypoints:
          buff.write(_get_struct_3f().pack(*val3.kp))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.objects is None:
        self.objects = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.objects = []
      for i in range(0, length):
        val1 = zed_interfaces.msg.Object()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 4
        (_x.label_id, _x.instance_id,) = _get_struct_2h().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.sublabel = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.sublabel = str[start:end]
        start = end
        end += 4
        (val1.confidence,) = _get_struct_f().unpack(str[start:end])
        start = end
        end += 12
        val1.position = _get_struct_3f().unpack(str[start:end])
        start = end
        end += 24
        val1.position_covariance = _get_struct_6f().unpack(str[start:end])
        start = end
        end += 12
        val1.velocity = _get_struct_3f().unpack(str[start:end])
        _x = val1
        start = end
        end += 3
        (_x.tracking_available, _x.tracking_state, _x.action_state,) = _get_struct_B2b().unpack(str[start:end])
        val1.tracking_available = bool(val1.tracking_available)
        _v7 = val1.bounding_box_2d
        _v7.corners = []
        for i in range(0, 4):
          val3 = zed_interfaces.msg.Keypoint2Di()
          start = end
          end += 8
          val3.kp = _get_struct_2I().unpack(str[start:end])
          _v7.corners.append(val3)
        _v8 = val1.bounding_box_3d
        _v8.corners = []
        for i in range(0, 8):
          val3 = zed_interfaces.msg.Keypoint3D()
          start = end
          end += 12
          val3.kp = _get_struct_3f().unpack(str[start:end])
          _v8.corners.append(val3)
        start = end
        end += 12
        val1.dimensions_3d = _get_struct_3f().unpack(str[start:end])
        start = end
        end += 1
        (val1.skeleton_available,) = _get_struct_B().unpack(str[start:end])
        val1.skeleton_available = bool(val1.skeleton_available)
        _v9 = val1.head_bounding_box_2d
        _v9.corners = []
        for i in range(0, 4):
          val3 = zed_interfaces.msg.Keypoint2Df()
          start = end
          end += 8
          val3.kp = _get_struct_2f().unpack(str[start:end])
          _v9.corners.append(val3)
        _v10 = val1.head_bounding_box_3d
        _v10.corners = []
        for i in range(0, 8):
          val3 = zed_interfaces.msg.Keypoint3D()
          start = end
          end += 12
          val3.kp = _get_struct_3f().unpack(str[start:end])
          _v10.corners.append(val3)
        start = end
        end += 12
        val1.head_position = _get_struct_3f().unpack(str[start:end])
        _v11 = val1.skeleton_2d
        _v11.keypoints = []
        for i in range(0, 18):
          val3 = zed_interfaces.msg.Keypoint2Df()
          start = end
          end += 8
          val3.kp = _get_struct_2f().unpack(str[start:end])
          _v11.keypoints.append(val3)
        _v12 = val1.skeleton_3d
        _v12.keypoints = []
        for i in range(0, 18):
          val3 = zed_interfaces.msg.Keypoint3D()
          start = end
          end += 12
          val3.kp = _get_struct_3f().unpack(str[start:end])
          _v12.keypoints.append(val3)
        self.objects.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.objects)
      buff.write(_struct_I.pack(length))
      for val1 in self.objects:
        _x = val1.label
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1
        buff.write(_get_struct_2h().pack(_x.label_id, _x.instance_id))
        _x = val1.sublabel
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
        _x = val1.confidence
        buff.write(_get_struct_f().pack(_x))
        buff.write(val1.position.tostring())
        buff.write(val1.position_covariance.tostring())
        buff.write(val1.velocity.tostring())
        _x = val1
        buff.write(_get_struct_B2b().pack(_x.tracking_available, _x.tracking_state, _x.action_state))
        _v13 = val1.bounding_box_2d
        if len(_v13.corners) != 4:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (4, len(_v13.corners), '_v13.corners')))
        for val3 in _v13.corners:
          buff.write(val3.kp.tostring())
        _v14 = val1.bounding_box_3d
        if len(_v14.corners) != 8:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (8, len(_v14.corners), '_v14.corners')))
        for val3 in _v14.corners:
          buff.write(val3.kp.tostring())
        buff.write(val1.dimensions_3d.tostring())
        _x = val1.skeleton_available
        buff.write(_get_struct_B().pack(_x))
        _v15 = val1.head_bounding_box_2d
        if len(_v15.corners) != 4:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (4, len(_v15.corners), '_v15.corners')))
        for val3 in _v15.corners:
          buff.write(val3.kp.tostring())
        _v16 = val1.head_bounding_box_3d
        if len(_v16.corners) != 8:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (8, len(_v16.corners), '_v16.corners')))
        for val3 in _v16.corners:
          buff.write(val3.kp.tostring())
        buff.write(val1.head_position.tostring())
        _v17 = val1.skeleton_2d
        if len(_v17.keypoints) != 18:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (18, len(_v17.keypoints), '_v17.keypoints')))
        for val3 in _v17.keypoints:
          buff.write(val3.kp.tostring())
        _v18 = val1.skeleton_3d
        if len(_v18.keypoints) != 18:
          self._check_types(ValueError("Expecting %s items but found %s when writing '%s'" % (18, len(_v18.keypoints), '_v18.keypoints')))
        for val3 in _v18.keypoints:
          buff.write(val3.kp.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.objects is None:
        self.objects = None
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.objects = []
      for i in range(0, length):
        val1 = zed_interfaces.msg.Object()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.label = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.label = str[start:end]
        _x = val1
        start = end
        end += 4
        (_x.label_id, _x.instance_id,) = _get_struct_2h().unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.sublabel = str[start:end].decode('utf-8', 'rosmsg')
        else:
          val1.sublabel = str[start:end]
        start = end
        end += 4
        (val1.confidence,) = _get_struct_f().unpack(str[start:end])
        start = end
        end += 12
        val1.position = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        start = end
        end += 24
        val1.position_covariance = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=6)
        start = end
        end += 12
        val1.velocity = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        _x = val1
        start = end
        end += 3
        (_x.tracking_available, _x.tracking_state, _x.action_state,) = _get_struct_B2b().unpack(str[start:end])
        val1.tracking_available = bool(val1.tracking_available)
        _v19 = val1.bounding_box_2d
        _v19.corners = []
        for i in range(0, 4):
          val3 = zed_interfaces.msg.Keypoint2Di()
          start = end
          end += 8
          val3.kp = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=2)
          _v19.corners.append(val3)
        _v20 = val1.bounding_box_3d
        _v20.corners = []
        for i in range(0, 8):
          val3 = zed_interfaces.msg.Keypoint3D()
          start = end
          end += 12
          val3.kp = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
          _v20.corners.append(val3)
        start = end
        end += 12
        val1.dimensions_3d = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        start = end
        end += 1
        (val1.skeleton_available,) = _get_struct_B().unpack(str[start:end])
        val1.skeleton_available = bool(val1.skeleton_available)
        _v21 = val1.head_bounding_box_2d
        _v21.corners = []
        for i in range(0, 4):
          val3 = zed_interfaces.msg.Keypoint2Df()
          start = end
          end += 8
          val3.kp = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
          _v21.corners.append(val3)
        _v22 = val1.head_bounding_box_3d
        _v22.corners = []
        for i in range(0, 8):
          val3 = zed_interfaces.msg.Keypoint3D()
          start = end
          end += 12
          val3.kp = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
          _v22.corners.append(val3)
        start = end
        end += 12
        val1.head_position = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
        _v23 = val1.skeleton_2d
        _v23.keypoints = []
        for i in range(0, 18):
          val3 = zed_interfaces.msg.Keypoint2Df()
          start = end
          end += 8
          val3.kp = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=2)
          _v23.keypoints.append(val3)
        _v24 = val1.skeleton_3d
        _v24.keypoints = []
        for i in range(0, 18):
          val3 = zed_interfaces.msg.Keypoint3D()
          start = end
          end += 12
          val3.kp = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
          _v24.keypoints.append(val3)
        self.objects.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2I = None
def _get_struct_2I():
    global _struct_2I
    if _struct_2I is None:
        _struct_2I = struct.Struct("<2I")
    return _struct_2I
_struct_2f = None
def _get_struct_2f():
    global _struct_2f
    if _struct_2f is None:
        _struct_2f = struct.Struct("<2f")
    return _struct_2f
_struct_2h = None
def _get_struct_2h():
    global _struct_2h
    if _struct_2h is None:
        _struct_2h = struct.Struct("<2h")
    return _struct_2h
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
_struct_B2b = None
def _get_struct_B2b():
    global _struct_B2b
    if _struct_B2b is None:
        _struct_B2b = struct.Struct("<B2b")
    return _struct_B2b
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
