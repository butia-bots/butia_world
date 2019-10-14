import uuid
import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from butia_vision_msgs.msg import Recognitions3D, Description3D

from .world_plugin import WorldPlugin

class DetectionWriterPlugin(WorldPlugin):

  def __init__(self, topic, distance_treshold):
    super().__init__()
    self.subscriber = rospy.Subscriber(topic, Recognitions3D, self._on_recognition)
  
  def run(self):
    pass

  def _must_update(self, image_header: Header, description: Description3D):
    pose_stamped = PoseStamped()
    pose_stamped.header = image_header
    pose_stamped.pose.position.x = description.pose.pose.position.x
    pose_stamped.pose.position.y = description.pose.pose.position.y
    pose_stamped.pose.position.z = description.pose.pose.position.z
    pose_stamped.pose.orientation.x = description.pose.pose.orientation.x
    pose_stamped.pose.orientation.y = description.pose.pose.orientation.y
    pose_stamped.pose.orientation.z = description.pose.pose.orientation.z
    pose_stamped.pose.orientation.w = description.pose.pose.orientation.w
    t = tf.TransformerROS()
    candidates_keys = self.r.keys(pattern=description.label_class + '*')

  def _generate_uid(self):
    return str(uuid.uuid4())
  
  def _save_description(self, description: Description3D):
    with self.r.pipeline() as pipe:
      description_id = description.label_class + self._generate_uid()
      pipe.hmset(description_id + '/position', {
        'x': description.pose.pose.position.x,
        'y': description.pose.pose.position.y,
        'z': description.pose.pose.position.z
      })
      pipe.hmset(description_id + '/orientation', {
        'x': description.pose.pose.orientation.x,
        'y': description.pose.pose.orientation.y,
        'z': description.pose.pose.orientation.z,
        'w': description.pose.pose.orientation.w
      })
      pipe.hmset(description_id + '/color', {
        'r': description.color.r,
        'g': description.color.g,
        'b': description.color.b,
        'a': description.color.a
      })
      pipe.execute()

  def _on_recognition(self, recognition: Recognitions3D):
    for description in recognition.descriptions:
      self._save_description(description)