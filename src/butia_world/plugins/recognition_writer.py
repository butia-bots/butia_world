import uuid
import rospy
import tf
import redis
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from butia_vision_msgs.msg import Recognitions3D, Description3D

from math import sqrt

from .world_plugin import WorldPlugin


def euclidian_distance(p1, p2):
  sums = (p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2
  return sqrt(sums)

def check_candidates_by_distance(description, candidate_keys, r, distance_threshold = 0.1):
  pose_keys = list(filter(lambda x: '/pose' in x, candidate_keys))
  for key in pose_keys:
    db_pose = r.hgetall(key)
    pose = Pose()
    pose.position.x = float(db_pose['px'])
    pose.position.y = float(db_pose['py'])
    pose.position.z = float(db_pose['pz'])
    pose.orientation.x = float(db_pose['ox'])
    pose.orientation.y = float(db_pose['oy'])
    pose.orientation.z = float(db_pose['oz'])
    pose.orientation.w = float(db_pose['ow'])

    distance = euclidian_distance(description.pose.pose.position, pose.position)
    if distance < distance_threshold:
      n_key = key.replace('/pose', '')
      return n_key
  
  return None

def check_candidates_by_label(description, candidate_keys, r):
  label_keys = list(filter(lambda x: description.label_class in x, candidate_keys))
  if len(label_keys) >= 1:
    d_id = label_keys[0].split('/')[-2]
    return description.label_class + '/' + d_id
  else:
    return None

class RecognitionWriterPlugin(WorldPlugin):

  def __init__(self, topic, check_function):
    WorldPlugin.__init__(self)
    self.topic = topic
    self.check_function = check_function

  def run(self):
    self.subscriber = rospy.Subscriber(self.topic, Recognitions3D, self._on_recognition)
    rospy.spin()

  def _must_update(self, image_header, description):
    candidate_keys = self.r.keys(pattern=description.label_class + '*')
    if len(candidate_keys) == 0:
      return None
    
    self.check_function['args'] = (description, candidate_keys, self.r,)
    return self.check_function['function'](*self.check_function['args'], **self.check_function['kwargs'])
    
  
  def _to_map(self, image_header, description):
    pose_stamped = PoseStamped()
    pose_stamped.header = image_header
    pose_stamped.pose.position.x = description.pose.pose.position.x
    pose_stamped.pose.position.y = description.pose.pose.position.y
    pose_stamped.pose.position.z = description.pose.pose.position.z
    pose_stamped.pose.orientation.x = description.pose.pose.orientation.x
    pose_stamped.pose.orientation.y = description.pose.pose.orientation.y
    pose_stamped.pose.orientation.z = description.pose.pose.orientation.z
    pose_stamped.pose.orientation.w = description.pose.pose.orientation.w
    
    pose_stamped_map = PoseStamped()
    t = tf.TransformerROS()
    pose_stamped_map = t.transformPose('map', pose_stamped)

    new_header = pose_stamped_map.header
    new_description = description
    new_description.pose = pose_stamped_map.pose

    return new_header, new_description 

  def _generate_uid(self):
    return str(uuid.uuid4())
  
  def _save_description(self, image_header, description):
    d_id = self._must_update(image_header, description)

    with self.r.pipeline() as pipe:
      if d_id:
        description_id = d_id
      else:
        description_id = '{label}/{id}'.format(
          label=description.label_class,
          id=self._generate_uid()
        )
      pipe.hmset(description_id + '/pose', {
        'px': description.pose.pose.position.x,
        'py': description.pose.pose.position.y,
        'pz': description.pose.pose.position.z,
        'ox': description.pose.pose.orientation.x,
        'oy': description.pose.pose.orientation.y,
        'oz': description.pose.pose.orientation.z,
        'ow': description.pose.pose.orientation.w
      })
      pipe.hmset(description_id + '/color', {
        'r': description.color.r,
        'g': description.color.g,
        'b': description.color.b,
        'a': description.color.a
      })
      pipe.execute()

  def _on_recognition(self, recognition):
    image_header = recognition.image_header
    for description in recognition.descriptions:
      self._save_description(image_header, description)