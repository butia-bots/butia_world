import uuid
import rospy
import tf
import redis
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from butia_vision_msgs.msg import Recognitions3D, Description3D

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from math import sqrt, atan2, cos, sin, pi

from .world_plugin import WorldPlugin


def euclidian_distance(p1, p2):
  sums = (p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2
  return sqrt(sums)

def check_candidates_by_distance(description, candidate_keys, r, distance_threshold = 0.1):
  pose_keys = list(filter(lambda x: b'/pose' in x, candidate_keys))
  for key in pose_keys:
    db_pose = r.hgetall(key)
    pose = Pose()
    pose.position.x = float(db_pose[b'px'])
    pose.position.y = float(db_pose[b'py'])
    pose.position.z = float(db_pose[b'pz'])
    pose.orientation.x = float(db_pose[b'ox'])
    pose.orientation.y = float(db_pose[b'oy'])
    pose.orientation.z = float(db_pose[b'oz'])
    pose.orientation.w = float(db_pose[b'ow'])

    distance = euclidian_distance(description.pose.pose.position, pose.position)
    print(distance)
    if distance < distance_threshold:
      n_key = key.replace(b'/pose', b'')
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

  def __init__(self, topic, check_function, approach_distance = 1):
    WorldPlugin.__init__(self)
    self.topic = topic
    self.check_function = check_function
    self.approach_distance = approach_distance
    self.source_frames = []
    self.tfl = tf.TransformListener()


  def run(self):
    self.subscriber = rospy.Subscriber(self.topic, Recognitions3D, self._on_recognition)
    rospy.spin()

  def _must_update(self, description):
    candidate_keys = self.r.keys(pattern=description.label_class + '*')
    if len(candidate_keys) == 0:
      return None
    
    self.check_function['args'] = (description, candidate_keys, self.r,)
    return self.check_function['function'](*self.check_function['args'], **self.check_function['kwargs'])
    
  
  def _to_link(self, image_header, description, link='map'):
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
    
    if (image_header.frame_id, link) in self.source_frames:
      self.tfl.waitForTransform(image_header.frame_id, link, rospy.Time(), rospy.Duration(1.0))
      self.source_frames.append((image_header.frame_id, link))
    try:
      pose_stamped_map = self.tfl.transformPose(link, pose_stamped)
    
    except:
      rospy.logerr('Transform does not exist.')
      return None

    #pose_stamped_map = self.transformer.transformPose(link, pose_stamped)

    new_description = description
    new_description.pose = pose_stamped_map

    return new_description 

  def _generate_uid(self):
    return str(uuid.uuid4())
  
  def _save_description(self, description):
    d_id = self._must_update(description)

    with self.r.pipeline() as pipe:
      if d_id:
        description_id = d_id
      else:
        description_id = '{label}/{id}'.format(
          label=description.label_class,
          id=self._generate_uid()
        ).encode('utf-8')
      print(description_id)
      pipe.hmset(description_id + b'/pose', {
        'px': description.pose.pose.position.x,
        'py': description.pose.pose.position.y,
        'pz': description.pose.pose.position.z,
        'ox': description.pose.pose.orientation.x,
        'oy': description.pose.pose.orientation.y,
        'oz': description.pose.pose.orientation.z,
        'ow': description.pose.pose.orientation.w
      })
      pipe.hmset(description_id + b'/color', {
        'r': description.color.r,
        'g': description.color.g,
        'b': description.color.b,
        'a': description.color.a
      })
      pipe.execute()
    return d_id

  def _save_target(self, uid, pose):
    dx = pose.position.x
    dy = pose.position.y
    angle = atan2(dy, dx)
    distance = sqrt(dx**2 + dy**2)
    distance -= self.approach_distance

    nx = distance*cos(angle)
    ny = distance*sin(angle)
    nz = 0

    orientation = pose.orientation
    orientation_l = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_l)
    yaw = yaw - pi

    orientation_l = quaternion_from_euler(roll, pitch, yaw)

    npose = pose
    npose.position.x = nx
    npose.position.y = ny
    npose.position.z = nz
    npose.orientation.x = orientation_l[0]
    npose.orientation.y = orientation_l[1]
    npose.orientation.z = orientation_l[2]
    npose.orientation.w = orientation_l[3]

    key = 'target/' + str(uid) + '/pose'
    self.r.hmset(key, {
        'px': npose.position.x,
        'py': npose.position.y,
        'pz': npose.position.z,
        'ox': npose.orientation.x,
        'oy': npose.orientation.y,
        'oz': npose.orientation.z,
        'ow': npose.orientation.w
      })

  def _on_recognition(self, recognition):
    image_header = recognition.image_header
    for description in recognition.descriptions:
      description = self._to_link(image_header, description, link=self.fixed_frame)
      if description is not None:
        uid = self._save_description(description)
      #target is not tested yet
      #image_header, description = self._to_link(image_header, description, link='footprint_link')
      #self._save_target(uid, description.pose.pose)