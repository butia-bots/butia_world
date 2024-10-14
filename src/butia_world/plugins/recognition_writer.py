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

import weaviate
# import weaviate.classes as wvc

import ros_numpy
from PIL import Image

import base64
from io import BytesIO

from threading import Event

from std_srvs.srv import Empty

def toBase64(img: Image.Image):
  buffered = BytesIO()
  img.save(buffered, format="JPEG")
  img_str = base64.b64encode(buffered.getvalue())
  return img_str


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

    distance = euclidian_distance(description.bbox.center.position, pose.position)
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
    self.vector_client = weaviate.connect_to_local()
    self.must_remove_trigger = Event()
    self.must_remove_trigger.clear()
    self.must_remove_srv = rospy.Service("/butia_world/trigger_remove", Empty, lambda req: self.must_remove_trigger.set())
    if not self.vector_client.collections.exists("WorldObjects"):
      self.vector_collection = self.vector_client.collections.create(
        name="WorldObjects",
        vectorizer_config=weaviate.Configure.Vectorizer.multi2vec_clip(image_fields=["image"]),
      )
    else:
      self.vector_collection = self.vector_client.collections.get("WorldObjects")

  def run(self):
    self.subscriber = rospy.Subscriber(self.topic, Recognitions3D, self._on_recognition)
    rospy.spin()

  def _must_update(self, description):
    candidate_keys = self.r.keys(pattern=description.label + '*')
    if len(candidate_keys) == 0:
      return None
    
    self.check_function['args'] = (description, candidate_keys, self.r,)
    return self.check_function['function'](*self.check_function['args'], **self.check_function['kwargs'])
    
  
  def _to_link(self, image_header, description, link='map'):
    pose_stamped = PoseStamped()
    pose_stamped.header = image_header
    pose_stamped.pose.position.x = description.bbox.center.position.x
    pose_stamped.pose.position.y = description.bbox.center.position.y
    pose_stamped.pose.position.z = description.bbox.center.position.z
    pose_stamped.pose.orientation.x = description.bbox.center.orientation.x
    pose_stamped.pose.orientation.y = description.bbox.center.orientation.y
    pose_stamped.pose.orientation.z = description.bbox.center.orientation.z
    pose_stamped.pose.orientation.w = description.bbox.center.orientation.w
    
    pose_stamped_map = PoseStamped()
    
    if (image_header.frame_id, link) in self.source_frames:
      self.tfl.waitForTransform(link, image_header.frame_id, rospy.Time(), rospy.Duration(1.0))
      self.source_frames.append((image_header.frame_id, link))
    try:
      pose_stamped_map = self.tfl.transformPose(link, pose_stamped)
    
    except:
      rospy.logerr('Transform does not exist.')
      return None

    #pose_stamped_map = self.transformer.transformPose(link, pose_stamped)

    new_description = description
    new_description.bbox.center.position = pose_stamped_map.pose.position
    new_description.bbox.center.orientation = pose_stamped_map.pose.orientation

    return new_description 

  def _generate_uid(self):
    return str(uuid.uuid4())
  
  def _save_description(self, description, image_rgb):
    d_id = self._must_update(description)
    with self.r.pipeline() as pipe:
      if d_id:
        description_id = d_id
      else:
        description_id = '{label}/{id}'.format(
          label=description.label,
          id=self._generate_uid()
        ).encode('utf-8')
        if image_rgb != None:
          self.vector_collection.data.insert(
            properties={
              "image": toBase64(Image.fromarray(ros_numpy.numpify(image_rgb)[int(description.bbox2D.center.y-description.bbox2D.size_y//2):int(description.bbox2D.center.y+description.bbox2D.size_y//2),int(description.bbox2D.center.x-description.bbox2D.size_x//2):int(description.bbox2D.center.x+description.bbox2D.size_x//2)][:,:,::-1])).decode(),
              "worldKey": description_id.decode()
            },
          )
      print(description_id)
      pipe.hmset(description_id + b'/pose', {
        'px': description.bbox.center.position.x,
        'py': description.bbox.center.position.y,
        'pz': description.bbox.center.position.z,
        'ox': description.bbox.center.orientation.x,
        'oy': description.bbox.center.orientation.y,
        'oz': description.bbox.center.orientation.z,
        'ow': description.bbox.center.orientation.w,
        'sx': description.bbox.size.x,
        'sy': description.bbox.size.y,
        'sz': description.bbox.size.z,
        'stamp': rospy.Time.now().to_sec()
      })
      '''pipe.hmset(description_id + b'/color', {
        'r': description.color.r,
        'g': description.color.g,
        'b': description.color.b,
        'a': description.color.a
      })'''
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
    
  def _remove_unavailable_descriptions(self, image_header, descriptions, distance_threshold=0.1):
    link = image_header.frame_id
    keys = self.r.keys(pattern="*/pose")
    keys = [k for k in keys if not k.startswith(b"target/")]
    keys_not_in_area = set(keys)
    for key in list(keys_not_in_area):
      data = self.r.hgetall(key)
      ps = PoseStamped()
      ps.header.frame_id = "map"
      ps.pose.position.x = data[b'px']
      ps.pose.position.y = data[b'py']
      ps.pose.position.z = data[b'pz']
      ps.pose.orientation.x = data[b'ox']
      ps.pose.orientation.y = data[b'oy']
      ps.pose.orientation.z = data[b'oz']
      ps.pose.orientation.w = data[b'ow']
      stamp = float(data[b'stamp'])
      self.tfl.waitForTransform(image_header.frame_id, ps.header.frame_id, rospy.Time(), rospy.Duration(1.0))
      try:
        ps = self.tfl.transformPose(image_header.frame_id, ps)
      except:
        continue
      if abs(ps.pose.position.x) < 1.0 and abs(ps.pose.position.y) < 1.0 and ps.pose.position.z > 1.5:
        for description in descriptions:
          if euclidian_distance(ps.pose.position, description.bbox.center.position) < distance_threshold or rospy.Time.now() - rospy.Time.from_sec(stamp) < rospy.Duration(5.0):
            if key in keys_not_in_area:
              keys_not_in_area.remove(key)
      else:
        keys_not_in_area.remove(key)
    world_keys_not_in_area = [k.strip(b"/pose") for k in list(keys_not_in_area)]
    for obj in self.vector_collection.query.fetch_objects().objects:
      if obj.properties['worldKey'].encode('utf-8') in world_keys_not_in_area:
        self.vector_collection.data.delete_by_id(obj.uuid)
    with self.r.pipeline() as pipe:
      for key in list(keys_not_in_area):
        pipe.delete(key)
      pipe.execute()

  def _on_recognition(self, recognition):
    image_header = recognition.image_rgb.header
    if self.must_remove_trigger.is_set():
      self._remove_unavailable_descriptions(image_header, recognition.descriptions)
      self.must_remove_trigger.clear()
    for description in recognition.descriptions:
      rospy.loginfo(description)
      if description.bbox.size.x > 0.1 or description.bbox.size.y > 0.1 or description.bbox.size.z > 0.1:
        description = self._to_link(image_header, description, link=self.fixed_frame)
        if description is not None:
          uid = self._save_description(description, image_rgb=recognition.image_rgb)
        #target is not tested yet
        #image_header, description = self._to_link(image_header, description, link='footprint_link')
        #self._save_target(uid, description.pose.pose)
