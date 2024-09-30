import rospy

import tf

from .world_plugin import WorldPlugin
from butia_world_msgs.srv import GetKey, GetKeyResponse, GetPose, GetPoseResponse

from geometry_msgs.msg import Pose, PoseStamped, Vector3

from math import sqrt

import weaviate
import weaviate.classes as wvc

class PosePlugin(WorldPlugin):
  def readPose(self, key):
    pose = Pose()
    rospy.loginfo(self.r.keys())
    db_pose = self.r.hgetall(key)
    rospy.loginfo(key)
    rospy.loginfo(db_pose)
    pose.position.x = float(db_pose[b'px'])
    pose.position.y = float(db_pose[b'py'])
    pose.position.z = float(db_pose[b'pz'])
    pose.orientation.x = float(db_pose[b'ox'])
    pose.orientation.y = float(db_pose[b'oy'])
    pose.orientation.z = float(db_pose[b'oz'])
    pose.orientation.w = float(db_pose[b'ow'])  
    
    return pose

  def readSize(self, key):
    size = Vector3()
    db_size = self.r.hgetall(key)
    try:
      size.x = float(db_size[b'sx'])
      size.y = float(db_size[b'sy'])
      size.z = float(db_size[b'sz'])
    except Exception:
      pass

    return size

  def setStaticPose(self):
    poses = rospy.get_param('/butia_world/pose/targets', {})
    with self.r.pipeline() as pipe:
      for p_id, pose in poses.items():
        print(pose)
        key = 'target/' + p_id + '/' + 'pose'
        pipe.hmset(key, pose)
      pipe.execute()
  
  def getClosestKey(self, req):
    query = req.query
    max_size = req.max_size
    keys = self.r.keys(query)
    if len(keys) == 0:
      results = self.vector_collection.query.near_text(query=query, certainty=req.threshold)
      rospy.loginfo(results.objects)
      keys = [f'{obj.properties["worldKey"]}/pose' for obj in results.objects]
    rospy.loginfo(keys)
    keys = list(filter(lambda x: '/pose' in x and 'target' not in x, keys))
    rospy.loginfo(keys)
    min_distance = float('inf')
    min_key = None
    for key in keys:
      if max_size > 0.0:
        size = self.readSize(key)
        if size.x > max_size or size.y > max_size or size.z > max_size:
          continue
      pose = PoseStamped()
      pose.header.frame_id = self.fixed_frame
      pose.header.stamp = rospy.Time.now()

      try:
        pose.pose = self.readPose(key)
      except:
        continue

      t = tf.TransformerROS()
      p = t.transformPose('/map', pose)

      distance = sqrt(p.pose.position.x**2 + p.pose.position.y**2 + p.pose.position.z**2)
      if distance < min_distance:
        min_distance = distance
        min_key = key
    
    if min_key == None:
      return GetKeyResponse(success=False)
    min_key = min_key.replace('/pose', '')
    rospy.loginfo("Key: " + min_key)

    return GetKeyResponse(key=min_key, success=True)

  def getPose(self, req):
    key = req.key
    res = GetPoseResponse()
    pose = self.readPose(key)
    res.pose = pose
    rospy.loginfo(pose)
    size = self.readSize(key)
    res.size = size
    rospy.loginfo(size)
    return res

  def run(self):
    self.vector_client = weaviate.connect_to_local()
    self.vector_collection = self.vector_client.collections.get("WorldObjects")
    self.setStaticPose()
    self.closest_key_server = rospy.Service('/butia_world/get_closest_key', GetKey, self.getClosestKey)
    self.pose_server = rospy.Service('/butia_world/get_pose', GetPose, self.getPose)
    rospy.spin()
