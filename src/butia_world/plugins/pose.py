import rospy

import tf

from .world_plugin import WorldPlugin
from butia_world_msgs.srv import GetKey, GetKeyResponse, GetPose, GetPoseResponse

from geometry_msgs.msg import Pose, PoseStamped, Vector3

from math import sqrt

import chromadb
from chromadb.utils.embedding_functions import OpenCLIPEmbeddingFunction

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
      size.x = float(db_size['sx'])
      size.y = float(db_size['sy'])
      size.z = float(db_size['sz'])
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
    keys = self.r.keys(query)
    if len(keys) == 0:
      results = self.chroma_collection.query(query_texts=[query,])
      rospy.loginfo(results)
      keys = [f'{results["ids"][i]}/pose' for i in range(len(results['ids'])) if results["distances"][i][0] < 1.0 - req.threshold]
    rospy.loginfo(keys)
    keys = list(filter(lambda x: '/pose' in x and 'target' not in x, keys))
    rospy.loginfo(keys)
    min_distance = float('inf')
    min_key = None
    for key in keys:
      pose = PoseStamped()
      pose.header.frame_id = self.fixed_frame
      pose.header.stamp = rospy.Time.now()

      pose.pose = self.readPose(key)

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
    self.embedding_function = OpenCLIPEmbeddingFunction()
    self.chroma_client = chromadb.PersistentClient()
    self.chroma_collection = self.chroma_client.get_or_create_collection(name="world-objects", embedding_function=self.embedding_function)
    self.setStaticPose()
    self.closest_key_server = rospy.Service('/butia_world/get_closest_key', GetKey, self.getClosestKey)
    self.pose_server = rospy.Service('/butia_world/get_pose', GetPose, self.getPose)
    rospy.spin()
