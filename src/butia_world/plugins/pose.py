import rospy

import tf

from .world_plugin import WorldPlugin
from butia_world_msgs.srv import GetKey, GetKeyResponse, GetPose, GetPoseResponse

from geometry_msgs.msg import Pose, PoseStamped

from math import sqrt

class PosePlugin(WorldPlugin):
  def readPose(self, key):
    pose = Pose()
    db_pose = self.r.hgetall(key)
    pose.position.x = float(db_pose['px'])
    pose.position.y = float(db_pose['py'])
    pose.position.z = float(db_pose['pz'])
    pose.orientation.x = float(db_pose['ox'])
    pose.orientation.y = float(db_pose['oy'])
    pose.orientation.z = float(db_pose['oz'])
    pose.orientation.w = float(db_pose['ow'])  
    
    return pose

  def setStaticPose(self):
    poses = rospy.get_param('/butia_world/pose/targets')
    with self.r.pipeline() as pipe:
      for p_id, pose in poses.items():
        print(pose)
        key = 'target/' + p_id + '/' + 'pose'
        pipe.hmset(key, pose)
      pipe.execute()
  
  def getClosestKey(self, req):
    query = req.query
    keys = self.r.keys(query)
    keys = list(filter(lambda x: '/pose' in x, keys))

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

    return GetKeyResponse(min_key)

  def getPose(self, req):
    key = req.key
    pose = self.readPose(key)

    return pose

  def run(self):
    self.setStaticPose()
    self.closest_key_server = rospy.Service('/butia_world/get_closest_key', GetKey, self.getClosestKey)
    self.pose_server = rospy.Service('/butia_world/get_pose', GetPose, self.getPose)
    rospy.spin()
