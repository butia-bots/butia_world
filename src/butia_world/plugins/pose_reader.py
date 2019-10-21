import time

import rospy
from butia_world_msgs.msg import DBPose
from butia_world_msgs.srv import GetPoses, GetPosesResponse
from .world_plugin import WorldPlugin

class PoseReaderPlugin(WorldPlugin):

  def callback(self, req):
    query = req.query
    keys = self.r.keys(query)
    keys = list(filter(lambda x: '/pose' in x, keys))
    db_poses = []
    for key in keys:
      pose = DBPose()
      pose.label = key.replace('/pose', '')
      db_pose = self.r.hgetall(key)
      pose.pose.position.x = float(db_pose['px'])
      pose.pose.position.y = float(db_pose['py'])
      pose.pose.position.z = float(db_pose['pz'])
      pose.pose.orientation.x = float(db_pose['ox'])
      pose.pose.orientation.y = float(db_pose['oy'])
      pose.pose.orientation.z = float(db_pose['oz'])
      pose.pose.orientation.w = float(db_pose['ow'])
      db_poses.append(pose)
    return GetPosesResponse(db_poses)

  def run(self):
    pose_server = rospy.Service('/butia_world/get_poses', GetPoses, self.callback)
    rospy.spin()
