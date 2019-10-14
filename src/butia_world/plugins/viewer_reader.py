import rospy
import tf
from geometry_msgs.msg import Pose

from .world_plugin import WorldPlugin

class ViewerReaderPlugin(WorldPlugin): 

  def run(self):
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
      pose_keys = self.r.keys('*/pose')
      pose = Pose()
      for key in pose_keys:
        link = key.split('/')[-2]
        db_pose = self.r.hvals(key)
        pose.position.x = float(db_pose[0])
        pose.position.y = float(db_pose[1])
        pose.position.z = float(db_pose[2])
        pose.orientation.x = float(db_pose[3])
        pose.orientation.y = float(db_pose[4])
        pose.orientation.z = float(db_pose[5])
        pose.orientation.w = float(db_pose[6])
        #br.sendTransform(pose.position, pose.orientation, rospy.Time.now(), link, "map")

        print(db_pose)
      rate.sleep()
