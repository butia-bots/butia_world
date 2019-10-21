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
        link = key.replace('/pose', '')
        db_pose = self.r.hgetall(key)
        pose.position.x = float(db_pose['px'])
        pose.position.y = float(db_pose['py'])
        pose.position.z = float(db_pose['pz'])
        pose.orientation.x = float(db_pose['ox'])
        pose.orientation.y = float(db_pose['oy'])
        pose.orientation.z = float(db_pose['oz'])
        pose.orientation.w = float(db_pose['ow'])

        p = pose.position
        o = pose.orientation
        br.sendTransform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w), rospy.Time.now(), link, 'kinect2_rgb_optical_frame')
      rate.sleep()
