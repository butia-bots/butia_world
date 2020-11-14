import rospy
import tf
from geometry_msgs.msg import Pose

from .world_plugin import WorldPlugin

class ViewerReaderPlugin(WorldPlugin): 

  def run(self):
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
      pose_keys = self.r.keys(b'*/pose')
      pose = Pose()
      for key in pose_keys:
        print(key)
        link = key.replace(b'/pose', b'')
        db_pose = self.r.hgetall(key)
        pose.position.x = float(db_pose[b'px'])
        pose.position.y = float(db_pose[b'py'])
        pose.position.z = float(db_pose[b'pz'])
        pose.orientation.x = float(db_pose[b'ox'])
        pose.orientation.y = float(db_pose[b'oy'])
        pose.orientation.z = float(db_pose[b'oz'])
        pose.orientation.w = float(db_pose[b'ow'])

        p = pose.position
        o = pose.orientation
        br.sendTransform((p.x, p.y, p.z), (o.x, o.y, o.z, o.w), rospy.Time.now(), link.decode('utf-8'), self.fixed_frame)
      rate.sleep()
