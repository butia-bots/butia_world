import time

import rospy
from std_msgs.msg import Int32
from .world_plugin import WorldPlugin

class HeartbeatReaderPlugin(WorldPlugin):

  def run(self):
    publisher = rospy.Publisher('world/heartbeat', Int32)
    message = Int32()
    while True:
      time.sleep(1)
      message.data = int(self.r.get('heartbeat'))
      publisher.publish(message)
