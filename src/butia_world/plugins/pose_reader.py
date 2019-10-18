import time

import rospy
from .world_plugin import WorldPlugin

class PoseReaderPlugin(WorldPlugin):

  def run(self):
    publisher = rospy.Publisher('world/heartbeat', Int32)
    message = Int32()
    while True:
      time.sleep(1)
      message.data = int(self.r.get('heartbeat'))
      publisher.publish(message)
