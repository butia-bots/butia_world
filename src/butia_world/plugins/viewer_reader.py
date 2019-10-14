import rospy
from .world_plugin import WorldPlugin

class ViewerReaderPlugin(WorldPlugin):

  def run(self):
    #br = tf.TransformBroadcaster()
    pos_keys = self.r.keys('*/position')
    ori_keys = self.r.keys('*/orientation')
    print(keys)
    #while True:
    #  time.sleep(1)
    #  message.data = int(self.r.get('heartbeat'))
    #  publisher.publish(message)
