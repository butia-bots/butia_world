import time

import rospy
from butia_world_msgs.srv import GetUids, GetUidsResponse
from .world_plugin import WorldPlugin

class UidReaderPlugin(WorldPlugin):

  def callback(self, req):
    query = req.query
    keys = self.r.keys(query)
    uids = []
    for key in keys:
      uids.append(key.split('/')[-2])
    return GetUidsResponse(uids)

  def run(self):
    pose_server = rospy.Service('/butia_world/get_uids', GetUids, self.callback)
    rospy.spin()
