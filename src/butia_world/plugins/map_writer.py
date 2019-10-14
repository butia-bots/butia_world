import rospy

from .world_plugin import WorldPlugin

class MapWriterPlugin(WorldPlugin):

  def run(self):
    points = rospy.get_param('/butia_world/map/points')
    with self.r.pipeline() as pipe:
      for p_id, point in points.items():
        for v_id, value in point.items():
          key = p_id + '/' + v_id
          pipe.delete(key)
          pipe.rpush(key, *value)
      pipe.execute()
    #self.r.bgsave()