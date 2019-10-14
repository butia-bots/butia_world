import rospy

from .world_plugin import WorldPlugin

class MapWriterPlugin(WorldPlugin):

  def run(self):
    poses = rospy.get_param('/butia_world/map/targets')
    with self.r.pipeline() as pipe:
      for p_id, pose in poses.items():
        print(pose)
        key = 'target/' + p_id + '/' + 'pose'
        pipe.hmset(key, pose)
      pipe.execute()