from abc import abstractmethod

import rospy
import redis

class WorldPlugin:

  def __init__(self):
    redis_params = rospy.get_param('/butia_world/redis')
    self.r = redis.Redis(**redis_params)
    self.fixed_frame = rospy.get_param('/butia_world/fixed_frame')

  @abstractmethod
  def run(self):
    raise NotImplementedError