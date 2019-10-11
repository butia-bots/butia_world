from abc import abstractmethod

import rospy
import redis

class WorldPlugin:

  def __init__(self):
    redis_params = rospy.get_param('redis')
    self.r = redis.Redis(**redis_params)

  @abstractmethod
  def run(self):
    raise NotImplementedError