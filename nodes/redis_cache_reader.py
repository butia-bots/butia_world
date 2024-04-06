#!/usr/bin/env python3

import rospy
from butia_world.plugins import RedisCacheReader

if __name__ == '__main__':
  rospy.init_node('redis_cache_reader')
  topic = rospy.get_param('/butia_world/redis_cache/topic', '/butia_vision/r/redis_cache')
  plugin = RedisCacheReader(topic)
  plugin.run()