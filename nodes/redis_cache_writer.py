#!/usr/bin/env python3

import rospy
from butia_world.plugins import RedisCacheWriter

if __name__ == '__main__':
  rospy.init_node('redis_cache_writer')
  topic = rospy.get_param('/butia_world/redis_cache/topic', '/butia_vision/r/redis_cache')
  plugin = RedisCacheWriter(topic)
  plugin.run()