#!/usr/bin/env python3

import rospy
from butia_world.plugins import RedisCacheReader

if __name__ == '__main__':
  rospy.init_node('redis_cache_reader_test')
  rospy.loginfo('Starting redis cache reader')
  plugin = RedisCacheReader()
  plugin.run()