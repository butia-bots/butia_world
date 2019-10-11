#!/usr/bin/env python3

import rospy
from butia_world.plugins import HeartbeatReaderPlugin

if __name__ == '__main__':
  rospy.init_node('boiler_plate_reader')
  plugin = HeartbeatReaderPlugin()
  plugin.run()