#!/usr/bin/env python

import rospy
from butia_world.plugins import PoseReaderPlugin

if __name__ == '__main__':
  rospy.init_node('pose_reader')
  plugin = PoseReaderPlugin()
  plugin.run()