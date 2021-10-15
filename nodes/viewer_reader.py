#!/usr/bin/env python3

import rospy
from butia_world.plugins import ViewerReaderPlugin

if __name__ == '__main__':
  rospy.init_node('viewer_reader')
  plugin = ViewerReaderPlugin()
  plugin.run()