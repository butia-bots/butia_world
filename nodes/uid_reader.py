#!/usr/bin/env python

import rospy
from butia_world.plugins import UidReaderPlugin

if __name__ == '__main__':
  rospy.init_node('uid_reader')
  plugin = UidReaderPlugin()
  plugin.run()