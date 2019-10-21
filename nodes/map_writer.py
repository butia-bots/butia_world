#!/usr/bin/env python

import rospy
from butia_world.plugins import MapWriterPlugin

if __name__ == '__main__':
  rospy.init_node('map_writer')
  plugin = MapWriterPlugin()
  plugin.run()