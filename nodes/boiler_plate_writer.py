#!/usr/bin/env python3

import rospy
from butia_world.plugins import HeartbeatWriterPlugin

if __name__ == '__main__':
  rospy.init_node('boiler_plate_writer')
  plugin = HeartbeatWriterPlugin()
  plugin.run()