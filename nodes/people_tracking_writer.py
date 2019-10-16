#!/usr/bin/env python

import rospy
from butia_world.plugins import *

if __name__ == '__main__':
  rospy.init_node('people_tracking_writer')
  check_function = {
                    'function' : check_candidates_by_label,
                    'args' : (),
                    'kwargs' : {
                        'distance_threshold' : 0.1
                    }
                   }
  plugin = RecognitionWriterPlugin('/butia_vision/pt/people_tracking3d', check_function)
  plugin.run()