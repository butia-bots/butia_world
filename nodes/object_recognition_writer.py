#!/usr/bin/env python

import rospy
from butia_world.plugins import *

if __name__ == '__main__':
  rospy.init_node('object_recognition_writer')
  check_function = {
                    'function' : check_candidates_by_distance,
                    'args' : (),
                    'kwargs' : {
                        'distance_threshold' : 0.1
                    }
                   }
  plugin = RecognitionWriterPlugin('/butia_vision/or/object_recognition3d', check_function)
  plugin.run()