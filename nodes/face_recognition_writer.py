#!/usr/bin/env python

import rospy
from butia_world.plugins import *

if __name__ == '__main__':
  rospy.init_node('face_recognition_writer')
  check_function = {
                    'function' : check_candidates_by_label,
                    'args' : (),
                    'kwargs' : {
                    }
                   }
  plugin = RecognitionWriterPlugin('/butia_vision/fr/face_recognition3d', check_function)
  plugin.run()