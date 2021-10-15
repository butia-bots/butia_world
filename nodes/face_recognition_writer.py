#!/usr/bin/env python3

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
  topic = rospy.get_param('/butia_world/face_recognition/topic', '/butia_vision/fr/face_recognition3d')
  plugin = RecognitionWriterPlugin(topic, check_function)
  plugin.run()