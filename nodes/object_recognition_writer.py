#!/usr/bin/env python3

import rospy
from butia_world.plugins import *

if __name__ == '__main__':
  rospy.init_node('object_recognition_writer')
  distance_threshold = rospy.get_param('/butia_world/object_recognition/distance_threshold', 0.2)
  check_function = {
                    'function' : check_candidates_by_distance,
                    'args' : (),
                    'kwargs' : {
                        'distance_threshold' : distance_threshold
                    }
                   }
  topic = rospy.get_param('/butia_world/object_recognition/topic', '/butia_vision/br/object_recognition3d')
  plugin = RecognitionWriterPlugin(topic, check_function)
  plugin.run()