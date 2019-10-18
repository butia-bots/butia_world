#!/usr/bin/env python

import rospy
from butia_world.plugins import *

if __name__ == '__main__':
  rospy.init_node('people_tracking_writer')
  check_function = {
                    'function' : check_candidates_by_label,
                    'args' : (),
                    'kwargs' : {
                    }
                   }
                     
  topic = rospy.get_param('/butia_world/people_tracking/topic', '/butia_vision/pt/people_tracking3d')
  to_map = rospy.get_param('/butia_world/people_tracking/to_map', True)
  plugin = RecognitionWriterPlugin(topic, check_function, to_map)
  plugin.run()