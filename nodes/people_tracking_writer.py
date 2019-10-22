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
  fixed_frame = rospy.get_param('/butia_world/fixed_frame', 'map')
  plugin = RecognitionWriterPlugin(topic, check_function)
  plugin.run()