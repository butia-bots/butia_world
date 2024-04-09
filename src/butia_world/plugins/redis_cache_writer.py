import rospy
import uuid
from butia_vision_msgs.msg import FaceEncoding, FaceDescription
from std_msgs.msg import Header
from butia_world_msgs.srv import RedisCacheWriterSrv, RedisCacheReaderSrv

from .world_plugin import WorldPlugin

class RedisCacheWriter(WorldPlugin):

    def __init__(self):
        WorldPlugin.__init__(self)
        self.cache = {}

    def run(self):
        rospy.Service('redis_cache_writer_srv', RedisCacheWriterSrv, self._on_recognition)
        rospy.spin()
        
    def _generate_uid(self):
        return str(uuid.uuid4())
    
    def _toCompose(self, request):
        composed = {}
        data = request.description
        for item in data.descriptions:
            composed['label'] = item.label
            composed['face_encode'] = item.encoding
        return composed
    
    def _pushToRedis(self, data):
        description_id = '{label}:{id}'.format(
                label='faces',
                id=self._generate_uid()
            ).encode('utf-8')
        try:
            self.r.hset(description_id, 'content', str(data))
        except Exception as e:
            rospy.logerr(e)
        return 
  
    def _on_recognition(self, request):
        try:
            composed = self._toCompose(request)
            self._pushToRedis(composed)
            rospy.loginfo('Data pushed to Redis')
            return True
        except Exception as e:
            rospy.logerr(e)
            return False