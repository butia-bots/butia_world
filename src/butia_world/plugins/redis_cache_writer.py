import rospy
from butia_vision_msgs.msg import FaceEncoding

from .world_plugin import WorldPlugin, RedisCacheReader

class RedisCacheWriter(WorldPlugin):

    def __init__(self, topic):
        WorldPlugin.__init__(self)
        self.topic = topic
        self.cache = {}

    def run(self):
        self.subscriber = rospy.Subscriber(self.topic, FaceEncoding, self._on_recognition)
        rospy.loginfo('RedisCachePlugin running...')
        rospy.spin()
    
    def _getDataFromRedis(self, pattern = 'faces:*'):
        cursor = 0
        keys_count = 0

        # Scan keys matching the pattern "faces:*"
        while True:
            cursor, keys = self.r.scan(cursor, match=pattern)
            keys_count += len(keys)
            for key in keys:
                hash_value = self.r.hgetall(key)
                self.cache[key] = hash_value
            if cursor == 0:
                break
        return keys_count

    def _sendToCache(self):
        reader = RedisCacheReader(self.topic)
        reader.run()
    
    def _toCompose(self, data):
        composed = {}
        for description in data.descriptions:
            print(description.label)
            composed['label'] = description.label
            composed['face_encoding'] = description.encoding
        return self._pushToRedis(composed)
    
    def _pushToRedis(self, data):
        new_key = self._get_last_key() + 1
        print(new_key)
        id = 'faces:{last_key}'.format(last_key=new_key).encode('utf-8')
        with self.r.pipeline() as pipe:
            pipe.hmset(id, data)
            pipe.execute()
        return self._sendToCache()
  
    def _on_recognition(self, recognition):
        return self._toCompose(recognition)