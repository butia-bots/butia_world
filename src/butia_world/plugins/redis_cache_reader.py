import rospy
from butia_vision_msgs.msg import FaceEncoding, FaceDescription
from std_msgs.msg import Header
from butia_world_msgs.srv import RedisCacheReaderSrv

from .world_plugin import WorldPlugin
class RedisCacheReader(WorldPlugin):

    def __init__(self):
        WorldPlugin.__init__(self)
        self.cache = {}
        
    def run(self):
        rospy.Service('redis_cache_reader_srv', RedisCacheReaderSrv, self.getFromRedisCache)
        rospy.loginfo('Cache is being updated')
        #self._getDataFromRedis()
        rospy.spin()
        
    def getFromRedisCache(self, request):
        data = self._getDataFromRedis()
        return data
    
        
    def _getDataFromRedis(self, pattern = 'faces:*'):
        cursor = 0
        # Scan keys matching the pattern "faces:*"
        while True:
            count, keys = self.r.scan(cursor, match=pattern)
            for key in keys:
                hash_value = self.r.hgetall(key)
                self.cache[key] = hash_value
            cursor = count
            if cursor == 0:
                break
        data = self.encapsulateData()
        return data
    
    def encapsulateData(self) -> None:
        redis_cache = FaceEncoding()
        h = Header()
        h.stamp = rospy.Time.now()
        redis_cache.header = h
        
        for key, item in self.cache.items():
            item_str = {k.decode('utf-8'): v for k, v in item.items()}
            content_dict = eval(item_str['content'])  # Convert string to dictionary
            label = content_dict['label']
            encodings = content_dict['face_encode']
            
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = key.decode('utf-8')
            
            for encoding in encodings:
                description = FaceDescription()
                description.header = h
                description.global_id = key.decode('utf-8')
                description.label = label
                description.encoding = encoding
                redis_cache.descriptions.append(description)
            
        return redis_cache
