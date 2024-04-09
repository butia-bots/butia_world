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
            #rospy.loginfo(f'Keys found:{count}')
            for key in keys:
                #rospy.loginfo(f'Keys found:{key}')
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
            description = FaceDescription()
            
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = key.decode('utf-8')
            
            description.header = h
            
            # Decode bytes to string before using them as keys
            item_str = {k.decode('utf-8'): v for k, v in item.items()}
            content_dict = eval(item_str['content'])  # Convert string to dictionary
            description.label = content_dict['label']
            description.encoding = content_dict['face_encode']
            for i in range(10):
                if content_dict['label'] == f'test{i}':
                    print(h.frame_id)
            
            redis_cache.descriptions.append(description)
            
        return redis_cache