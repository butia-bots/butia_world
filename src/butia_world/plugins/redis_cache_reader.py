import rospy
from butia_vision_msgs.msg import FaceEncoding, FaceDescription
from std_msgs.msg import Header

from .world_plugin import WorldPlugin

class RedisCacheReader(WorldPlugin):

    def __init__(self, topic):
        WorldPlugin.__init__(self)
        self.topic = topic
        self.cache = {}
        
    def run(self):
        self.cache_publisher = rospy.Publisher(self.topic, FaceEncoding, queue_size=10)
        rospy.loginfo('RedisCachePlugin running...')
        rospy.spin()
        
    def _getDataFromRedis(self, pattern = 'faces:*'):
        cursor = 0

        # Scan keys matching the pattern "faces:*"
        while True:
            cursor, keys = self.r.scan(cursor, match=pattern)
            for key in keys:
                hash_value = self.r.hgetall(key)
                self.cache[key] = hash_value
            if cursor == 0:
                break
        return self.encapsulateData()
    
    def encapsulateData(self):
        redis_cache = FaceEncoding()
        h = Header()
        h.stamp = rospy.Time.now()
        redis_cache.header = h
        
        for key, item in self.cache.items():
            description = FaceDescription()
            
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = key
            
            description.header = h
            description.label = self.cache[b'label']
            description.encoding = self.cache[b'face_encode']
            
            redis_cache.descriptions.append(description)
            
        if len(redis_cache.descriptions) > 0:
            self.cache_publisher.publish(redis_cache)
        return