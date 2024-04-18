import rospy
import asyncio
from aioredis import Redis
from aioredis import BlockingConnectionPool, Redis
from redis.commands.search.indexDefinition import IndexDefinition, IndexType


class WorldPluginAsync:
    def __init__(self, host="localhost", port=6379, decode_responses=True) -> None:
        self._local_cache = {}
        self.__pool = BlockingConnectionPool(host=host, port=port, decode_responses=decode_responses)
    
    def __await__(self):
        return self.init().__await__()
    
    async def init(self):
        self._pool = await Redis(connection_pool=self.__pool)
        await self.getData()
        asyncio.create_task(self._listen_invalidate())
        return self
    
    async def _listen_invalidate(self) -> None:
        pubsub = self._pool.pubsub()
        await pubsub.execute_command(b"CLIENT", b"ID")
        client_id = await pubsub.connection.read_response()
        await pubsub.execute_command(
             f"CLIENT TRACKING on REDIRECT {client_id} BCAST"
        )
        await pubsub.connection.read_response()
        await pubsub.subscribe("__redis__:invalidate")

        while True:
            message = await pubsub.get_message(ignore_subscribe_messages=True, timeout=0.1)
            print('Message is: ' + str(message))
            if message is None or not message.get("data"):
                continue
            key = message["data"][0]
            if key in self._local_cache:
                print(f"Invalidating key {key}")
                del self._local_cache[key]
                
    async def set(self, key, value) -> None:
        self._local_cache[key] = value
        await self._pool.set(key, value)
    
    async def setPipeline(self, key, pattern, value) -> None:
        pipe = self._pool.pipeline()
        self._local_cache[key] = value
        pipe.set(key, pattern, value)
        await self.pipeline.execute()
        
    async def get(self, key):
        if key in self._local_cache:
            return self._local_cache[key]
        value = await self._pool.json().get(key)
        if value is not None:
            self._local_cache[key] = value
        return value
    
    async def getData(self, pattern) -> None:
        keys = await self._pool.keys(str(pattern))
        for key in keys:
            self.get(key)
        return
    
    def _createIndex(self, index: str, prefix: str, index_type: IndexType, fields: tuple) -> None:
        definition = IndexDefinition(prefix=prefix, index_type=index_type)
        self._pool.ft(index).create_index(fields, definition)
    
    def _getCachedKeys(self):
        return sorted(self._local_cache.keys())