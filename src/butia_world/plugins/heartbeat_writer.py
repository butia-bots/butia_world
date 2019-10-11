import time

from .world_plugin import WorldPlugin

class HeartbeatWriterPlugin(WorldPlugin):

  def run(self):
    self.r.set('heartbeat', 0)
    while True:
      self.r.incr('heartbeat')
      time.sleep(1)
    