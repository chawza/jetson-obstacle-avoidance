from dotenv import load_dotenv
load_dotenv()

import os
from multiprocessing import Process, Event
import asyncio

from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from websockets.server import WebSocketServerProtocol, serve
import SharedArray as sa

class Broadcast(Process):
  def __init__(self, ws_handler: WebSocketServerProtocol, host, port, stop_event_name):
    super(Broadcast, self).__init__()
    self.host = host
    self.port = port
    self.ws_handler = ws_handler
    self.stop_event_name = stop_event_name

  async def start_server(self):
    async with serve(
      self.ws_handler,
      self.host,
      self.port
    ):
      print('Braodcast: serving in {} {}'.format(self.host, self.port))
      await self.stop_event()

  def run(self):
    asyncio.run(self.start_server())
  
  async def stop_event(self,):
    """
    an event loop that watch a value in a memory. it waits the value to be True
    """
    event = sa.attach(self.stop_event_name)
    while True:
      if event[0] == True:
        break
      await asyncio.sleep(1)
    return
