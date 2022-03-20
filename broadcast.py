from dotenv import load_dotenv
load_dotenv()

import os
from multiprocessing import Process, Event
import asyncio

from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from websockets.server import WebSocketServerProtocol, serve

class Broadcast(Process):
  def __init__(self, ws_handler: WebSocketServerProtocol, host, port):
    super(Broadcast, self).__init__()
    self.host = host
    self.port = port
    self.ws_handler = ws_handler

  async def start_server(self):
    curr_lopp = asyncio.get_event_loop()
    self.cam_stop = asyncio.Event(loop=curr_lopp)

    async with serve(
      self.ws_handler,
      self.host,
      self.port
    ):
      print('Braodcast server starts')
      await self.cam_stop.wait()
      print('NONE')

  def run(self):
    asyncio.run(self.start_server())
