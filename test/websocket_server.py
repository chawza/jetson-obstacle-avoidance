import asyncio
import websockets
from websockets.server import WebSocketServerProtocol
from websockets.exceptions import ConnectionClosed

async def ws_handler(websocket: WebSocketServerProtocol):
  print('handler start')
  try:
    while True:
      message = await websocket.recv()
      print(message)
      print(f'host {websocket.host}\tport: {websocket.port}\t\tmessage: {message}')

  except Exception as err:
    print(err)

  finally:
    await websocket.close()
    print('handler close')


async def main():
  async with websockets.serve(ws_handler, '192.168.100.10', 8765):
    await asyncio.Future()


if __name__ == '__main__':
  print('Server start')
  asyncio.run(main())