import asyncio
import websockets
import time

async def main():
  async with websockets.connect('ws://192.168.100.10:8765') as websocket:
    print(f'Client websocket connection is connected to {websocket.host} in port {websocket.port}')
    while True:
      message = 'HELLO ' + websocket.host
      print(message)
      await websocket.send(message)
      time.sleep(1)

if __name__ == '__main__':
  print('Client Starting')
  loop = asyncio.get_event_loop()
  loop.run_until_complete(main())