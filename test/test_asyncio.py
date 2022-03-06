from asynchat import async_chat
import asyncio
from time import perf_counter

server = None

async def createServer():
  await asyncio.sleep(1)
  return 'SERVER'

async def TaskOne():
  print('Task 1: Activated')
  await asyncio.sleep(3)
  print('Task 1: Deactivatad')
  return 'Task 1 Done'

async def TaskTwo():
  print('Task 2: Activated')
  await asyncio.sleep(5)
  print('Task 2: Deactivated')
  return 'Task 2 Done'

async def main():
  global server
  print('program starts')
  server = await createServer()
  print('Server created')
  result = await asyncio.gather(TaskOne(), TaskTwo())
  print(result)

# asyncio.run(main())

async def do_something(event):
  print('sleeping')
  await asyncio.sleep(3)
  event.set()
  print('wakes up')
  return None

async def stuff_that_waits(event):
  print('stop waiting')
  await event.wait()
  print('start stuff')


async def main2():
  stop_server = asyncio.Event()
  print('start')
  result = await asyncio.gather(do_something(stop_server), stuff_that_waits(stop_server))
  print(result)
  print('done')

asyncio.run(main2())

