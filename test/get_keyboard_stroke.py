import sys
import time
from threading import Thread
if sys.platform == 'linux':
  from getch import getch
else:
  from msvcrt import getch

key = ''

def watch_keyboard():
  global key
  print('Enter W,S,E,Q')
  while key.lower() != 'q':
    time.sleep(.001)
    key = getch()

def main_loop():
  global key
  current_key = None
  print('main application is running')
  while key.lower() != 'q':
    time.sleep(.001)
    if current_key != key:
      current_key = key
      print(f'key: {current_key}')

t1 = Thread(target=watch_keyboard)
t2 = Thread(target=main_loop)

if __name__ == '__main__':
  print("App is UP")
  t1.start()
  t2.start()

  t1.join()
  t2.join()
  print('Good Bye')