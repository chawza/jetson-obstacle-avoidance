from getch import getch
from robot import Robot, Action
import time

bot = Robot()

while True:
  key = getch()
  
  if key == 'w':
    bot.forward()
  elif key == 's':
    bot.backward()
  elif key == 'x':
    bot.stop()
  
  elif key == 'q':
    bot.rotate_left()
  elif key == 'e':
    bot.rotate_right()
  elif key == 'z':
    break
  print('state: {} speed: {}'.format(bot.state, bot.speed))

bot.quit()