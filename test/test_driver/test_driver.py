import sys
from Jetson import GPIO
import time
from threading import Thread
from getch import getch

class Pin:
  left_power = 12
  left_direction = 11
  right_power = 16
  right_direction = 15

GPIO.setmode(GPIO.BOARD)
GPIO.setup(
  [
    Pin.left_power,
    Pin.left_direction,
    Pin.right_power,
    Pin.right_direction
  ],
  GPIO.OUT, initial=GPIO.LOW
)

class Action:
  forward = 'FORWARD'
  backward = 'BACKWARD'
  left = 'LEFT'
  right = 'RIGHT'
  stop = 'STOP'

class Robot:
  def forward(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.left_direction,
        Pin.right_power
      ],
      GPIO.HIGH
    )
    GPIO.output(
      Pin.right_direction,
      GPIO.LOW
    )

  def backward(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power,
        Pin.right_direction
      ],
      GPIO.HIGH
    )
    GPIO.output(
      [
        Pin.left_direction,
      ],
      GPIO.LOW
    )

  def left(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power
      ],
      GPIO.HIGH
    )
    GPIO.output(
      [
        Pin.right_direction,
        Pin.left_direction
      ],
      GPIO.LOW
    )
  
  def right(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power
      ],
      GPIO.HIGH
    )
    GPIO.output(
      [
        Pin.right_direction,
        Pin.left_direction
      ],
      GPIO.HIGH
    )

  def stop(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power,
      ],
      GPIO.LOW
    )


def app():
  print('APP RUNS')
  robot = Robot()
  running = True
  curr_key = ''
  
  while running:
    time.sleep(.001)
    key = getch()

    if key.lower() != curr_key:
      curr_key = key.lower()
      if curr_key == 'w':
        robot.forward()
        print(Action.forward)
      elif curr_key == 'a':
        robot.left()
        print(Action.left)
      elif curr_key == 'd':
        robot.right()
        print(Action.right)
      elif curr_key == 's':
        robot.backward()
        print(Action.backward)
      elif curr_key == 'e':
        robot.stop()
        print(Action.stop)
      else:
        print(f'invalid input: {key}')
        robot.stop()
    
    if key == 'q':
      running = False
      print('Good BYE')
  

if __name__ == '__main__':
  try:
    app()
  except Exception as err:
    raise err
  finally:
    GPIO.cleanup()
