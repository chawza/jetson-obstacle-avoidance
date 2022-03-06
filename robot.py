import sys
from Jetson import GPIO
import time
from threading import Thread
from getch import getch

class Pin:
  left_power = 16
  left_direction = 15
  right_power = 12
  right_direction = 11

class Action:
  forward = 'FORWARD'
  backward = 'BACKWARD'
  left = 'LEFT'
  right = 'RIGHT'
  stop = 'STOP'

class Robot:
  def __init__(self):
    print('Robot Activated')
    self.current_action = ''
    self.is_cleaned_up = False
    self._setup_pins()
  
  def _setup_pins(self):
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

  def forward(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power,
        Pin.left_direction
      ],
      GPIO.HIGH
    )
    GPIO.output(
      Pin.right_direction,
      GPIO.LOW
    )
    self.current_action = Action.forward

  def backward(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power,
        Pin.right_direction,
      ],
      GPIO.HIGH
    )
    GPIO.output(
      Pin.left_direction,
      GPIO.LOW
    )
    self.current_action = Action.backward
  
  def left(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power,
      ],
      GPIO.HIGH
    )
    GPIO.output(
      [
        Pin.left_direction,
        Pin.right_direction
      ],
      GPIO.LOW
    )
    self.current_action = Action.left
  
  def right(self):
    GPIO.output(
      [
        Pin.left_power,
        Pin.right_power,
      ],
      GPIO.HIGH
    )
    GPIO.output(
      [
        Pin.left_direction,
        Pin.right_direction
      ],
      GPIO.HIGH
    )
    self.current_action = Action.right

  def stop(self):
    print('ROBOT: STOP')
    self._setup_pins()
    self.current_action = Action.stop

  def quit(self):
    self.stop()
    print("Robot Deactivated")
    GPIO.cleanup()
    self.is_cleaned_up = True

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
      elif curr_key == 's':
        robot.backward()
        print(Action.backward)
      elif curr_key == 'e':
        robot.stop()
        print(Action.stop)
      else:
        print(f'invalid input: {key}')
        robot.stop()
    
    if key == 'q' or key == ' ':
      running = False
  
  print('APP STOPS')
  GPIO.cleanup()
