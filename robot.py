from Jetson import GPIO
import time
from getch import getch

class Pin:
  inp1 = 16
  inp2 = 15
  inp3 = 12
  inp4 = 11

class Action:
  forward = 'FORWARD'
  backward = 'BACKWARD'
  left = 'LEFT'
  right = 'RIGHT'
  stop = 'STOP'

class Robot:
  def __init__(self):
    self.current_action = ''
    self.is_cleaned_up = False
    self._setup_pins()
    self.mode = 'drive'
    self.stop()
    print('Robot Activated')
  
  def _setup_pins(self):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(
      [
        Pin.inp1,
        Pin.inp2,
        Pin.inp3,
        Pin.inp4
      ],
      GPIO.OUT, initial=GPIO.LOW
    )
    print('Robot Setup')

  def _left_forward(self):
    GPIO.output(Pin.inp1, GPIO.LOW)
    GPIO.output(Pin.inp2, GPIO.HIGH)

  def _left_backward(self):
    GPIO.output(Pin.inp1, GPIO.HIGH)
    GPIO.output(Pin.inp2, GPIO.LOW)
  
  def _left_stop(self):
    GPIO.output(Pin.inp1, GPIO.LOW)
    GPIO.output(Pin.inp2, GPIO.LOW)

  def _right_forward(self):
    GPIO.output(Pin.inp3, GPIO.HIGH)
    GPIO.output(Pin.inp4, GPIO.LOW)

  def _right_backward(self):
    GPIO.output(Pin.inp3, GPIO.LOW)
    GPIO.output(Pin.inp4, GPIO.HIGH)
  
  def _right_stop(self):
    GPIO.output(Pin.inp3, GPIO.LOW)
    GPIO.output(Pin.inp4, GPIO.LOW)

  def forward(self):
    self._left_forward()
    self._right_forward()

    self.current_action = Action.forward

  def backward(self):
    self._left_backward()
    self._right_backward()

    self.current_action = Action.backward
  
  def rotate_left(self):
    self._left_backward()
    self._right_forward()

    self.current_action = Action.left
  
  def rotate_right(self):
    self._right_backward()
    self._left_forward()

    self.current_action = Action.right

  def stop(self):
    self._left_stop()
    self._right_stop()
    self.current_action = Action.stop

  def quit(self):
    self.stop()
    if self.is_cleaned_up is not True:
      GPIO.cleanup()
      print('ROBOT Cleanup')
      self.is_cleaned_up = True
    print("Robot Deactivated")

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
