from Jetson import GPIO
import time
from getch import getch

'''
W : Forward / Speed up
S : Backward / Speed dowm
A : turn left
D : turn right
Q : rotate left
E : rotate right
'''

class Pin:
  inp1 = 16
  inp2 = 15
  inp3 = 12
  inp4 = 11
  ena = 32
  enb = 33

class Action:
  forward = 'FORWARD'
  backward = 'BACKWARD'
  left = 'LEFT'
  right = 'RIGHT'
  stop = 'STOP'
  rotate_left = 'ROTATE LEFT'
  rotate_right = 'ROTATE RIGHT'

class Robot:
  def __init__(self):
    # GPIO Pin
    self._setup_pins()

    # Robot variables
    self.state = ''
    self.is_cleaned_up = False
    self.mode = 'drive'
    self.speed = 0  # 0...255
    self.speed_step = 25
    
    # Robot initiation
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
    self.left_pwm = GPIO.PWM(Pin.ena, 500)
    self.right_pwm = GPIO.PWM(Pin.enb, 500)
    self.left_pwm.start(25)
    self.right_pwm.start(25)
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
    self.speed += self.speed_step
    if self.speed > 100:
      self.speed = 100
    set_speed = self.speed if self.speed >= 0 else -self.speed
    self.left_pwm.ChangeDutyCycle(set_speed)
    self.right_pwm.ChangeDutyCycle(set_speed)

    if self.speed >= 0:
      self._left_forward() 
      self._right_forward()
      self.state = Action.forward
    else:
      self._left_backward()
      self._right_backward()
      self.state = Action.backward

  def backward(self):
    self.speed -= self.speed_step
    if self.speed < -100:
      self.speed = -100
    
    set_speed = self.speed if self.speed >= 0 else -self.speed
    self.left_pwm.ChangeDutyCycle(set_speed)
    self.right_pwm.ChangeDutyCycle(set_speed)

    if self.speed < 0:
      self._left_backward()
      self._right_backward()
      self.state = Action.backward
    else:
      self._left_forward()
      self._right_forward()
      self.state = Action.forward

  def right(self):
    
    self.left_pwm.ChangeDutyCycle(50)
    self.right_pwm.ChangeDutyCycle(50)

    self._left_backward()
    self._right_forward()

    self.state = Action.left

  def rotate_left(self):
    self.left_pwm.ChangeDutyCycle(75)
    self.right_pwm.ChangeDutyCycle(75)

    self._left_backward()
    self._right_forward()

    self.state = Action.rotate_left
  
  def rotate_right(self):
    self.left_pwm.ChangeDutyCycle(75)
    self.right_pwm.ChangeDutyCycle(75)

    self._right_backward()
    self._left_forward()

    self.state = Action.rotate_right

  def stop(self):
    self._left_stop()
    self._right_stop()
    self.speed = 0
    self.state = Action.stop

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
