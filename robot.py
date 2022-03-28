from Jetson import GPIO
import time
from cv2 import RETR_CCOMP
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
    self.turn_dir = 0 # when n < 0 : turn left when n > 0 : turn right
    self.max_turn = 3
    
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
    #if device is turning, forward will go straight
    if self.turn_dir != 0:
      self.turn_dir = 0
      return

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
    #if device is turning, robot stops
    if self.turn_dir != 0:
      self.turn_dir = 0
      self.stop()
      return

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
    if self.turn_dir == self.max_turn:
      return
    if self.speed == 0:
      return
    
    if self.turn_dir < 0:
      self.turn_dir = 0

    self.turn_dir += 1
    turn_difference = int(self.speed * (self.turn_dir / self.max_turn))

    self.left_pwm.ChangeDutyCycle(self.speed - turn_difference)
    self.right_pwm.ChangeDutyCycle(self.speed + turn_difference)

    self.state = Action.right

  def left(self):
    if self.turn_dir == -self.max_turn:
      return
    if self.speed == 0:
      return
    
    if self.turn_dir > 0:
      self.turn_dir = 0

    self.turn_dir += -1
    turn_difference = int(self.speed * ( (-self.turn_dir) / self.max_turn))
    
    self.left_pwm.ChangeDutyCycle(self.speed + turn_difference)
    self.right_pwm.ChangeDutyCycle(self.speed - turn_difference)

    self.state = Action.left

    self._left_forward()
    self._right_forward()

  def rotate_left(self):
    self.left_pwm.ChangeDutyCycle(100)
    self.right_pwm.ChangeDutyCycle(100)

    self._left_backward()
    self._right_forward()

    self.state = Action.rotate_left
  
  def rotate_right(self):
    self.left_pwm.ChangeDutyCycle(100)
    self.right_pwm.ChangeDutyCycle(100)

    self._right_backward()
    self._left_forward()

    self.state = Action.rotate_right

  def stop(self):
    self._left_stop()
    self._right_stop()
    self.speed = 0
    self.turn_dir = 0
    self.state = Action.stop

  def quit(self):
    self.stop()
    if self.is_cleaned_up is not True:
      GPIO.cleanup()
      print('ROBOT Cleanup')
      self.is_cleaned_up = True
    print("Robot Deactivated")

def app():
  print('Robot Activated')
  bot = Robot()
  while True:
    key = getch()
    
    if key == 'w':
      bot.forward()
    elif key == 's':
      bot.backward()
    elif key == 'x':
      bot.stop()
    elif key == 'd':
      bot.right()
    elif key == 'a':
      bot.left()
    
    elif key == 'q':
      bot.rotate_left()
    elif key == 'e':
      bot.rotate_right()
    elif key == 'z':
      break
    print('state: {} speed: {}'.format(bot.state, bot.speed))

  bot.quit()
  print('Robot Deactivated')

if __name__ == '__main__':
  import sys

  if '--drive' in sys.argv or '-d' in sys.argv:
    app()
