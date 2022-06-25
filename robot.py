try:
  from Jetson import GPIO
  from getch import getch
except ImportError:
  pass

'''
W : Forward / Speed up
S : Backward / Speed dowm
A : turn left
D : turn right
Q : rotate left
E : rotate right
'''

class Pin:
  inp1 = 22
  inp2 = 21
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
    self.speed_step = 20
    self.turn_dir = 0 # when n < 0 : turn left when n > 0 : turn right
    self.max_turn = 10
    
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
      GPIO.OUT,
      initial=GPIO.LOW
    )
    self.left_pwm = GPIO.PWM(Pin.ena, 500)
    self.right_pwm = GPIO.PWM(Pin.enb, 500)
    self.left_pwm.start(0)
    self.right_pwm.start(0)
    print('Robot Setup')

  def _left_forward(self):
    GPIO.output(Pin.inp3, GPIO.LOW)
    GPIO.output(Pin.inp4, GPIO.HIGH)

  def _left_backward(self):
    GPIO.output(Pin.inp3, GPIO.HIGH)
    GPIO.output(Pin.inp4, GPIO.LOW)
  
  def _left_stop(self):
    GPIO.output(Pin.inp3, GPIO.HIGH)
    GPIO.output(Pin.inp4, GPIO.HIGH)

  def _right_forward(self):
    GPIO.output(Pin.inp1, GPIO.HIGH)
    GPIO.output(Pin.inp2, GPIO.LOW)

  def _right_backward(self):
    GPIO.output(Pin.inp1, GPIO.LOW)
    GPIO.output(Pin.inp2, GPIO.HIGH)
  
  def _right_stop(self):
    GPIO.output(Pin.inp1, GPIO.HIGH)
    GPIO.output(Pin.inp2, GPIO.HIGH)
  
  def forward(self, set_speed=None):
    if set_speed == None:
      #if device is turning, forward will go straight
      if self.turn_dir != 0:
        self.turn_dir = 0
        return

      self.speed += self.speed_step
      if self.speed > 100:
        self.speed = 100
      set_speed = abs(self.speed)
    self.left_pwm.ChangeDutyCycle(set_speed - (set_speed  * .3))
    self.right_pwm.ChangeDutyCycle(set_speed)

    if self.speed >= 0:
      self._left_forward() 
      self._right_forward()
      self.state = Action.forward
    else:
      self._left_backward()
      self._right_backward()
      self.state = Action.backward

  def backward(self, set_speed=None):
    if set_speed == None:
      #if device is turning, robot stops
      if self.turn_dir != 0:
        self.turn_dir = 0
        self.stop()
        return

      self.speed -= self.speed_step
      if self.speed < -100:
        self.speed = -100
      set_speed = abs(self.speed)

    self.left_pwm.ChangeDutyCycle(set_speed - (set_speed  * .3))
    self.right_pwm.ChangeDutyCycle(set_speed)

    if self.speed < 0:
      self._left_backward()
      self._right_backward()
      self.state = Action.backward
    else:
      self._left_forward()
      self._right_forward()
      self.state = Action.forward

  def right(self, set_turn_dir=None):
    if set_turn_dir is None:
      if self.turn_dir == self.max_turn:
        return
      if self.speed == 0:
        return

      if self.turn_dir >= 0:
        self.turn_dir += self.turn_step
      else:
        self.turn_dir = 0

    else:
      self.turn_dir = set_turn_dir

    turn_difference = round(self.speed * (self.turn_dir / self.max_turn))

    left_speed = self.speed - turn_difference
    right_pwm = self.speed + turn_difference
    print(f'LEFT: {left_speed}\tRIGHT: {right_pwm}')
    self.left_pwm.ChangeDutyCycle(left_speed)
    self.right_pwm.ChangeDutyCycle(right_pwm)

    self.state = Action.right

    self._left_forward()
    self._right_forward()

    self.state = Action.right

  def left(self, set_turn_dir=None):
    if set_turn_dir is None:
      if self.turn_dir == -self.max_turn:
        return
      if self.speed == 0:
        return
      
      if self.turn_dir <= 0:
        self.turn_dir -= self.turn_step
      else:
        self.turn_dir = 0

    else:
      self.turn_dir = set_turn_dir

    print('TURN DIR ', self.turn_dir)

    turn_difference = round(self.speed *  (abs(self.turn_dir) / self.max_turn))
    
    left_speed = self.speed + turn_difference
    right_pwm = self.speed - turn_difference
    print(f'LEFT: {left_speed}\tRIGHT: {right_pwm}')
    self.left_pwm.ChangeDutyCycle(left_speed)
    self.right_pwm.ChangeDutyCycle(right_pwm)

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
    print("Robot Quit")

  def print_debug(self):    
    print('state: {} speed: {} turn dir: {}'.format(self.state, self.speed, self.turn_dir))
    
def app():
  print('Robot Activated')
  bot = Robot()
  while True:
    try:
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
      bot.print_debug()
    except Exception as err:
      print(err)

  bot.quit()
  print('Robot Deactivated')

if __name__ == '__main__':
  import sys

  if '--drive' in sys.argv or '-d' in sys.argv:
    app()
