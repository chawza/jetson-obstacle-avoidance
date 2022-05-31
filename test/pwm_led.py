from Jetson import GPIO
import time
from getch import getch

GPIO.setmode(GPIO.BOARD)

pin_pwm0 = 32
pin_pwm2 = 33

GPIO.setup(pin_pwm0, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(pin_pwm2, GPIO.OUT, initial=GPIO.LOW)

pin_1 = GPIO.PWM(pin_pwm0, 1000)
pin_2 = GPIO.PWM(pin_pwm2, 1000)

percentage = 0

pin_1.start(percentage)
pin_2.start(percentage)

try:
  while True:
    if percentage >= 100:
      break
    percentage += 10
    pin_1.ChangeDutyCycle(percentage)
    pin_2.ChangeDutyCycle(percentage)
    print('Percentage: {}'.format(percentage))
    time.sleep(.5)
finally:
  GPIO.cleanup()
