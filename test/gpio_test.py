import Jetson.GPIO as GPIO
import time

def blinking_led_light():
  pin = 21
  pin2 = 22

  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(pin2, GPIO.OUT, initial=GPIO.LOW)

  for i in range(1,10 + 1):
    print(f'second: {i}')
    GPIO.output(pin, GPIO.HIGH)
    GPIO.output(pin2, GPIO.HIGH)
    time.sleep(.5)
    GPIO.output(pin, GPIO.LOW)
    GPIO.output(pin2, GPIO.LOW)
    time.sleep(.5)

if __name__ == '__main__':
  # GPIO.JETSON_INFO
  # GPIO.VERSION

  GPIO.setmode(GPIO.BOARD)
  try:
    blinking_led_light()
  finally:
    GPIO.cleanup()
