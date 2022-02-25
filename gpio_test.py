import Jetson.GPIO as GPIO
import time

def blinking_led_light():
  pin = 7

  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

  for i in range(1,10 + 1):
    print(f'second: {i}')
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(1)


if __name__ == '__main__':
  # GPIO.JETSON_INFO
  # GPIO.VERSION

  GPIO.setmode(GPIO.BOARD)
  try:
    blinking_led_light()
  finally:
    GPIO.cleanup()
