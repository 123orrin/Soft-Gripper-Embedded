from adafruit_motorkit import MotorKit
import time
import board
import RPi.GPIO as GPIO

# NOTE: Remember to change Motor number and Encoder Pins
ENCODER_PIN_A = 16
ENCODER_PIN_B = 19
kit = MotorKit(i2c=board.I2C())

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ENCODER_PIN_A, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(ENCODER_PIN_B, GPIO.IN, GPIO.PUD_UP)

start = time.time()
kit.motor3.throttle = 0.4
last = None
count = 0

while count < 2000:
    if GPIO.input(ENCODER_PIN_B) != last:
        print(count)
        count += 1
        last = GPIO.input(ENCODER_PIN_B)
        
kit.motor3.throttle = -0.4
while count > 0:
    if GPIO.input(ENCODER_PIN_A) != last:
        print(count)
        count -= 1
        last = GPIO.input(ENCODER_PIN_A)

kit.motor3.throttle = 0
