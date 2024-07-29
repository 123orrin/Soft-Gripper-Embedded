from adafruit_motorkit import MotorKit
import time
import board
import RPi.GPIO as GPIO

JOY_X_PIN = 16
JOY_Y_PIN = 25
JOY_BUTTON_PIN = 22
kit = MotorKit(i2c=board.I2C())

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(JOY_X_PIN, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(JOY_Y_PIN, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(JOY_BUTTON_PIN, GPIO.IN, GPIO.PUD_UP)

start = time.time()
kit.motor1.throttle = 0
factor = 1

while time.time() - start < 60:
    if not GPIO.input(JOY_BUTTON_PIN):
        factor = factor * -1
        print('switch direction')
        time.sleep(1)
    if not GPIO.input(JOY_X_PIN):
        kit.motor1.throttle = 0.5 * factor
    else:
        kit.motor1.throttle = 0
        

kit.motor1.throttle = 0
