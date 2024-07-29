from adafruit_motorkit import MotorKit
import time
import board
import RPi.GPIO as GPIO

ENCODER_PIN_A = 4
ENCODER_PIN_B = 17
kit = MotorKit(i2c=board.I2C())

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(ENCODER_PIN_A, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(ENCODER_PIN_B, GPIO.IN, GPIO.PUD_UP)

kit.motor1.throttle = 0.4
kit.motor2.throttle = 0
kit.motor3.throttle = 0
last_A = None
last_B = None
count = 0

while count < 1125:
    val_B = GPIO.input(ENCODER_PIN_B)
    val_A = GPIO.input(ENCODER_PIN_A)
    
    if val_B != last_B:
        count += 1
        last_B = val_B
    elif val_A != last_A:
        count += 1
        last_A = val_A
    print(count)
        

kit.motor1.throttle = 0
kit.motor2.throttle = 0
kit.motor3.throttle = 0
