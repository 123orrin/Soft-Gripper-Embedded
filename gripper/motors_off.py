from adafruit_motorkit import MotorKit
import time
import board
import RPi.GPIO as GPIO
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
i2c = busio.I2C(board.SCL, board.SDA)
kit = MotorKit(i2c=i2c)
kit.motor1.throttle = 0.
kit.motor2.throttle = 0.
kit.motor3.throttle = 0.
kit.motor4.throttle = 0.
