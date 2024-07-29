from adafruit_motorkit import MotorKit
import time
import board

kit = MotorKit(i2c=board.I2C())

i = 0
kit.motor1.throttle = 0
kit.motor2.throttle = 0
kit.motor3.throttle = 0

while i < 3:
    kit.motor1.throttle = 0.3
    kit.motor2.throttle = 0.3
    kit.motor3.throttle = 0.3
    time.sleep(1)
    kit.motor1.throttle = -0.3
    kit.motor2.throttle = -0.3
    kit.motor3.throttle = -0.3
    time.sleep(1)
    i = i + 1

kit.motor1.throttle = 0
kit.motor2.throttle = 0
kit.motor3.throttle = 0