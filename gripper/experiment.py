from adafruit_motorkit import MotorKit
import time
import board
import RPi.GPIO as GPIO
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Set Encoder Pins
MOTOR1_ENCODER_PIN_A = 4
MOTOR1_ENCODER_PIN_B = 17
MOTOR2_ENCODER_PIN_A = 12
MOTOR2_ENCODER_PIN_B = 13
MOTOR3_ENCODER_PIN_A = 16
MOTOR3_ENCODER_PIN_B = 19

# Set Joystick Pins
JOY_BUTTON_PIN = 18

# Create the I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create Motor Object
kit = MotorKit(i2c=i2c)

# Create the ADC object 
ads = ADS.ADS1115(i2c, gain=2/3, address=0x48)
# Create single-ended input on channel 0
chan0 = AnalogIn(ads, ADS.P0)
chan1 = AnalogIn(ads, ADS.P1)

# Set all Pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(MOTOR1_ENCODER_PIN_A, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(MOTOR1_ENCODER_PIN_B, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(MOTOR2_ENCODER_PIN_A, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(MOTOR2_ENCODER_PIN_B, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(MOTOR3_ENCODER_PIN_A, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(MOTOR3_ENCODER_PIN_B, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(JOY_BUTTON_PIN, GPIO.IN, GPIO.PUD_UP)

# Program variables
start = time.time()
loop_timeout = 3

kit.motor1.throttle = 0
kit.motor2.throttle = 0
kit.motor3.throttle = 0

count = 0
motor1_encoder_count = 0
motor2_encoder_count = 0
motor3_encoder_count = 0


# Amount to pull in encoder counts
# 400 counts is one full turn of the main shaft
desired = 400
motor1_encoder_desired = 0
motor1_encoder_desired = 0
motor1_encoder_desired = 0

motor1_encoderA_last = None
motor1_encoderB_last = None
motor2_encoderA_last = None
motor2_encoderB_last = None
motor3_encoderA_last = None
motor3_encoderB_last = None

while True:
    chan0_val = chan0.value
    chan1_val = chan1.value
        
    control_start = time.time()
    time_in_loop = 0
    while count < desired:
        kit.motor1.throttle = -0.30
        kit.motor2.throttle = -0.30
        kit.motor3.throttle = -0.30

        motor1_encoderA_val = GPIO.input(MOTOR1_ENCODER_PIN_A)
        motor1_encoderB_val = GPIO.input(MOTOR1_ENCODER_PIN_B)   
        if motor1_encoderA_val != motor1_encoderA_last:
            count += 1
            motor1_encoderA_last = motor1_encoderA_val
        if motor1_encoderB_val != motor1_encoderB_last:
            count += 1
            motor1_encoderB_last = motor1_encoderB_val

    time.sleep(5)
                        
    while count > 0:
        kit.motor1.throttle = 0.30
        kit.motor2.throttle = 0.30
        kit.motor3.throttle = 0.30
        motor1_encoderA_val = GPIO.input(MOTOR1_ENCODER_PIN_A)
        motor1_encoderB_val = GPIO.input(MOTOR1_ENCODER_PIN_B)   
        if motor1_encoderA_val != motor1_encoderA_last:
            count -= 1
            motor1_encoderA_last = motor1_encoderA_val
        if motor1_encoderB_val != motor1_encoderB_last:
            count -= 1
            motor1_encoderB_last = motor1_encoderB_val

                
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0
    kit.motor3.throttle = 0
    

