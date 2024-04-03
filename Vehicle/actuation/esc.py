import time
import board
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(board.SCL, board.SDA)

pca = PCA9685(i2c)

pca.frequency = 50

servo1 = servo.Servo(pca.channels[0])
ESC = servo.Servo(pca.channels[1])

speed = 0.63

mapped_speed = int(speed * 180)

def motor_init():

    for i in range(mapped_speed):
        ESC.angle = i
        time.sleep(0.05)
    ESC.angle = 0

motor_init()

while True:
    servo1.angle = 50
    ESC.angle = mapped_speed
    time.sleep(5)

    servo1.angle = 90
    ESC.angle = mapped_speed
    time.sleep(5)

    servo1.angle = 140
    ESC.angle = mapped_speed
    time.sleep(5)
    

pca.deinit()