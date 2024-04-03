import csv
import time
import pygame
import board
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from datetime import datetime
import cv2
import numpy as np
import pyrealsense2 as rs
import os

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)

image_path = '/home/jetson/self_driving/vision5/RGB'
depth_path =  '/home/jetson/self_driving/vision5/Depth'

# Start streaming
pipeline.start(config)

# Setup for PCA9685 and servos
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50
servo1 = servo.Servo(pca.channels[0])
ESC = servo.Servo(pca.channels[1])

max_speed = 0.63

def map_joystick_value(value, min_limit=-1, max_limit=1, map_to_min=0, map_to_max=180):
    value = max(min(value, max_limit), min_limit)
    mapped_value = ((value - min_limit) / (max_limit - min_limit)) * (map_to_max - map_to_min) + map_to_min
    return int(mapped_value)

def save_to_csv(file_path, data):
    with open(file_path, 'w', newline='') as csvfile:
        fieldnames = ['Timestamp', 'JoystickValue', 'MappedAngle']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for row in data:
            writer.writerow(row)

def motor_init():
    init_speed = int(max_speed * 180)
    for i in range(init_speed):
        ESC.angle = i
        time.sleep(0.05)
    ESC.angle = 0

def joy_throttle(value, min_limit=0, max_limit=0.98, map_to_min=97, map_to_max=130):
    value = max(min(value, max_limit), min_limit)
    mapped_value = ((value - min_limit) / (max_limit - min_limit)) * (map_to_max - map_to_min) + map_to_min
    print(mapped_value)
    return int(mapped_value)

def joy_servo(value, min_limit=-0.99, max_limit=0.99, map_to_min=39, map_to_max=150):
    value = max(min(value, max_limit), min_limit)
    mapped_value = ((value - min_limit) / (max_limit - min_limit)) * (map_to_max - map_to_min) + map_to_min
    print(mapped_value)
    return int(mapped_value)

def emergency_stop():
    ESC.angle = 0  # Assuming 0 is the neutral position for your ESC
    print("Emergency Stop Activated!")

def is_controller_connected(joystick):
    try:
        joystick.get_name()
        return True
    except pygame.error:
        return False

def main():
    pygame.init()
    pygame.joystick.init()

    try:
        frame = 0
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print(f"Joystick Name: {joystick.get_name()}")
        print(f"Number of Axes: {joystick.get_numaxes()}")
        print(f"Number of Buttons: {joystick.get_numbuttons()}")

        data = []

        while True:
            pygame.event.pump()

            timestamp = timestamp = datetime.now().strftime('%H:%M:%S')
            joystick_value = joystick.get_axis(0)  # Example for one axis
            mapped_angle = map_joystick_value(joystick_value, map_to_min=0, map_to_max=180)

            if not is_controller_connected(joystick):
                emergency_stop()
                break  # Stop loop if controller is disconnected

            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    if event.axis == 0:
                        axis_value = joystick.get_axis(0)
                        servo1.angle = joy_servo(axis_value)
                        print(f"Axis 0 Value: {axis_value}")
                    elif event.axis == 2:
                        axis_value = joystick.get_axis(2)
                        print(f"Left Trigger Value: {axis_value}")
                    elif event.axis == 4:
                        axis_value = joystick.get_axis(4)
                        ESC.angle = joy_throttle(axis_value)
                        print(f"Right Trigger Value: {axis_value}")
                elif event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                    button = event.button
                    print(f"Button {button} {'pressed' if event.type == pygame.JOYBUTTONDOWN else 'released'}")

            data.append({'Timestamp': timestamp, 'JoystickValue': joystick_value, 'MappedAngle': mapped_angle})
            #time.sleep(0.1)  # Optional delay to reduce data rate
            
             # Wait for a coherent pair of frames: depth and color
        
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            depth_image = np.asanyarray(depth_frame.get_data())
            rgb_image = np.asanyarray(color_frame.get_data())

            # Apply colormap to the depth image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.2), cv2.COLORMAP_JET)

            # Display the depth colormap
            cv2.imshow('RealSense RGB', rgb_image)
            cv2.imshow('RealSense Depth', depth_colormap)

            os.chdir(image_path) 
            cv2.imwrite(f'{frame}.png',rgb_image)
            os.chdir(depth_path) 
            cv2.imwrite(f'{frame}.png',depth_colormap)

            frame+=1
            # Break the loop when 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Stop streaming
        emergency_stop()
        pipeline.stop()
        cv2.destroyAllWindows()
        save_to_csv('joystick_data1.csv', data)
        pygame.quit()
        print("Data saved and program exited.")

if __name__ == "__main__":
    main()
