import pygame
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

max_speed = 0.63


def motor_init():
    init_speed = int(max_speed * 180)
    for i in range(init_speed):
        ESC.angle = i
        time.sleep(0.05)
    ESC.angle = 0


def joy_throttle(value, min_limit=0, max_limit=0.98, map_to_min=97, map_to_max=130):
    value = max(min(value, max_limit), min_limit)
    mapped_value = ((value - min_limit) / (max_limit - min_limit)) * (map_to_max - map_to_min) + map_to_min
    #print(mapped_value)
    return int(mapped_value)


def joy_servo(value, min_limit=-0.99, max_limit=0.99, map_to_min=39, map_to_max=150):
    value = max(min(value, max_limit), min_limit)
    mapped_value = ((value - min_limit) / (max_limit - min_limit)) * (map_to_max - map_to_min) + map_to_min
    print(mapped_value)
    return int(mapped_value)

# def joy_servo(joystick_value, offset=8):

#     return int((joystick_value + 1) * 90)

def emergency_stop():
    # Set the ESC to neutral position to stop the car
    ESC.angle = 0  # Assuming 90 is the neutral position for your ESC
    print("Emergency Stop Activated!")


def is_controller_connected(joystick):
    try:
        # Attempt to get the name of the joystick to check if it's still connected
        joystick.get_name()
        return True
    except pygame.error:
        # If an error occurs (e.g., joystick not connected), return False
        return False


def main():
    pygame.init()
    motor_init()

    # Initialize the joystick
    pygame.joystick.init()

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print(f"Joystick Name: {joystick.get_name()}")
        print(f"Number of Axes: {joystick.get_numaxes()}")
        print(f"Number of Buttons: {joystick.get_numbuttons()}")

        while True:
            pygame.event.pump()

            # Check if the controller is still connected
            if not is_controller_connected(joystick):
                emergency_stop()
                break  # Optionally break out of the loop or handle reconnection

            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    # Handle joystick axis motion
                    if event.axis == 0:
                        # Axis 0 (left stick horizontal)
                        axis_value = joystick.get_axis(0)
                        servo1.angle = joy_servo(axis_value)
                        print(f"Axis 0 Value: {axis_value}")

                    elif event.axis == 2:
                        # Axis 2 (left trigger)
                        axis_value = joystick.get_axis(2)
                        print(f"Left Trigger Value: {axis_value}")

                    elif event.axis == 4:
                        # Axis 5 (right trigger)
                        axis_value = joystick.get_axis(4)
                        ESC.angle = joy_throttle(axis_value)
                        print(f"Right Trigger Value: {axis_value}")

                elif event.type == pygame.JOYBUTTONDOWN:
                    # Handle button press
                    button = event.button
                    print(f"Button {button} pressed")

                elif event.type == pygame.JOYBUTTONUP:
                    # Handle button release
                    button = event.button
                    print(f"Button {button} released")

    except KeyboardInterrupt:
        pass
    finally:
        emergency_stop()
        pygame.quit()
        pca.deinit()


if __name__ == "__main__":
    main()
