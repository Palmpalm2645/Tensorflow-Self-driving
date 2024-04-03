import pygame

def main():
    pygame.init()

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

            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    # Handle joystick axis motion
                    if event.axis == 0:
                        # Axis 0 (left stick horizontal)
                        axis_value = joystick.get_axis(0)
                        print(f"Axis 0 Value: {axis_value}")

                    elif event.axis == 2:
                        # Axis 2 (left trigger)
                        axis_value = joystick.get_axis(2)
                        print(f"Left Trigger Value: {axis_value}")

                    elif event.axis == 4:
                        # Axis 5 (right trigger)
                        axis_value = joystick.get_axis(4)
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
        pygame.quit()

if __name__ == "__main__":
    main()
