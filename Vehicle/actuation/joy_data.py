import csv
import time
import pygame

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

def main():
    pygame.init()
    pygame.joystick.init()

    try:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        print(f"Joystick Name: {joystick.get_name()}")
        print(f"Number of Axes: {joystick.get_numaxes()}")
        print(f"Number of Buttons: {joystick.get_numbuttons()}")
        

        data = []

        while True:
            pygame.event.pump()

            timestamp = time.time()
            joystick_value = joystick.get_axis(0)  # Change the index for different axes
            mapped_angle = map_joystick_value(joystick_value, map_to_min=0, map_to_max=180)

            data.append({'Timestamp': timestamp, 'JoystickValue': joystick_value, 'MappedAngle': mapped_angle})

            # Print the values (optional)
            print(f"Timestamp: {timestamp}, JoystickValue: {joystick_value}, MappedAngle: {mapped_angle}")

            time.sleep(0.1)  # Add a small delay to avoid high data rates

    except KeyboardInterrupt:
        pass
    finally:
        # Save data to CSV file before quitting
        save_to_csv('joystick_data.csv', data)
        pygame.quit()

if __name__ == "__main__":
    main()
