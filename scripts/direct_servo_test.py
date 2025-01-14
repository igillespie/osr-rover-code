import board
from adafruit_pca9685 import PCA9685
from time import sleep
import argparse

# Initialize I2C and PCA9685
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50  # Set frequency for standard servos (50 Hz)

# Function to calculate duty cycle from pulse width
def pulse_width_to_duty_cycle(pulse_width, frequency=50):
    pulse_length = 1_000_000 / frequency  # Pulse length in microseconds (e.g., 20ms for 50Hz)
    duty_cycle = int((pulse_width / pulse_length) * 0xFFFF)  # Convert to 16-bit duty cycle
    return duty_cycle

# Function to convert angle to pulse width
def angle_to_pulse_width(angle, min_angle=0, max_angle=300, min_pulse=500, max_pulse=2500):
    """
    Converts an angle to a corresponding pulse width.
    :param angle: Desired servo angle (0° to 300°).
    :param min_angle: Minimum angle (default: 0°).
    :param max_angle: Maximum angle (default: 300°).
    :param min_pulse: Minimum pulse width in μs (default: 500 μs).
    :param max_pulse: Maximum pulse width in μs (default: 2500 μs).
    :return: Corresponding pulse width in μs.
    """
    if not (min_angle <= angle <= max_angle):
        raise ValueError(f"Angle {angle} out of range ({min_angle}–{max_angle})")
    # Map angle to pulse width
    pulse_width = min_pulse + (angle / max_angle) * (max_pulse - min_pulse)
    return pulse_width

# Function to move the servo to a specific angle
def set_servo_angle(channel, angle):
    """
    Moves the servo on a specific channel to a desired angle.
    :param channel: PCA9685 channel number (0–15).
    :param angle: Desired servo angle (0° to 300°).
    """
    pulse_width = angle_to_pulse_width(angle)  # Convert angle to pulse width
    duty_cycle = pulse_width_to_duty_cycle(pulse_width)  # Convert pulse width to duty cycle
    pca.channels[channel].duty_cycle = duty_cycle  # Set duty cycle
    print(f"Moved channel {channel} to angle {angle}° (Pulse width: {pulse_width:.2f} μs, duty cycle {duty_cycle:.2f})")

# Example usage
# channel = 4  # Servo channel
# set_servo_angle(channel, 0)     # Move to 0°
# sleep(2)
# set_servo_angle(channel, 150)   # Move to center (150°)
# sleep(2)
# set_servo_angle(channel, 300)   # Move to 300°
# sleep(2)

# # Clean up: Turn off PWM output for the servo
# pca.channels[channel].duty_cycle = 0
# print("Servo motion complete.")

def main():
    parser = argparse.ArgumentParser(description="Control a servo using PCA9685.")
    parser.add_argument("channel", type=int, help="PCA9685 channel number (0–15).")
    parser.add_argument("angle", type=float, help="Desired servo angle (0° to 300°).")
    args = parser.parse_args()

    # Move the servo to the specified angle
    try:
        set_servo_angle(args.channel, args.angle)
        sleep(3)
    except ValueError as e:
        print(f"Error: {e}")
    finally:
        # Clean up: Turn off PWM output for the servo
        pca.channels[args.channel].duty_cycle = 0
        print("Servo motion complete.")

if __name__ == "__main__":
    main()