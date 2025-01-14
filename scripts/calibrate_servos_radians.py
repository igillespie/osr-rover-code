"""
Simple calibration/test script for PCA9685 PWM motor control

Moves motors back and forth

Setup:
$ pip3 install adafruit-circuitpython-servokit

Instructions: https://learn.adafruit.com/adafruit-16-channel-servo-driver-with-raspberry-pi/using-the-adafruit-library
"""
from time import sleep
import argparse
from math import degrees
from adafruit_servokit import ServoKit

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog='ServoCalibrator',
        description='Helps calibrate the corner motors. When running this script, make sure that the servo motors can move freely.',
        epilog='If you need help, please ask on Slack on the #troubleshooting channel!'
    )
    
    parser.add_argument(
        'motor_index', 
        type=int, 
        choices=range(16), 
        help="Which channel on the PCA9685 board and thus which motor we're commanding. "
             "Normally, 0 corresponds to the back right corner, 1 to the front right, 2 to the front left, and 3 to the back left."
    )
    parser.add_argument(
        'target_angle', 
        type=float, 
        help="The target angle in radians. Assumes 0 radians maps to 150° and adjusts accordingly."
    )
    args = parser.parse_args()

    # Define the mapping
    base_angle_degrees = 150  # Corresponds to 0 radians
    radians_to_degrees_offset = degrees(args.target_angle)
    mapped_angle = base_angle_degrees + radians_to_degrees_offset

    kit = ServoKit(channels=16)
    sleep(0.1)
    kit.servo[args.motor_index].actuation_range = 300
    kit.servo[args.motor_index].set_pulse_width_range(500, 2500)
    kit.servo[args.motor_index].angle = mapped_angle
    print(f"Servo motor at channel {args.motor_index} was set to {mapped_angle}° (input: {args.target_angle} radians)")