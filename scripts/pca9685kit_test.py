import argparse
from adafruit_servokit import ServoKit
from pca9685_kit import PCA9685Kit

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(
        description="Control a servo connected to the PCA9685.",
        epilog="Example: python3 servo_control.py --channel 4 --angle 190"
    )
    parser.add_argument("--channel", type=int, required=True, help="The PCA9685 channel to control (0–15).")
    parser.add_argument("--angle", type=float, required=True, help="The angle to move the servo to (0–300).")
    args = parser.parse_args()

    # Initialize ServoKit and PCA9685Kit
    kit = ServoKit(channels=16)
    pca_kit = PCA9685Kit(kit)

    # Configure the servo
    pca_kit.configure_servo(channel=args.channel, actuation_range=300, min_pulse=500, max_pulse=2500)

    # Move the servo to the specified angle
    pca_kit.move_to_angle(channel=args.channel, angle=args.angle)

if __name__ == "__main__":
    main()