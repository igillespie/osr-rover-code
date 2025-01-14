from adafruit_motor.servo import Servo

class PCA9685Kit:
    def __init__(self, kit):
        """
        Initializes the PCA9685Kit wrapper around an Adafruit ServoKit instance.

        :param kit: An instance of adafruit_servokit.ServoKit.
        """
        self.kit = kit
        self.channels = {}

    def configure_servo(self, channel, actuation_range=180, min_pulse=750, max_pulse=2250):
        """
        Configures a servo on a given channel.

        :param channel: The PCA9685 channel to configure.
        :param actuation_range: The range of motion of the servo in degrees.
        :param min_pulse: Minimum pulse width in microseconds.
        :param max_pulse: Maximum pulse width in microseconds.
        """
        if channel not in self.channels:
            self.channels[channel] = {
                "actuation_range": actuation_range,
                "min_pulse": min_pulse,
                "max_pulse": max_pulse,
            }
            print(f"Configured servo on channel {channel} with actuation range {actuation_range}, pulse width range {min_pulse}–{max_pulse}")
        else:
            print(f"Channel {channel} is already configured.")

    def move_to_angle(self, channel, angle):
        """
        Moves the servo on a specified channel to a given angle.

        :param channel: The PCA9685 channel to control.
        :param angle: The desired angle (0 to actuation_range).
        """
        if channel not in self.channels:
            raise ValueError(f"Channel {channel} is not configured. Call configure_servo() first.")

        # Retrieve servo configuration
        config = self.channels[channel]
        actuation_range = config["actuation_range"]
        min_pulse = config["min_pulse"]
        max_pulse = config["max_pulse"]

        # Validate angle
        if not (0 <= angle <= actuation_range):
            raise ValueError(f"Angle {angle} is out of range (0–{actuation_range})")

        # Map angle to pulse width
        pulse_width = min_pulse + (angle / actuation_range) * (max_pulse - min_pulse)
        duty_cycle = int((pulse_width / (1_000_000 / 50)) * 0xFFFF)  # Assuming 50 Hz frequency

        # Set duty cycle directly on PCA9685
        self.kit._pca.channels[channel].duty_cycle = duty_cycle
        #print(f"Moved servo on channel {channel} to angle {angle}° (Pulse width: {pulse_width:.2f} μs, Duty cycle: {duty_cycle})")