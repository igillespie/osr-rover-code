import rclpy
from rclpy.node import Node
import math
from rclpy.parameter import Parameter

# project libraries
from adafruit_servokit import ServoKit

# message imports
from sensor_msgs.msg import JointState
from osr_interfaces.msg import CommandCorner, Status

RAD_TO_DEG = 180 / math.pi




""" Not sure if there is a bug in ServoKit or not, but I could never get them to actuate a full 300˚. After experimenting, I found if I manually calculate the right duty cycle based on the passed angle and max actuation range, I can get the servos to move how they should. """

# Define the method
def move_servo_to_angle(kit, channel, angle, actuation_range, min_pulse=500, max_pulse=2500):
    """
    Custom method to set servo angle with actuation range and pulse width mapping.
    :param kit: ServoKit instance managing the PCA9685.
    :param channel: Servo channel number (0–15).
    :param angle: Target angle to set (0 to actuation_range).
    :param actuation_range: Full range of servo angles (default: 300°).
    :param min_pulse: Minimum pulse width in μs (default: 500 μs).
    :param max_pulse: Maximum pulse width in μs (default: 2500 μs).
    """
    if not (0 <= angle <= actuation_range):
        raise ValueError(f"Angle {angle} is out of range (0–{actuation_range})")

    # Map angle to pulse width
    pulse_width = min_pulse + (angle / actuation_range) * (max_pulse - min_pulse)
    duty_cycle = int((pulse_width / (1_000_000 / 50)) * 0xFFFF)  # Assuming 50 Hz frequency

    # Access the PCA9685 controller via the ServoKit instance
    kit._pca.channels[channel].duty_cycle = duty_cycle

    #print(f"Servo on channel {channel} moved to {angle}° (Pulse width: {pulse_width:.2f} μs)")







class ServoWrapper(Node):
    """Interface between the PCA9685 controlling the servos and the higher level rover code"""
    corner_motors = ['corner_right_back', 'corner_right_front', 'corner_left_front', 'corner_left_back']

    def __init__(self):
        super().__init__("servo_wrapper")
        self.log = self.get_logger()
        # self.log.set_level(10)
        self.log.info("Initializing corner servo controllers")
        self.kit = None
        self.declare_parameters(
            namespace='',
            parameters=[
                ('centered_pulse_widths', Parameter.Type.INTEGER_ARRAY)
            ]
        )

        # PWM settings from https://www.gobilda.com/2000-series-dual-mode-servo-25-2-torque/
        self.servo_actuation_range = 300  # [deg]
        self.centered_pulse_widths = self.get_parameter('centered_pulse_widths').get_parameter_value().integer_array_value
        assert(len(self.centered_pulse_widths) == 4)
        self.pulse_width_range = (500, 2500)  # [microsec]
        self.deg_per_sec = 200
        # initial values for position estimate (first element) and goal (second element) for each corner motor in deg
        self.corner_state_goal = [(0, 0)] * 4

        self.connect_pca9685()
        
        self.enc_pub = self.create_publisher(JointState, "/corner_state", 1)
        self.corner_cmd_sub = self.create_subscription(CommandCorner, "/cmd_corner", self.corner_cmd_cb, 1)
        self.enc_pub_timer_period = 0.1  # [s]
        self.servo_direction = -1  # set to 1 if the servos are positive pwm clockwise
        self.enc_pub_timer = self.create_timer(self.enc_pub_timer_period, self.publish_encoder_estimate)

    def connect_pca9685(self):
        self.log.debug("Creating ServoKit instance")
        self.kit = ServoKit(channels=16)

        self.log.info("setting servo params")
        for servo_id in range(4):
            self.kit.servo[servo_id].actuation_range = self.servo_actuation_range
            #self.kit.servo[servo_id].set_pulse_width_range(*self.pulse_width_range)

    def corner_cmd_cb(self, cmd: CommandCorner):
        #self.log.info(f"Received corner command message: {cmd}", throttle_duration_sec=1)
        if not self.kit:
            self.log.error("ServoKit not instantiated yet, dropping cmd", throttle_duration_sec=5)
            return

        for ind, corner_name in zip(range(4), self.corner_motors):
            # store goal so we can estimate current angle
            angle = getattr(cmd, corner_name[7:]+"_pos") * RAD_TO_DEG
            # TODO make readable, cleaner
            self.corner_state_goal[ind] = (self.corner_state_goal[ind][0], angle)
            # offset to coordinate frame where x points to the middle of the rover, z down
            # and apply middle of actuation range offset, taking into account if servo is positive ccw or cw
            angle = self.centered_pulse_widths[ind] + (self.servo_direction * angle)
            #self.log.info(f"motor {corner_name} commanded to {angle}")
            # limit to operating range of servo
            angle = max(min(angle, self.servo_actuation_range), 0)
            # send to motor
            #self.kit.servo[ind].angle = angle

            #Use our custom function that does a better job of setting the servo positions
            move_servo_to_angle(self.kit, ind, angle, self.servo_actuation_range)

    def publish_encoder_estimate(self):
        """
        Publish an estimate of where each corner motor currently is.
        
        Estimate is based on the last estimate + velocity * time delta if there
        is a difference between goal and current angle
        and is expressed in the motor frame (z down, x forward)
        """
        enc_msg = JointState()
        enc_msg.header.stamp = self.get_clock().now().to_msg()
        for ind, motor_name in zip(range(4), self.corner_motors):
            curr_angle, goal_angle = self.corner_state_goal[ind]
            self.log.debug(f"motor {motor_name}: curr_angle: {curr_angle}, goal: {goal_angle}", throttle_duration_sec=1)
            goal_differential = goal_angle - curr_angle
            velocity = 0
            # compare differential to step size so we can't overshoot and oscillate
            if abs(goal_differential) > self.deg_per_sec * self.enc_pub_timer_period:
                # assume we're running at the desired frequency
                deg_traveled = self.deg_per_sec * self.enc_pub_timer_period
                self.corner_state_goal[ind]= (curr_angle + math.copysign(deg_traveled, goal_differential), goal_angle)
                velocity = math.copysign(self.deg_per_sec, goal_differential)
                
            elif abs(goal_differential) != 0:
                # assume we're pretty much there
                self.corner_state_goal[ind] = (goal_angle, goal_angle)
            
            enc_msg.name.append(motor_name)
            enc_msg.position.append(self.corner_state_goal[ind][0] / RAD_TO_DEG)
            enc_msg.velocity.append(velocity / RAD_TO_DEG)
            enc_msg.effort.append(0)
        self.enc_pub.publish(enc_msg)

def main(args=None):
    rclpy.init(args=args)

    wrapper = ServoWrapper()

    rclpy.spin(wrapper)
    wrapper.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
