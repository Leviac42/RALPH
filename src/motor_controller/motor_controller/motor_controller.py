import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import serial


class ControlNode(Node):

    def __init__(self):
        super().__init__("control_node")

        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.on_joy, 10
        )
        # self.subscription_twist = self.create_subscription(
        #     Twist, "/cmd_vel", self.on_twist, 10
        # )

        self.motor_left = 0
        self.motor_right = 0
        self.motor_drive = 0
        self.turn_axis = 0
        self.stick_button = 0
        self.stick = "dual"
        self.turn_speed = 0
        self.forward_speed = 0
        self.reverse_speed = 0
        
        self.serial_port = serial.Serial("/dev/ttyS0", 9600, timeout=0.5)

    def scale(self, value, in_min, in_max, out_min, out_max):
        # Scale the value from the input range to the output range.
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def switch_control_mode(self, stick):
    # Get the current control mode.
        current_mode = stick
        # Switch to the other control mode.
        if current_mode == "single":
            stick = "dual"
        else:
            stick = "single"

        # Return the new control mode.
        return stick

    def on_joy(self, msg):
        # Print joystick values.
        # self.get_logger().info("Axes: {}".format(msg.axes))
        # self.get_logger().info("Axes 0: {}".format(msg.axes[0]))
        # self.get_logger().info("Axes 1: {}".format(msg.axes[1]))
        # self.get_logger().info("Axes 2: {}".format(msg.axes[2]))
        # self.get_logger().info("Axes 3: {}".format(msg.axes[3]))
        # self.get_logger().info("Axes 4: {}".format(msg.axes[4]))
        # self.get_logger().info("Axes 5: {}".format(msg.axes[5]))
        # self.get_logger().info("Axes 6: {}".format(msg.axes[6]))
        # self.get_logger().info("Axes 7: {}".format(msg.axes[7]))
        # self.get_logger().info("Buttons: {}".format(msg.buttons))
        
        # self.motor_left = msg.axes[4] * 100
        # self.motor_right = msg.axes[1] * 100
        # self.turn_axis = msg.axes[0] * 100
        
        # Dual stick mode, left stick controls left motor, right stick controls right motor

        self.stick_button = msg.buttons[0]

        if self.stick_button == 1:
            self.stick = self.switch_control_mode(self.stick)
    

        if self.stick == "dual":
            self.dual_stick(msg)
        else:
            self.single_stick(msg)


    def dual_stick(self, msg):
        # self.stick_button = msg.buttons[0]
        # Left joy stick axes[4]
        self.motor_left = msg.axes[4] * 100
        self.motor_right = msg.axes[1] * 100
        self.forward_speed = msg.axes[5] * 100
        self.reverse_speed = msg.axes[2] * 100

        if self.forward_speed > 99:
            self.forward_speed = 100
        elif self.forward_speed < -99:
            self.forward_speed = -100

        if self.reverse_speed > 99:
            self.reverse_speed = 100
        elif self.reverse_speed < -99:
            self.reverse_speed = -100

        if self.forward_speed <.05 and self.forward_speed > -.05:
            self.forward_speed = 0
        if self.reverse_speed <.05 and self.reverse_speed > -.05:
            self.reverse_speed = 0


        self.reverse_speed = self.scale(self.reverse_speed, 100, -100, 0, 100)
        self.forward_speed = self.scale(self.forward_speed, 100, -100, 0, 100)

        self.motor_left, self.motor_right = self.convert_to_motor_packet(self.motor_left, self.motor_right, self.forward_speed, self.reverse_speed)

        return self.motor_left, self.motor_right 

    def convert_to_motor_packet(self, motor_left, motor_right, forward_speed, reverse_speed):
        def map_value(value, in_min, in_max, out_min, out_max):
            return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

        def clamp(value, min_value, max_value):
            return max(min(value, max_value), min_value)

        if forward_speed == 0 and reverse_speed == 0 and motor_left == 0 and motor_right == 0:
            return 64, 192  # Full stop for both motors

        
        if forward_speed > 0:
            base_speed = map_value(forward_speed, 0, 100, 1, 63)
            mode_flag = "Forward"
        elif reverse_speed > 0:
            base_speed = map_value(reverse_speed, 0, 100, 1, 63)
            mode_flag = "Reverse"
        else:
            base_speed = 0
            mode_flag = "Turn"

        # Mapping joystick values for turning
        motor_left_delta = map_value(motor_left, -100, 100, -63, 63)
        motor_right_delta = map_value(motor_right, -100, 100, -63, 63)

        motor_left_speed = base_speed + motor_left_delta
        motor_right_speed = base_speed + motor_right_delta

        # Check if both forward_speed and reverse_speed are 0 for spinning in opposite directions
        if forward_speed == 0 and reverse_speed == 0:
            if motor_left != 0:
                motor_left_speed = motor_left_delta 
            elif motor_right != 0:
                motor_right_speed = motor_right_delta
            # motor_left_speed = motor_left_delta
            # motor_right_speed = motor_right_delta

        # Map to the correct range based on forward or reverse flag
        if mode_flag == "Reverse":
            motor_left_speed = map_value(motor_left_speed, -63, 126, 65, 127)
            motor_right_speed = map_value(motor_right_speed, -63, 126, 129, 191)
        elif mode_flag == "Forward":
            motor_left_speed = map_value(motor_left_speed, -63, 126, 1, 63)
            motor_right_speed = map_value(motor_right_speed, -63, 126, 193, 255)
        elif mode_flag == "Turn":
            if motor_left > 0:
                motor_left_speed = map_value(motor_left_speed, -63, 126, 1, 63)
            elif motor_left < 0:
                motor_left_speed = map_value(motor_left_speed, -63, 126, 65, 127)
            if motor_right > 0:
                motor_right_speed = map_value(motor_right_speed, -63, 126, 129, 191)
            elif motor_right < 0:
                motor_right_speed = map_value(motor_right_speed, -63, 126, 193, 255)

        # Clamp to ensure within valid range
        motor_left_speed = clamp(motor_left_speed, 1, 127)
        motor_right_speed = clamp(motor_right_speed, 129, 255)

        return motor_left_speed, motor_right_speed    
    

    def single_stick(self, msg):
        forward_speed = msg.axes[0] * 100
        turn_speed = msg.axes[1] * 100
        # self.stick_button = msg.buttons[0]

        
        # Calculate basic motor speeds from forward input
        if forward_speed > 0:
            self.motor_left = self.scale(forward_speed, 1, 100, 1, 63)
            self.motor_right = self.scale(forward_speed, 1, 100, 129, 191)
        elif forward_speed < 0:
            self.motor_left = self.scale(forward_speed, -1, -100, 65, 127)
            self.motor_right = self.scale(forward_speed, -1, -100, 193, 255)
        else:
            self.motor_left = 64
            self.motor_right = 192
        
        # Adjust motor speeds for turning
        if turn_speed > 0:  # Stick moved to the right
            # Decrease speed of right motor and increase speed of left motor
            # self.motor_left = min(self.motor_left + abs(self.motor_left-turn_speed), 63) if forward_speed >= 0 else max(self.motor_left - abs(turn_speed), 65)
            self.motor_left = self.scale(min(self.motor_left+self.turn_speed, 63), 1, 63, 1, 63)
            # self.motor_right = max(self.motor_right - abs(self.motor_right+turn_speed), 129) if forward_speed >= 0 else min(self.motor_right + abs(turn_speed), 191)
            self.motor_right = self.scale(max(self.motor_right-self.turn_speed, 129), 129, 191, 129, 191)
        elif turn_speed < 0:  # Stick moved to the left
            # Decrease speed of left motor and increase speed of right motor
            # self.motor_left = max(self.motor_left - abs(self.motor_left+turn_speed), 1) if forward_speed >= 0 else min(self.motor_left + abs(turn_speed), 127)
            # self.motor_right = min(self.motor_right + abs(self.motor_right-turn_speed), 191) if forward_speed >= 0 else max(self.motor_right - abs(turn_speed), 193)
            self.motor_left = self.scale(max(self.motor_left-self.turn_speed, 1), 1, 63, 1, 63)
            self.motor_right = self.scale(min(self.motor_right+self.turn_speed, 191), 129, 191, 129, 191)
        

        # Spot turn when only axis[0] is being used and no forward/backward movement
        if forward_speed == 0:
            if turn_speed > 0:  # Right spot turn
                self.motor_left = self.scale(turn_speed, 1, 100, 1, 63)
                self.motor_right = self.scale(-turn_speed, -1, -100, 193, 255)
            elif turn_speed < 0:  # Left spot turn
                self.motor_left = self.scale(turn_speed, -1, -100, 65, 127)
                self.motor_right = self.scale(-turn_speed, 1, 100, 129, 191)
        # if self.stick_button == 1:
        #     if self.stick == "single":
        #         self.stick = "dual"
        #     else:
        #         self.stick = "single"
        return self.motor_left, self.motor_right            
                

    def publish_motor_commands(self):
        packetleft = bytearray()
        packetright = bytearray()

        packetleft.append(int(self.motor_left))
        packetright.append(int(self.motor_right))

        self.serial_port.write(packetleft)
        self.serial_port.write(packetright)

def main():
    rclpy.init()

    control_node = ControlNode()

    try:
        while rclpy.ok():
            control_node.publish_motor_commands()
            rclpy.spin_once(control_node)
    except KeyboardInterrupt:
        pass


    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
