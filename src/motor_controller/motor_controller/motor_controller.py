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
        self.stick = "single"
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


        # self.get_logger().info("Axes 0: {}".format(msg.axes[0]))
        # self.get_logger().info("Axes 1: {}".format(msg.axes[1]))
        # self.get_logger().info("Axes 2: {}".format(msg.axes[2]))
        # self.get_logger().info("Axes 3: {}".format(msg.axes[3]))
        # self.get_logger().info("Axes 4: {}".format(msg.axes[4]))
        # self.get_logger().info("Axes 5: {}".format(msg.axes[5]))
        # self.get_logger().info("Axes 6: {}".format(msg.axes[6]))
        # self.get_logger().info("Axes 7: {}".format(msg.axes[7]))

        # self.forward_speed = self.scale(self.forward_speed, 100, -100, 0, 100)
        # self.reverse_speed = self.scale(self.reverse_speed, 100, -100, 0, 100)

        if self.forward_speed > 0 and self.reverse_speed == 0:
            self.motor_left = self.scale(self.forward_speed, 100, -100, 1, 63)
            self.motor_right = self.scale(self.forward_speed, 100, -100, 193, 255)
        elif self.reverse_speed > 0 and self.forward_speed == 0:
            self.motor_left = self.scale(self.reverse_speed, 100, -100, 65, 127)
            self.motor_right = self.scale(self.reverse_speed, 100, -100, 129, 191)
        else:
            self.forward_speed = 0
            self.reverse_speed = 0


        if self.motor_left > 0 and self.forward_speed == 0 and self.reverse_speed == 0:
            self.motor_left = self.scale(self.motor_left, 1, 100, 1, 63)
        elif self.motor_left < 0 and self.forward_speed == 0 and self.reverse_speed == 0:
            self.motor_left = self.scale(self.motor_left, -1, -100, 65, 127)
        elif self.motor_left == 0 and self.forward_speed == 0 and self.reverse_speed == 0:
            self.motor_left = 64
        if self.motor_right > 0 and self.forward_speed == 0 and self.reverse_speed == 0:
            self.motor_right = self.scale(self.motor_right, 1, 100, 193, 255)
        elif self.motor_right < 0 and self.forward_speed == 0 and self.reverse_speed == 0:
            self.motor_right = self.scale(self.motor_right, -1, -100, 129, 191)
        elif self.motor_left == 0 and self.forward_speed == 0 and self.reverse_speed == 0:
            self.motor_right = 192


        self.get_logger().info("Motor Left: {}".format(self.motor_left))
        self.get_logger().info("Motor Right: {}".format(self.motor_right))
        self.get_logger().info("Forward Speed: {}".format(self.forward_speed))
        self.get_logger().info("Reverse Speed: {}".format(self.reverse_speed))

        return self.motor_left, self.motor_right
    
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
