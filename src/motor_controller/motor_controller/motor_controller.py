import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import serial
import random

class FakeSerial:
    def __init__(self):
        self.connected = False
    
    def open(self):
        self.connected = True
        print("Fake serial connection opened.")
        
    def close(self):
        self.connected = False
        print("Fake serial connection closed.")
        
    def write(self, data):
        if self.connected:
            print(f"Fake serial sending: {data}")
            return len(data)
        else:
            print("Fake serial is not open. Cannot write.")
            return 0
    
    def readline(self):
        if self.connected:
            # Generate some fake data to simulate a serial read
            fake_data = f"FAKE_DATA_{random.randint(0, 100)}\r\n"
            print(f"Fake serial receiving: {fake_data}")
            return fake_data.encode('utf-8')
        else:
            print("Fake serial is not open. Cannot read.")
            return b""


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

        try:
            self.serial_port = serial.Serial("/dev/ttyAML0", 9600, timeout=0.5)
            self.get_logger().info("Opened serial port")
        except ImportError as e:
            print(f"Error opening serial port: {e}")
            self.get_logger().info("Failed to open serial port")
            self.serial_port = None

        if self.serial_port is None:
            self.serial_port = FakeSerial()
            self.get_logger().info("Using fake serial port")

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

        # if self.forward_speed <.05 and self.forward_speed > -.05:
        #     self.forward_speed = 0
        # if self.reverse_speed <.05 and self.reverse_speed > -.05:
        #     self.reverse_speed = 0


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
            #'''Print out the motor speeds for debugging'''
            # self.get_logger().info("Motor Left Speed: {}".format(motor_left_speed))
            # self.get_logger().info("Motor Right Speed: {}".format(motor_right_speed))
            motor_left_speed = map_value(motor_left_speed, 1, 126, 65, 127)
            motor_right_speed = map_value(motor_right_speed, 1, 126, 129, 191)
        elif mode_flag == "Forward":
            motor_left_speed = clamp(map_value(motor_left_speed, 1, 126, 1, 63), 1, 63)
            motor_right_speed = clamp(map_value(motor_right_speed, 1, 126, 193, 255), 193, 255)
        elif mode_flag == "Turn":
            if motor_left > 0:
                motor_left_speed = map_value(motor_left_speed, 0, 126, 1, 63)
            elif motor_left < 0:
                motor_left_speed = map_value(motor_left_speed, 0, -126, 65, 127)
            if motor_right > 0:
                motor_right_speed = map_value(motor_right_speed, 0, 126, 193, 255)
            elif motor_right < 0:
                motor_right_speed = map_value(motor_right_speed, 0, -126, 129, 191)

        # Clamp to ensure within valid range
        motor_left_speed = clamp(motor_left_speed, 1, 127)
        motor_right_speed = clamp(motor_right_speed, 129, 255)

        return motor_left_speed, motor_right_speed    
    
    def single_stick(self, msg):
        # Extract joystick values for forward and turning speed.
        forward_speed = msg.axes[0] * 100
        turn_speed = msg.axes[1] * 100

        # Handle forward/reverse movement
        if forward_speed >= 0:
            motor_left_speed = motor_right_speed = forward_speed
        else:
            motor_left_speed = motor_right_speed = forward_speed

        # Handle turning, assuming turn_speed positive for right turn, negative for left turn.
        motor_left_speed += turn_speed
        motor_right_speed -= turn_speed

        # Clamp motor speed values to range [-100, 100] before passing to convert_to_motor_packet.
        motor_left_speed = max(min(motor_left_speed, 100), -100)
        motor_right_speed = max(min(motor_right_speed, 100), -100)

        # Call convert_to_motor_packet with the calculated motor speeds and 0 for forward and reverse speeds.
        self.motor_left, self.motor_right = self.convert_to_motor_packet(motor_left_speed, motor_right_speed, 0, 0)

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
