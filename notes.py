Python function that takes in a value from the controller and converts it to a value that can be sent to the motor controller.
Motor Controller Serial Packet Values:
Motor left CW Speed min 1 max 63
Motor left CCW speed min 65 max 127
Motor left Full Stop 64
Motor right CW Speed min 129 max 191
Motor right CCW speed min 193 max 255
Motor right Full Stop 192


Input from controller ranges:
Left joystick forward 0-100, Variable name motor_left
Left joystick backward 0-(-100), Variable name motor_left
Right joystick forward 0-100, Variable name motor_right
Right joystick backward 0-(-100), Variable name motor_right
Left joystick center 0, Variable name motor_left 
Right joystick center 0, Variable name motor_right
Left trigger button for reverse 0-100, Variable name forward_speed
Right trigger button for reverse 0-100, Variable name reverse speed



To get bluetooth to work:
