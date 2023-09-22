import unittest
from motor_controller import MotorController

class TestMotorController(unittest.TestCase):

    def setUp(self):
        self.controller = MotorController()

    def test_single_stick_forward(self):
        msg = MockMsg(axes=[1, 0])
        left, right = self.controller.single_stick(msg)
        self.assertEqual(left, 63)
        self.assertEqual(right, 129)

    def test_single_stick_backward(self):
        msg = MockMsg(axes=[-1, 0])
        left, right = self.controller.single_stick(msg)
        self.assertEqual(left, 65)
        self.assertEqual(right, 193)

    def test_single_stick_left(self):
        msg = MockMsg(axes=[0, -1])
        left, right = self.controller.single_stick(msg)
        self.assertEqual(left, 65)
        self.assertEqual(right, 129)

    def test_single_stick_right(self):
        msg = MockMsg(axes=[0, 1])
        left, right = self.controller.single_stick(msg)
        self.assertEqual(left, 63)
        self.assertEqual(right, 191)

    def test_single_stick_spot_turn_left(self):
        msg = MockMsg(axes=[0, -1])
        self.controller.single_stick(msg)
        msg = MockMsg(axes=[0, -0.5])
        left, right = self.controller.single_stick(msg)
        self.assertEqual(left, 65)
        self.assertEqual(right, 191)

    def test_single_stick_spot_turn_right(self):
        msg = MockMsg(axes=[0, 1])
        self.controller.single_stick(msg)
        msg = MockMsg(axes=[0, 0.5])
        left, right = self.controller.single_stick(msg)
        self.assertEqual(left, 63)
        self.assertEqual(right, 193)

    def test_single_stick_no_movement(self):
        msg = MockMsg(axes=[0, 0])
        left, right = self.controller.single_stick(msg)
        self.assertEqual(left, 64)
        self.assertEqual(right, 192)

    def test_publish_motor_commands(self):
        self.controller.motor_left = 63
        self.controller.motor_right = 129
        self.controller.publish_motor_commands()
        # assert that the motor commands were sent to the serial port

class MockMsg:
    def __init__(self, axes, buttons=None):
        self.axes = axes
        self.buttons = buttons

if __name__ == '__main__':
    unittest.main()