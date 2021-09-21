import rclpy
from rclpy.node import Node
import odrive
from odrive.enums import *
import time
from std_msgs.msg import Float32
import fibre.utils
import fibre.remote_object

class ExcavationNode(Node):
	def __init__(self):
		super().__init__('exavation_node')
		print("Finding ODrive")
		self.allDrives = odrive.find_any(path="usb", find_multiple=True)
		self.odrv0 = self.allDrives[0]
		print("ODrive voltage:")
		print(str(self.odrv0.vbus_voltage))
		self.calibrate()
		self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		self.odrv0.axis0.controller.input_vel = 0
		self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
		#self.odrv0.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
		#self.odrv0.axis1.controller.input_pos = 0
		self.odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
		self.odrv0.axis1.controller.input_vel = 0
		self.left_speed_subscription = self.create_subscription(Float32, 'drive_left_speed', self.left_listener_callback, 10)
		self.right_speed_subscription = self.create_subscription(Float32, 'drive_right_speed', self.right_listener_callback, 10)

	def left_listener_callback(self, msg):
		speed = msg.data * 20
		self.odrv0.axis0.controller.input_vel = speed
		print("msg.data: " + str(msg.data))
		print("speed: " + str(speed))

	def right_listener_callback(self, msg):
		#position = 0
		#self.odrv0.axis1.controller.input_pos += position
		speed = msg.data * 20
		self.odrv0.axis1.controller.input_vel = speed
		print("msg.data: " + str(msg.data))
		print("speed: " + str(speed))

	def calibrate(self):
		self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
			time.sleep(0.1)
		self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
		while self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
			time.sleep(0.1)
		print("Calibrated")


def main(args=None):
	rclpy.init(args=args)

	excavation_node = ExcavationNode()

	rclpy.spin(excavation_node)

	excavation_node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
