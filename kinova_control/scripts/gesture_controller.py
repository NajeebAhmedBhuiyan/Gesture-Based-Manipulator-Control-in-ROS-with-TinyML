#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kinova_control.srv import KeyTrigger
import serial
from time import sleep

class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        self.service_client = self.create_client(KeyTrigger, 'key_trigger_service')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.ser.flush()
        
        self.gesture_map = {
            "up-down": "e",
            "to-fro": "t",
            "left-right": "f",
            "rectangle": "g",
            "rectangle-flat": "h",
            "circle": "v"
        }
        
        self.get_logger().info("Gesture controller ready")

    def send_service_request(self, key):
        req = KeyTrigger.Request()
        req.key_pressed = key
        future = self.service_client.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Motion executed successfully")
            else:
                self.get_logger().error(f"Motion execution failed")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def read_gestures(self):
        try:
            while rclpy.ok():
                if self.ser.in_waiting > 0:
                    gesture = self.ser.readline().decode('utf-8').strip()
                    if gesture in self.gesture_map:
                        key = self.gesture_map[gesture]
                        self.send_service_request(key)
                        self.get_logger().info(f"Sent key: {key} for gesture: {gesture}")
        except KeyboardInterrupt:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    controller = GestureController()
    controller.read_gestures()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()