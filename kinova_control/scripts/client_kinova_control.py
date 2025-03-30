#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from kinova_control.srv import KeyTrigger
import pygame
import sys

class MoveItClient(Node):
    def __init__(self):
        super().__init__('moveit_client')
        self.cli = self.create_client(KeyTrigger, 'key_trigger_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Pygame setup with smaller window
        pygame.init()
        self.screen = pygame.display.set_mode((500, 400))
        pygame.display.set_caption("Kinova Control Interface")
        self.font = pygame.font.Font(None, 22)
        self.header_font = pygame.font.Font(None, 25)
        self.color_white = (255, 255, 255)
        self.color_green = (50, 205, 50)
        self.background = (30, 30, 30)

        # Simplified instructional text
        self.instructions = [
            "Kinova Gen3 Control",
            "",
            "Gesture Controls:",
            "Up-Down          : Home Position", # e
            "Left-Right       : Place", # f
            "Forward-Backward : Pickup", # t
            "Rectangle        : Top-Right", # g
            "Flat Rect        : Bottom-Right", # h
            "Circle           : Random" # v
        ]

    def draw_display(self):
        self.screen.fill(self.background)
        
        # Draw header
        header = self.header_font.render("Kinova Control Interface", True, self.color_green)
        self.screen.blit(header, (20, 20))

        # Draw instructions
        y_offset = 60
        for line in self.instructions:
            if "Controls:" in line:
                text = self.font.render(line, True, self.color_green)
            else:
                text = self.font.render(line, True, self.color_white)
            
            self.screen.blit(text, (30, y_offset))
            y_offset += 25

        pygame.display.flip()

    def send_key(self, key):
        req = KeyTrigger.Request()
        req.key_pressed = key
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = MoveItClient()
    
    try:
        while True:
            client.draw_display()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.KEYDOWN:
                    key = pygame.key.name(event.key)
                    client.get_logger().info(f"Sending key '{key}' to service")
                    response = client.send_key(key)
                    if response.success:
                        client.get_logger().info(f"Motion for key '{key}' executed successfully")
                    else:
                        client.get_logger().error(f"Planning failed for key '{key}'")
    except KeyboardInterrupt:
        pass
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()