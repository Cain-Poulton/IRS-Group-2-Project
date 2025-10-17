#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re

class BoxToBowiePublisher(Node):
    def __init__(self):
        super().__init__('box_to_bowie_publisher')

        # Subscriber: listens to the PLC / sensor topic
        self.sub_status = self.create_subscription(
            String,
            'hmi/unified_status',
            self.status_callback,
            10
        )

        # Publisher: tells the nav node where to move
        self.pub_bowie = self.create_publisher(String, 'bowie_position', 10)

        # Track last published location to avoid duplicates
        self.last_location = None

        self.get_logger().info("🤖 Box-to-Bowie node started. Listening to /hmi/unified_status...")

    def status_callback(self, msg):
        try:
            # Parse JSON safely
            data = json.loads(msg.data)
            location_raw = data.get("box", {}).get("location", "")

            # Clean up (remove colons, spaces, etc.)
            location = re.sub(r'[^A-Za-z]', '', location_raw).upper()

            if location not in ["A", "B", "C"]:
                self.get_logger().warn(f"Unknown box location '{location_raw}' (cleaned='{location}')")
                return

            # Publish only if the location changed
            if location != self.last_location:
                self.last_location = location
                msg_out = String()
                msg_out.data = location
                self.pub_bowie.publish(msg_out)
                self.get_logger().info(f"📦 Box at {location} → publishing to /bowie_position")
            else:
                self.get_logger().debug(f"Box still at {location}, no republish.")

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received.")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BoxToBowiePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
