#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re
import time

class Main_Control_Node(Node):
    def __init__(self):
        super().__init__('main_control_node')

        # Subscriber: listens to the PLC
        self.sub_status = self.create_subscription(
            String,
            'hmi/unified_status',
            self.status_callback,
            10
        )

        # Publishers
        self.pub_bowie_pos = self.create_publisher(String, 'bowie_position', 10)
        self.pub_bowie_articulation = self.create_publisher(String, 'bowie_articulation', 10)

        # Internal state
        self.current_location = None
        self.last_published_location = None

        # Main process timer
        self.create_timer(1.0, self.main_process)

        self.get_logger().info("Main_Control_Node started. Listening to /hmi/unified_status")

    # -----------------------
    # Callback: from HMI
    # -----------------------
    def status_callback(self, msg): # Reads hmi/unified_status for box location
        try:
            data = json.loads(msg.data)
            location_raw = data.get("box", {}).get("location", "")
            location = re.sub(r'[^A-Za-z]', '', location_raw).upper()

            self.current_location = location

        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON received.")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    # -----------------------
    # Main Loop
    # -----------------------
    def main_process(self):
        msg_pos = String()
        msg_art = String()

        if self.current_location == "A": # Detects box moving on conveyor
            self.get_logger().info("Box on conveyer detected. Confirming position")
            msg_art.data = "carry"
            self.pub_bowie_articulation.publish(msg_art)
            
            time.sleep(8.0)
            updated = self.current_location # Updated box position
            self.get_logger().info(f"Box confirmed at position: {updated}")
            self.last_published_location = updated

            if updated == "A": # Process for box position A
                msg_pos.data = "A"
                self.get_logger().info(f"Published positon A")
                self.pub_bowie_pos.publish(msg_pos)

            elif updated == "B": # Process for box position B
                msg_pos.data = "B"
                self.get_logger().info(f"Published positon B")
                self.pub_bowie_pos.publish(msg_pos)

            elif updated == "C": # Process for box position C
                msg_pos.data = "C"
                self.get_logger().info(f"Published positon C")
                self.pub_bowie_pos.publish(msg_pos)

            else:
                self.get_logger().warn(f"Uknown box location")

            time.sleep(12.0) # Grab box
            msg_art.data = "reach"
            self.get_logger().info(f"Published articulation Reach")
            self.pub_bowie_articulation.publish(msg_art)
            time.sleep(8.0)
            msg_art.data = "carry"
            self.get_logger().info(f"Published articulation Carry")
            self.pub_bowie_articulation.publish(msg_art)
            time.sleep(8.0)
            msg_pos.data = "shelf" # Navigate to shelf
            self.get_logger().info(f"Published position Shelf")
            self.pub_bowie_pos.publish(msg_pos)
            time.sleep(60.0)
            msg_art.data = "place" # Place on shelf
            self.get_logger().info(f"Published articulation Place")
            self.pub_bowie_articulation.publish(msg_art)
            time.sleep(10.0)
            msg_art.data = "carry" # Return to arbituary position
            self.get_logger().info(f"Published articulation Carry")
            self.pub_bowie_articulation.publish(msg_art)
            time.sleep(10.0)
            msg_pos.data = "default"
            self.get_logger().info(f"Published position Default")
            self.pub_bowie_pos.publish(msg_pos)
            time.sleep(50.0)
            self.current_location = None
            

def main(args=None): # node initialization and shut down
    rclpy.init(args=args)
    node = Main_Control_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

