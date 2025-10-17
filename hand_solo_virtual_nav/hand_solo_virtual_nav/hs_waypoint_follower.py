#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# --- Helper function to build a PoseStamped ---
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x = x
    ps.pose.position.y = y
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps

def main():
    rclpy.init()
    node = rclpy.create_node('go_to_box_position')

    # Nav2 Action Client
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # --- Literal send_and_wait function ---
    def send_and_wait(pose: PoseStamped) -> bool:
        node.get_logger().info('Waiting for Nav2 action server...')
        client.wait_for_server()

        # Update timestamp (required in headers)
        pose.header.stamp = node.get_clock().now().to_msg()

        # Wrap pose in a NavigateToPose goal message
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Simple feedback callback: prints distance left to target
        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance remaining: {dist:.2f} m')
            except Exception:
                pass  # ignore if feedback doesn't have distance

        # Send the goal
        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()

        if not handle or not handle.accepted:
            node.get_logger().error('Goal was rejected!')
            return False

        # Wait until navigation is finished
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        result = result_future.result()

        if result is None:
            node.get_logger().error('No result returned.')
            return False

        node.get_logger().info('Goal reached successfully!')
        return True

    # --- Define waypoints ---
    wpA = make_pose(1.0, 2.0, 0.0)
    wpB = make_pose(1.1, 0.8, 0.0025)
    wpC = make_pose(1.9, 2.9, 0.0025)

    # --- Callback for box positions ---
    def listener_callback(msg: String):
        location = msg.data.strip()  # raw message like "A", "B", "C"
        node.get_logger().info(f"Box located at position {location}")

        if location == "A":
            node.get_logger().info("Moving to location A...")
            send_and_wait(wpA)
        elif location == "B":
            node.get_logger().info("Moving to location B...")
            send_and_wait(wpB)
        elif location == "C":
            node.get_logger().info("Moving to location C...")
            send_and_wait(wpC)
        else:
            node.get_logger().warn(f"Unknown location '{location}' — ignoring.")

    # --- Subscriber ---
    node.create_subscription(String, 'bowie_position', listener_callback, 10)
    node.get_logger().info("GoToBox node running — waiting for bowie_position updates...")

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

