#!/usr/bin/env python3
import math
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# -----------------------
# Helper functions
# -----------------------
def make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.pose.position.x = x
    ps.pose.position.y = y
    half = yaw * 0.5
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


# -----------------------
# Main function
# -----------------------
def main():
    rclpy.init()
    node = rclpy.create_node('go_to_position')  # node initialization
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    node.get_logger().info('Waiting for Nav2 action server')
    client.wait_for_server()

    # -----------------------
    # Async send_and_wait
    # -----------------------
    def send_and_wait(pose: PoseStamped) -> None:
        pose.header.stamp = node.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = pose

        def feedback_cb(fb):
            try:
                dist = fb.feedback.distance_remaining
                node.get_logger().info(f'Distance {dist:.2f} m')
            except Exception:
                pass

        send_future = client.send_goal_async(goal, feedback_callback=feedback_cb)

        def goal_response_callback(fut):
            handle = fut.result()
            if not handle.accepted:
                node.get_logger().error('Goal was rejected!')
                return

            result_future = handle.get_result_async()

            def result_callback(res_fut):
                res = res_fut.result()
                if res is None:
                    node.get_logger().error('No result returned.')
                    return
                node.get_logger().info('Goal reached successfully!')

            result_future.add_done_callback(result_callback)

        send_future.add_done_callback(goal_response_callback)

    # -----------------------
    # Waypoints
    # -----------------------
    wpD = make_pose(0.0, 0.0, 5.93412)
    wpA = make_pose(1.95, -0.85, 0.349066)
    wpB = make_pose(2.2633333333, -0.29, 5.93412)
    wpC = make_pose(2.27, -0.29, 5.93412)
    wpS = make_pose(30.5, -3.7, 4.363324)

    # -----------------------
    # Subscriber callback
    # -----------------------
    def listener_callback(msg: String):
        location = msg.data.strip().upper()
        node.get_logger().info(f"Received box position: {location}")

        if location == "DEFAULT":
            node.get_logger().info("Navigating to Default")
            send_and_wait(wpD)
        elif location == 'A':
            node.get_logger().info("Navigating to A")
            send_and_wait(wpA)
        elif location == "B":
            node.get_logger().info("Navigating to B")
            send_and_wait(wpB)
        elif location == "C":
            node.get_logger().info("Navigating to C")
            send_and_wait(wpC)
        elif location == "SHELF":
            node.get_logger().info("Navigating to Shelf")
            send_and_wait(wpS)
        else:
            node.get_logger().warn(f"Unknown location '{location}'")

    # Subscriber: listens to bowie_position
    node.create_subscription(String, 'bowie_position', listener_callback, 10)
    node.get_logger().info("GoToBox node waiting for /bowie_position messages...")

    rclpy.spin(node)
    node.destroy_node()  # node shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()

