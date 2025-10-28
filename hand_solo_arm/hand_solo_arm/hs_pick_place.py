#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

# -----------------------
# Helper functions
# -----------------------
def create_goal(joint_names: List[str], joints: List[float], planning_group: str) -> MoveGroup.Goal:
    """Convert a list of joint angles into a MoveGroup goal."""
    goal = MoveGroup.Goal()
    req = MotionPlanRequest()
    req.group_name = planning_group
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.5
    req.max_acceleration_scaling_factor = 0.5

    cs = Constraints()
    for name, val in zip(joint_names, joints):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = val
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        cs.joint_constraints.append(jc)

    req.goal_constraints.append(cs)
    goal.request = req
    goal.planning_options.plan_only = False
    return goal

# -----------------------
# Main function
# -----------------------
def main():
    rclpy.init()
    node = rclpy.create_node('go_to_articulation')  # node initialization

    JOINT_NAMES = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']
    PLANNING_GROUP = 'tmr_arm'
    ACTION_NAME = '/move_action'
    client = ActionClient(node, MoveGroup, ACTION_NAME)

    node.get_logger().info("Waiting for MoveGroup action server")
    client.wait_for_server()

    # Poses
    POSES = {
        "static": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "carry": [0.349066, -1.178097263394773, 2.356, 0.0, 0.0, 0.0],
        "place": [0.0, 0.785, 0.785, 0.0, 0.0, 0.0],
        "reach": [0.0, 0.698132, 1.48353, 0.0, 0.0, 0.0]
    }

    # -----------------------
    # Asynchronous send_and_wait
    # -----------------------
    def send_and_wait(joints: List[float]) -> None:
        goal = create_goal(JOINT_NAMES, joints, PLANNING_GROUP)
        send_fut = client.send_goal_async(goal)

        def goal_response_callback(fut):
            handle = fut.result()
            if not handle.accepted:
                node.get_logger().error('Goal rejected')
                return

            res_fut = handle.get_result_async()

            def result_callback(res_fut_inner):
                res = res_fut_inner.result()
                if res.status == 3:  # SUCCEEDED
                    node.get_logger().info('Pose executed successfully')
                else:
                    node.get_logger().error(f'Pose failed with status {res.status}')

            res_fut.add_done_callback(result_callback)

        send_fut.add_done_callback(goal_response_callback)

    # -----------------------
    # Subscriber callback
    # -----------------------
    def articulation_callback(msg: String):
        command = msg.data.strip().lower()
        if command not in POSES:
            node.get_logger().warn(f"Unknown articulation command '{command}'")
            return
        node.get_logger().info(f"Executing pose '{command}' joints: {POSES[command]}")
        send_and_wait(POSES[command])

    # Subscriber: listens to bowie_articulation
    node.create_subscription(String, 'bowie_articulation', articulation_callback, 10)
    node.get_logger().info("Listening for /bowie_articulation commands...")

    rclpy.spin(node)
    node.destroy_node()  # node shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()

