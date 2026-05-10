import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive
import time
import threading


class ArmKeyPresser(Node):
    def __init__(self):
        super().__init__('arm_key_presser')
        self._cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self._cb_group)
        self.get_logger().info('Connected!')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning

    def _joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg

    def move_to_pose(self, x, y, z, ox=0.233, oy=-0.252, oz=-0.689, ow=0.639):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'world'
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = ox
        target_pose.pose.orientation.y = oy
        target_pose.pose.orientation.z = oz
        target_pose.pose.orientation.w = ow

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = 'world'
        pos_constraint.link_name = 'Wrist_Pitch'

        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.01, 0.01, 0.01]

        bv = BoundingVolume()
        bv.primitives = [bounding_box]
        bv.primitive_poses = [target_pose.pose]
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'world'
        ori_constraint.link_name = 'Wrist_Pitch'
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.5
        ori_constraint.absolute_y_axis_tolerance = 0.5
        ori_constraint.absolute_z_axis_tolerance = 0.5
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_constraint]
        # constraints.orientation_constraints = [ori_constraint]

        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        self.get_logger().info(f'Sending goal: x={x:.3f} y={y:.3f} z={z:.3f}')
        future = self._action_client.send_goal_async(goal)

        start = time.time()
        while not future.done():
            if time.time() - start > 15.0:
                self.get_logger().error('Goal send timed out!')
                return False
            time.sleep(0.05)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()

        start = time.time()
        while not result_future.done():
            if time.time() - start > 30.0:
                self.get_logger().error('Result timed out!')
                return False
            time.sleep(0.05)

        result = result_future.result()
        self.get_logger().info(f'Error code: {result.result.error_code.val}')
        return True

    def move_to_joints(self, joint_positions: list):
        joint_names = ['Base_joint', 'Bicep_Joint', 'Forearm_Joint', 'Wrist_pitch_joint']
        
        constraints = Constraints()
        for name, position in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request = MotionPlanRequest()
        request.group_name = 'arm'
        request.goal_constraints = [constraints]
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False

        self.get_logger().info(f'Moving to joints: {joint_positions}')
        future = self._action_client.send_goal_async(goal)

        start = time.time()
        while not future.done():
            if time.time() - start > 15.0:
                self.get_logger().error('Goal send timed out!')
                return False
            time.sleep(0.05)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()

        start = time.time()
        while not result_future.done():
            if time.time() - start > 30.0:
                self.get_logger().error('Result timed out!')
                return False
            time.sleep(0.05)

        result = result_future.result()
        self.get_logger().info(f'Error code: {result.result.error_code.val}')
        return True
def main(args=None):
    rclpy.init(args=args)
    node = ArmKeyPresser()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)  # give executor time to start

    node.move_to_pose(x=0.837, y=-0.337, z=0.539)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
