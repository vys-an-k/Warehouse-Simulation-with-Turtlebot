#usr/bin/env python3
import os
import tf2_ros
import time
from rclpy.duration import Duration
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
import rclpy
from rclpy.node import Node
from warehouse_interfaces.srv import AssignTask, TaskCompleted
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from functools import partial
from tf2_ros import Buffer
from tf2_ros import TransformListener
from rclpy.time import Time
from action_msgs.msg import GoalStatus


class RobotExecutorNode(Node):
    def __init__(self):
        super().__init__("robot_executor")

        self.robot_id_ = self.declare_parameter("robot_id", 0).value

        self.assign_client_ = self.create_client(AssignTask, "/assign_task") 
        self.get_logger().info("Task assignment client has started")

        self.task_client_ = self.create_client(TaskCompleted, "/task_completed")
        self.get_logger().info("Task completion client has started")

        robot_ns = f"/robot_{self.robot_id_}"
        action_name = robot_ns + '/navigate_to_pose'
        self.nav_client_ = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info("Navigation action client has started")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._startup_timer = self.create_timer(15.0, self._start_task_loop)
        self.current_task_ = None

    def _start_task_loop(self):
        self.get_logger().info("✅ Nav2 should now be active; starting task loop")
        # every 2s, ask for a task (but only once Nav2 is up)
        self.create_timer(2.0, self.request_task_callback)
        self._startup_timer.cancel()

    def request_task_callback(self):
        if self.current_task_ is not None:
            return  # Already doing something

        req = AssignTask.Request()
        req.robot_id = self.robot_id_

        if not self.assign_client_.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Task assignment service not available.")
            return

        self.assign_client_.call_async(req).add_done_callback(self.handle_assignment)

    def handle_assignment(self, future):
        response = future.result()
        if not response.success:
            self.get_logger().info("No task assigned. Retrying soon ...")
            return

        self.current_task_ = response.task_id
        self.get_logger().info(f"Received task {self.current_task_}")

        self.send_goal(response.shelf, next_phase="to_drop", drop=response.drop)

    def send_goal(self, point: Point, next_phase=None, drop=None):
        self._pending_goal = (point, next_phase, drop)
        self.get_logger().info(f"[robot_{self.robot_id_}] Waiting for NavigateToPose action server…")
        while rclpy.ok() and not self.nav_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("  …action server not available, retrying in 1s")
            time.sleep(1.0)

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        now = self.get_clock().now()
        ps.header.stamp = now.to_msg()
        ps.pose.position = point
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        goal.pose = ps

        self.get_logger().info(f"Sending goal: x={point.x}, y={point.y}")

        #self.nav_client_.wait_for_server()

        send_goal_future = self.nav_client_.send_goal_async(goal)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, next_phase, drop)
        )

    def goal_response_callback(self, future, next_phase, drop):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal rejected")
            self.get_logger().warn("Navigation goal rejected — retrying in 2s")
            self.create_timer(2.0, self._retry_pending_goal)
            return

        self.get_logger().info("Navigation goal accepted")
        time.sleep(0.1)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.nav_done_callback(future, next_phase, drop)
        )

    def nav_done_callback(self, future, next_phase, drop):
        result_response = future.result()
        status = result_response.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation successful")
            self._pending_goal = None
            if next_phase == "to_drop":
                self.send_goal(drop, next_phase="to_notify")
            else:
                self.notify_task_done()
        else:
            self.get_logger().warn("Navigation failed — retrying in 2s")
            #self.create_timer(2.0, self._retry_pending_goal)

    def _retry_pending_goal(self):
        """Timer callback: re-send the last pending goal, if any."""
        if self._pending_goal is None:
            return
        point, next_phase, drop = self._pending_goal
        self.get_logger().info(f"Retrying goal to x={point.x}, y={point.y}")
        self.send_goal(point, next_phase=next_phase, drop=drop)

    def notify_task_done(self):
        req = TaskCompleted.Request()
        req.robot_id = self.robot_id_
        req.task_id = self.current_task_

        if self.task_client_.service_is_ready():
            self.task_client_.call_async(req)
            self.get_logger().info(f"Task {self.current_task_} completed. Notified TaskManager.")
        else:
            self.get_logger().warn("Task completion service not available")

        self.current_task_ = None

def main(args=None):
    rclpy.init(args=args)
    node = RobotExecutorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
