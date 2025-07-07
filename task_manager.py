#usr/bin/env python3
import os
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
import rclpy
from rclpy.node import Node
import yaml
from warehouse_interfaces.srv import AssignTask
from warehouse_interfaces.srv import TaskCompleted
from geometry_msgs.msg import Point

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager")
        #tasks_file = self.declare_parameter('tasks_file', '').value
        #with open (tasks_file, 'r') as f:
            #data = yaml.safe_load(f)
        self.tasks = [
    # id,      shelf at (2,1), drop at (3.5,4.5)
    {
      'id': 1,
    'shelf': {'x': 2.0, 'y': 0.5, 'z': 0.0},
    'drop':  {'x': 2.0, 'y': 3.5, 'z': 0.0}
    },
    # Robot 1’s task — stays up in the top aisle
]

        self.get_logger().info(f"[INLINE] Loaded tasks: {self.tasks}")

        #self.tasks = data['tasks']
        self.ongoing = {}
        self.idle = set()

        self.assign_server_ = self.create_service(AssignTask, "assign_task", self.assign_callback)
        self.get_logger().info("Task assignment server has started")
        self.task_server_ = self.create_service(TaskCompleted, "task_completed", self.complete_callback)
        self.get_logger().info("Task completion server has started")

    def assign_callback(self, request, response):
        robot = request.robot_id
        # Ensure the robot is tracked as idle if not already
        if robot not in self.ongoing and robot not in self.idle:
            self.idle.add(robot)

        # Now check if it can be assigned a task
        if not self.tasks or robot not in self.idle:
            response.success = False
            return response

        # Assign task
        task = self.tasks.pop(0)
        response.success = True
        response.task_id = task['id']
        response.shelf = Point(**task['shelf'])
        response.drop = Point(**task['drop'])

        self.ongoing[robot] = task['id']
        self.idle.remove(robot)

        self.get_logger().info(f"Assigned task {task['id']} to {robot}")
        return response


    def complete_callback(self, request, response):
        robot = request.robot_id
        tid = request.task_id
        if self.ongoing.get(robot) == tid:
            response.success = True
            self.idle.add(robot)
            del self.ongoing[robot]
        else:
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()




