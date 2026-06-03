import sys
import rclpy
import lifecycle_msgs
from rclpy.node import Node
from rclpy.lifecycle.node import LifecycleNode
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String
from plansys2_upf.planner_upf.PlannerUPFNode import PlannerUPFNode

def main(args =None):
    rclpy.init(args = args)
    node = PlannerUPFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+C detected')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()