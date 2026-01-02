import rclpy
import lifecycle_msgs
import rclpy
from rclpy.node import Node
from rclpy.lifecycle.node import LifecycleNode
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String
from domain_expert_node_upf.DomainExpertNode import DomainUPFExpertNode
from problem_expert_node_upf.ProblemExpertNode import ProblemUPFExpertNode
import sys

def main(args =None):
    rclpy.init(args = args)
    node = ProblemUPFExpertNode()

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