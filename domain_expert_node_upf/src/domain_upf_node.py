import rclpy
from domain_upf import DomainExpertNode

def main(args =None):
    rclpy.init(args = args)
    node = DomainExpertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()