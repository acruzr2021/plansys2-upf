import rclpy
import lifecycle_msgs
import rclpy
from rclpy.node import Node
from rclpy.lifecycle.node import LifecycleNode
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String
from domain_expert_node_upf.DomainExpertNode import DomainUPFExpertNode
from plansys2_msgs.srv import (
    GetDomainName,
    GetDomainTypes,
    GetDomainActions,
    GetDomainActionDetails,
    GetDomainDurativeActionDetails,
    GetDomainDerivedPredicateDetails,
    GetStates,
    GetNodeDetails,
    GetDomain,
    ValidateDomain
)
import sys

# class DomainUPFExpertNode(LifecycleNode):
#     def __init__(self):
#         super().__init__('my_class_node') # Replace 'my_class_node' with your node name
#         self.domain_expert_ = None  # Assuming DomainExpert is handled elsewhere

#         # Services
#         self.get_name_service_ = self.create_service(GetDomainName, 'get_domain_name', self.get_domain_name_callback)
#         self.get_types_service_ = self.create_service(GetDomainTypes, 'get_domain_types', self.get_domain_types_callback)
#         self.get_domain_actions_service_ = self.create_service(GetDomainActions, 'get_domain_actions_callback')
#         self.get_domain_action_details_service_ = self.create_service(GetDomainActionDetails, 'get_domain_action_details_callback')
#         self.get_domain_durative_actions_service_ = self.create_service(GetDomainActions, 'get_domain_durative_actions_callback') # NOTE: Using GetDomainActions, check if correct
#         self.get_domain_durative_action_details_service_ = self.create_service(GetDomainDurativeActionDetails, 'get_domain_durative_action_details_callback')
#         self.get_domain_predicates_service_ = self.create_service(GetStates, 'get_domain_predicates_callback')
#         self.get_domain_predicate_details_service_ = self.create_service(GetNodeDetails, 'get_domain_predicate_details_callback')
#         self.get_domain_functions_service_ = self.create_service(GetStates, 'get_domain_functions_callback')
#         self.get_domain_function_details_service_ = self.create_service(GetNodeDetails, 'get_domain_function_details_callback')
#         self.get_domain_derived_predicates_service_ = self.create_service(GetStates, 'get_domain_derived_predicates_callback')
#         self.get_domain_derived_predicate_details_service_ = self.create_service(GetDomainDerivedPredicateDetails, 'get_domain_derived_predicate_details_callback')
#         self.get_domain_service_ = self.create_service(GetDomain, 'get_domain', self.get_domain_callback)

#         # Client
#         self.validate_domain_client_ = self.create_client(ValidateDomain, 'validate_domain')
#         self.validate_domain_callback_group_ = self.create_callback_group()

#         # Publisher
#         self.domain_pub_ = LifecyclePublisher(String, 'domain', qos_profile=10)

#         # Planning
#         self.popf_plan_solver_ = POPFPlanSolver() 

def main(args =None):
    rclpy.init(args = args)
    node = DomainUPFExpertNode()

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