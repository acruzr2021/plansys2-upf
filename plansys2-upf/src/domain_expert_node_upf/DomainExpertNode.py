import rclpy
from rclpy.node import Node
import os
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import *
from rclpy.lifecycle import TransitionCallbackReturn
import traceback
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
from rclpy.qos import QoSProfile
from domain_expert_node_upf.DomainUPFReader import DomainUPFReader

#from plansys2_domain_expert.domain_expert_node import DomainExpertNode
#from plansys2_popf_plan_solver.popf_plan_solver import PopfPlanSolver

#nodename=

class DomainUPFExpertNode(LifecycleNode):
    def __init__(self):
        super().__init__('domain_expert')

        # Declaración de parámetros
        self.declare_parameter('model_file', '')
        self.declare_parameter('validate_using_planner_node', False)

        self.validate_domain_callback_group = ReentrantCallbackGroup()

        self.get_name_service = self.create_service(
            GetDomainName,                                     # tipo de mensaje
            'domain_expert/get_domain_name',                   # tipo de servicio
            self.get_domain_name_service_callback,             # nombre del servicio
            callback_group=self.validate_domain_callback_group # función callback
        )

        self.get_types_service = self.create_service(
            GetDomainTypes,
            'domain_expert/get_domain_types',
            self.get_domain_types_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_actions_service = self.create_service(
            GetDomainActions,
            'domain_expert/get_domain_actions',
            self.get_domain_actions_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_action_details_service = self.create_service(
            GetDomainActionDetails,
            'domain_expert/get_domain_action_details',
            self.get_domain_action_details_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_durative_actions_service = self.create_service(
            GetDomainActions,
            'domain_expert/get_domain_durative_actions',
            self.get_domain_durative_actions_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_durative_action_details_service = self.create_service(
            GetDomainDurativeActionDetails,
            'domain_expert/get_domain_durative_action_details',
            self.get_domain_durative_action_details_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_predicates_service = self.create_service(
            GetStates,
            'domain_expert/get_domain_predicates',
            self.get_domain_predicates_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_predicate_details_service = self.create_service(
            GetNodeDetails,
            'domain_expert/get_domain_predicate_details',
            self.get_domain_predicate_details_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_functions_service = self.create_service(
            GetStates,
            'domain_expert/get_domain_functions',
            self.get_domain_functions_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_function_details_service = self.create_service(
            GetNodeDetails,
            'domain_expert/get_domain_function_details',
            self.get_domain_function_details_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_derived_predicates_service = self.create_service(
            GetStates,
            'domain_expert/get_domain_derived_predicates',
            self.get_domain_derived_predicates_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_derived_predicate_details_service = self.create_service(
            GetDomainDerivedPredicateDetails,
            'domain_expert/get_domain_derived_predicate_details',
            self.get_domain_derived_predicate_details_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.get_domain_service = self.create_service(
            GetDomain,
            'domain_expert/get_domain',
            self.get_domain_service_callback,
            callback_group=self.validate_domain_callback_group
        )

        self.validate_domain_client = self.create_client(ValidateDomain, 'planner/validate_domain')

        # self.domain_pub_ = LifecyclePublisher(String, 'domain', qos_profile=10)

        self.domain_pub_ = self.create_lifecycle_publisher(
            String,
            'domain',
            QoSProfile(depth=10)
        )

        self.popf_plan_solver = None
        self.domain_upf = DomainUPFReader(self.get_logger())

    def on_configure(self, state):
        self.get_logger().info(">>> ENTERING on_configure() <<<")
        try:
            
            model_file = self.get_parameter('model_file').value

            if not model_file:
                self.get_logger().error("Parameter 'model_file' is not set. Please provide PDDL domain file(s).")
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f"Loading model_file: {model_file}")

            model_paths = model_file.split(":")

            for path in model_paths:
                self.get_logger().info(f"Loading model_file: {path}")

                loaded = self.domain_upf.load_pddl(path)
                if not loaded:
                    self.get_logger().error(f"Error loading model_file: {path}")
                    continue
            
            self.get_logger().info(">>> CONFIGURE SUCCESS <<<")
            self.get_logger().info(f"[{self.domain_upf.get_name()}] Configured")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"CONFIGURE ERROR: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR
    
    def on_activate(self, state):
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Activating...")
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Deactivating...")
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Deactivated")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Cleaning up...")
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Cleaned up")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state):
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Shutting down...")
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        self.get_logger().error(f"[{self.domain_upf.get_name()}] Error transition")
        return TransitionCallbackReturn.SUCCESS

    def get_domain_name_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.name = self.domain_upf.get_name()

        return response

    def get_domain_types_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.types = self.domain_upf.get_types()

        return response
        

    def get_domain_actions_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.actions = self.domain_upf.get_actions()
        return response
        
    def get_domain_action_details_service_callback(self, request, response): #??
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            action = self.domain_upf.get_action(request.action, request.parameters)
            if action is not None:
                response.action = action
                response.success = True
            
            else:
                self.get_logger().warn(f"Requesting a non-existing action [{request.action}]")
                response.success = False
                response.error_info = "Action not found"
        return response

    def get_domain_durative_actions_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.actions = self.domain_upf.get_durative_actions()
        return response

    def get_domain_durative_action_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            action = self.domain_upf.get_durative_action(request.durative_action, request.parameters)

            if action:
                response.durative_action = action
                response.success = True
            else:
                self.get_logger().warn(f"Requesting a non-existing action [{request.durative_action}]")
                response.success = False
                response.error_info = "Durative action not found"
            return response

    def get_domain_predicates_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            predicates = self.domain_upf.get_predicates()
            response.states = predicates
        
        return response

    def get_domain_predicate_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            predicate = self.domain_upf.get_predicate(request.expression)
            
            if predicate:
                response.node = predicate
                response.success = True
            else:
                self.get_logger(f"Requesting a non-existing predicate [{str(request.expression)}]")
                response.success = False
                response.error_info = "Predicate not found"
        return response

    def get_domain_functions_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            functions = self.domain_upf.get_functions()
            response.states = functions

        return response

    def get_domain_function_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            function = self.domain_upf.get_function(request.expression)

            if function:
                response.node = function
                response.success = True
            else: 
                self.get_logger(f"Requesting a non-existing function [{str(request.expression)}]")
                response.success = False
                response.error_info = "Function not found"
        
        return response

    def get_domain_derived_predicates_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            predicates = self.domain_expert_.getDerivedPredicates()
            response.states = [self._convert_predicates_to_node(p) for p in predicates] #??

        return response
    
    def get_domain_derived_predicate_details_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            predicates = self.domain_expert.getDerivedPredicate(request.predicate)
            
            if predicates.size() > 0:
                response.predicates = predicates
                response.success = True
            else:
                self.get_logger(f"Requesting a non-existing derived predicate [{str(request.predicate)}]")
                response.success = False
                response.error_info = "Derived predicate not found"

        return response

    def get_domain_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            response.domain = str(self.domain_upf.get_domain())
        return response

