import rclpy
from rclpy.node import Node
import os
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
#from plansys2_msgs.msg import GetDomainName, GetDomainTypes, GetDomainActions, GetDomainActionDetails, GetDomainDurativeActions
from plansys2_msgs.msg import (
    GetDomainName,
    GetDomainTypes,
    GetDomainActions,
    GetDomainActionDetails,
    GetDomainDurativeActions,
    GetDomainDurativeActionDetails,
    GetStates,
    GetNodeDetails,
    GetStates,
    GetNodeDetails,
    GetStates,
    GetDomainDerivedPredicateDetails,
    GetDomain,
    ValidateDomain
)

from plansys2_domain_expert.domain_expert_node import DomainExpertNode
from plansys2_popf_plan_solver.popf_plan_solver import PopfPlanSolver

#nodename=

class DomainExpertNode(LifecycleNode):
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
            GetDomainDurativeActions,
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

    def on_configure(self):
        self.get_logger().info(f"[{self.get_name()}] Configuring...")
        
        model_file = self.get_parameter('model_file').value
        validate_using_planner_node = self.get_parameter('validate_using_planner_node').value
        
        model_files = model_file.split(":")

        if validate_using_planner_node:
            self.validate_domain_client = self.create_client(ValidateDomain, 'planner/validate_domain')

            while not self.validate_domain_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info("Waiting for planner/validate_domain service...")

        else:
            self.popf_plan_solver = POPFPlanSolver()
            self.popf_plan_solver.configure(self, "POPF")

        for i, model_file in enumerate(model_files):
            with open(model_file, 'r') as file:
                domain_str = file.read()

            if i == 0:
                self.domain_expert = DomainExpert(domain_str)
            else:
                self.domain_expert.extend_domain(domain_str)

            check_valid = True
            if validate_using_planner_node:
                request = ValidateDomain.Request()
                request.domain = self.domain_expert.get_domain()
                
                future = self.validate_domain_client.call_async(request)
                rclpy.spin_until_future_complete(self, future)
                
                if not future.done():
                    self.get_logger().error("Timed out waiting for service: planner/validate_domain")
                    return False
                
                check_valid = future.result().sucess
            else:
                check_valid = self.popf_plan_solver.is_domain_valid(self.domain_expert.get_domain(), self.get_namespace())

            if not check_valid:
                self.get_logger().error("PDDL syntax error")
                return False

        self.get_logger().info(f"[{self.get_name()}] Configured")
        return True
    
    def on_activate(self, state):
        self.get_logger().info(f"[{self.get_name()}] Activating...")
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")
        self.get_logger().info(f"[{self.get_name()}] Deactivated")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state):
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        self.get_logger().error(f"[{self.get_name}] Error transition")
        return TransitionCallbackReturn.SUCCESS

    def get_domain_name_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.name = self.domain_expert.get_name()

        return response

    def get_domain_types_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.name = self.domain_expert.get_name()

        return response
        

    def get_domain_actions_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.actions = self.domain_expert_.get_actions()
        return response
        
    def get_domain_action_details_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            action = self.domain_expert_.get_action(request.action, request.parameters)
            if action is not None:
                response.action = action
                response.success = True
            
            else:
                self.get_logger().warn(f"Requesting a non-existing action [{request.action}]")
                response.success = False
                response.error_info = "Action not found"
        return response

    def get_domain_durative_actions_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            action = self.domain_expert.getDurativeAction(request.durative_action, request.parameters)

            if action:
                response.durative_action = action
                response.success = True
            else:
                self.get_logger().warn(f"Requesting a non-existing action [{request.action}]")
                response.success = False
                response.error_info = "Durative action not found"

    def get_domain_durative_action_details_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response = True
            predicates = self.domain_expert_.get_predicates()
            response.states = [self._convert_predicate_to_node(p) for p in predicates]
        
        return response

    def get_domain_predicates_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            predicates = self.domain_expert_.get_predicates()
            response.states = [self._convert_predicate_to_node(p) for p in predicates]
        
        return response

    def get_domain_predicate_details_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            predicate = self.domain_expert.getPredicate(request.expression)
            
            if predicate:
                response.node = predicate.value() ##??
                response.success = True
            else:
                self.get_logger(f"Requesting a non-existing predicate [{str(request.expression)}]")
                response.success = False
                response.error_info = "Predicate not found"
        return response

    def get_domain_functions_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            functions = self.domain_expert_.get_predicates()
            response.states = [self._convert_function_to_node(f) for f in functions] #??

        return response

    def get_domain_function_details_service_callback(self, request, response):
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            function = self.domain_expert.getFunction(request.expression)

            if function:
                response.node = function.value()
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
        if self.domain_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            response.domain = str(self.domain_expert_.get_domain())
        return response

