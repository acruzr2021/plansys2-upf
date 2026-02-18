import rclpy
#from rclpy.node import Node
import os
from rclpy.lifecycle import Node, TransitionCallbackReturn, Publisher, State
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
#from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import *
#from lifecycle_msgs.msg import Transition, State
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

from lifecycle_msgs.msg import Transition, State

from rclpy.qos import QoSProfile
from plansys2_upf.domain_expert_node_upf.DomainUPFReader import DomainUPFReader
from lifecycle_msgs.srv import ChangeState, GetState

class DomainUPFExpertNode(Node):
    def __init__(self):
        super().__init__('domain_expert')

        # parameters
        self.declare_parameter('model_file', '')
        self.declare_parameter('validate_using_planner_node', False)

        self.domain_upf = DomainUPFReader(self.get_logger())

        # self._lifecycle_callbacks = {
        #     Transition.TRANSITION_CONFIGURE: [],
        #     Transition.TRANSITION_ACTIVATE: []
        # }


        self.validate_domain_callback_group = ReentrantCallbackGroup()

        # self.change_state_service = self.create_service(
        #     ChangeState,
        #     'domain_expert/change_state',
        #     self.__on_change_state
        # )

        self.get_state_service = self.create_service(
            GetState,
            'domain_expert/get_state',
            self.__on_get_state
        )

        self.get_name_service = self.create_service(
            GetDomainName,
            'domain_expert/get_domain_name',
            self.get_domain_name_service_callback,
            callback_group=self.validate_domain_callback_group
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
        

        # self.register_callback(
        #     Transition.TRANSITION_CONFIGURE,
        #     self.on_configure
        # )

        # self.register_callback(
        #     Transition.TRANSITION_ACTIVATE,
        #     self.on_activate
        # )

        self.validate_domain_client = self.create_client(ValidateDomain, 'planner/validate_domain')

        # self.domain_pub_ = LifecyclePublisher(String, 'domain', qos_profile=10)

        self.domain_pub_ = self.create_lifecycle_publisher(
            String,
            'domain',
            QoSProfile(depth=10)
        )

        self.popf_plan_solver = None

    # def on_configure(self, state):
    #     self.get_logger().info(">>> ENTERING on_configure() <<<")
    #     try:
            
    #         model_file = self.get_parameter('model_file').value

    #         if not model_file:
    #             self.get_logger().error("Parameter 'model_file' is not set. Please provide PDDL domain file(s).")
    #             return TransitionCallbackReturn.FAILURE
    #         self.get_logger().info(f"Loading model_file: {model_file}")

    #         model_paths = model_file.split(":")

    #         for path in model_paths:
    #             self.get_logger().info(f"Loading model_file: {path}")

    #             loaded = self.domain_upf.load_pddl(path)
    #             if not loaded:
    #                 self.get_logger().error(f"Error loading model_file: {path}")
    #                 continue
            
    #         self.get_logger().info(">>> CONFIGURE SUCCESS <<<")
    #         self.get_logger().info(f"[{self.domain_upf.get_name()}] Configured")
    #         return TransitionCallbackReturn.SUCCESS

    #     except Exception as e:
    #         self.get_logger().error(f"CONFIGURE ERROR: {e}")
    #         self.get_logger().error(traceback.format_exc())
    #         return TransitionCallbackReturn.ERROR
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info(">>> ENTERING on_configure() <<<")

        try:
            
            model_file = self.get_parameter('model_file').value

            if not model_file:
                self.get_logger().error(
                    "Parameter 'model_file' is not set. Please provide PDDL domain file(s)."
                )
                return TransitionCallbackReturn.FAILURE

            self.get_logger().info(f"Loading model_file: {model_file}")

            model_paths = model_file.split(":")

            first = True

            for path in model_paths:
                self.get_logger().info(f"Loading model_file: {path}")

                if first:
                    loaded = self.domain_upf.load_pddl(path)
                    if not loaded:
                        print(f"Error loading model_file: {path}")
                        return TransitionCallbackReturn.FAILURE
                    first = False
                else:
                    print('intentando extender')
                    loaded = self.domain_upf.extend_domain(path)
                    if not loaded:
                        print(f"Error loading model_file: {path}")
                        return TransitionCallbackReturn.FAILURE

                if not loaded:
                    self.get_logger().error(f"Error loading model_file: {path}")
                    return TransitionCallbackReturn.FAILURE

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
        #return TransitionCallbackReturn.SUCCESS
        return super().on_activate(state)


    def on_deactivate(self, state):
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Deactivating...")
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Deactivated")
        return super().on_deactivate(state)

    
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
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            predicates = self.domain_expert_.getDerivedPredicates()
            response.states = [self._convert_predicates_to_node(p) for p in predicates] #??

        return response
    
    def get_domain_derived_predicate_details_service_callback(self, request, response):
        if self.domain_upf is None:
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
    
    # def __on_change_state(self, request, response):
    #     #success = self.trigger_transition(request.transition.id)
    #     response.success = True
    #     return response


    def __on_get_state(self, request, response):
        # En Jazzy current_state es (id, label)
        sm_state = self._state_machine.current_state

        # Desempaquetamos la tupla
        state_id, state_label = sm_state

        # Creamos un mensaje limpio y copiamos los valores
        response.current_state = State()
        response.current_state.id = state_id
        response.current_state.label = state_label

        return response
    
    def trigger_transition(self, transition_id: int):
        return self._state_machine.trigger_transition_by_id(transition_id, True)



    # def register_callback(self, transition, callback):
    #     if transition not in self._lifecycle_callbacks:
    #         self._lifecycle_callbacks[transition] = []
    #     self._lifecycle_callbacks[transition].append(callback)

