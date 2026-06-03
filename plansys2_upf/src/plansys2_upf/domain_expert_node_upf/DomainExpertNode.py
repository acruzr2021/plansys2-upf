import rclpy
import traceback
from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from std_msgs.msg import String
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
    ValidateDomain,
)
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from plansys2_upf.domain_expert_node_upf.DomainUPFReader import DomainUPFReader


class DomainUPFExpertNode(Node):
    def __init__(self):
        """Lifecycle node that exposes DomainUPFReader as ROS 2 services."""
        super().__init__('domain_expert')

        self.declare_parameter('model_file', '')
        self.declare_parameter('validate_using_planner_node', False)

        self.domain_upf = DomainUPFReader(self.get_logger())

        self.validate_domain_callback_group = ReentrantCallbackGroup()

        self.get_state_service = self.create_service(
            GetState,
            'domain_expert/get_state',
            self.__on_get_state,
        )

        self.get_name_service = self.create_service(
            GetDomainName,
            'domain_expert/get_domain_name',
            self.get_domain_name_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_types_service = self.create_service(
            GetDomainTypes,
            'domain_expert/get_domain_types',
            self.get_domain_types_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_actions_service = self.create_service(
            GetDomainActions,
            'domain_expert/get_domain_actions',
            self.get_domain_actions_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_action_details_service = self.create_service(
            GetDomainActionDetails,
            'domain_expert/get_domain_action_details',
            self.get_domain_action_details_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_durative_actions_service = self.create_service(
            GetDomainActions,
            'domain_expert/get_domain_durative_actions',
            self.get_domain_durative_actions_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_durative_action_details_service = self.create_service(
            GetDomainDurativeActionDetails,
            'domain_expert/get_domain_durative_action_details',
            self.get_domain_durative_action_details_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_predicates_service = self.create_service(
            GetStates,
            'domain_expert/get_domain_predicates',
            self.get_domain_predicates_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_predicate_details_service = self.create_service(
            GetNodeDetails,
            'domain_expert/get_domain_predicate_details',
            self.get_domain_predicate_details_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_functions_service = self.create_service(
            GetStates,
            'domain_expert/get_domain_functions',
            self.get_domain_functions_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_function_details_service = self.create_service(
            GetNodeDetails,
            'domain_expert/get_domain_function_details',
            self.get_domain_function_details_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_derived_predicates_service = self.create_service(
            GetStates,
            'domain_expert/get_domain_derived_predicates',
            self.get_domain_derived_predicates_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_derived_predicate_details_service = self.create_service(
            GetDomainDerivedPredicateDetails,
            'domain_expert/get_domain_derived_predicate_details',
            self.get_domain_derived_predicate_details_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.get_domain_service = self.create_service(
            GetDomain,
            'domain_expert/get_domain',
            self.get_domain_service_callback,
            callback_group=self.validate_domain_callback_group,
        )

        self.validate_domain_client = self.create_client(ValidateDomain, 'planner/validate_domain')

        self.domain_pub_ = self.create_lifecycle_publisher(
            String,
            'domain',
            QoSProfile(depth=10),
        )

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Loads the PDDL domain from the model_file parameter.
        Supports multiple files separated by ':'; the first is loaded with load_pddl,
        the rest are merged with extend_domain.
        Returns FAILURE if model_file is not set or any file fails to load.
        """
        self.get_logger().info(">>> ENTERING on_configure() <<<")
        try:
            model_file = self.get_parameter('model_file').value
            if not model_file:
                self.get_logger().error(
                    "Parameter 'model_file' is not set. Please provide PDDL domain file(s)."
                )
                return TransitionCallbackReturn.FAILURE

            self.get_logger().info(f"Loading model_file: {model_file}")
            for i, path in enumerate(model_file.split(":")):
                self.get_logger().info(f"Loading model file: {path}")
                loader = self.domain_upf.load_pddl if i == 0 else self.domain_upf.extend_domain
                if not loader(path):
                    self.get_logger().error(f"Error loading model file: {path}")
                    return TransitionCallbackReturn.FAILURE

            self.get_logger().info(">>> CONFIGURE SUCCESS <<<")
            self.get_logger().info(f"[{self.domain_upf.get_name()}] Configured")
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"CONFIGURE ERROR: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state):
        """
        Activates the node and publishes the domain PDDL to the 'domain' topic.
        The LifecyclePublisher is automatically deactivated by the framework on deactivation.
        """
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Activating...")
        result = super().on_activate(state)
        msg = String()
        msg.data = str(self.domain_upf.get_domain())
        self.domain_pub_.publish(msg)
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Activated")
        return result

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
        self.get_logger().info(f"[{self.domain_upf.get_name()}] Shut down")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        self.get_logger().error(f"[{self.domain_upf.get_name()}] Error transition")
        return TransitionCallbackReturn.SUCCESS

    def get_domain_name_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        response.success = True
        response.name = self.domain_upf.get_name()
        return response

    def get_domain_types_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        response.success = True
        response.types = self.domain_upf.get_types()
        return response

    def get_domain_actions_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        response.success = True
        response.actions = self.domain_upf.get_actions()
        return response

    def get_domain_action_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
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
            return response
        response.success = True
        response.actions = self.domain_upf.get_durative_actions()
        return response

    def get_domain_durative_action_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        action = self.domain_upf.get_durative_action(request.durative_action, request.parameters)
        if action:
            response.durative_action = action
            response.success = True
        else:
            self.get_logger().warn(
                f"Requesting a non-existing action [{request.durative_action}]"
            )
            response.success = False
            response.error_info = "Durative action not found"
        return response

    def get_domain_predicates_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        response.success = True
        response.states = self.domain_upf.get_predicates()
        return response

    def get_domain_predicate_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        predicate = self.domain_upf.get_predicate(request.expression)
        if predicate:
            response.node = predicate
            response.success = True
        else:
            self.get_logger().warn(
                f"Requesting a non-existing predicate [{request.expression}]"
            )
            response.success = False
            response.error_info = "Predicate not found"
        return response

    def get_domain_functions_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        response.success = True
        response.states = self.domain_upf.get_functions()
        return response

    def get_domain_function_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        function = self.domain_upf.get_function(request.expression)
        if function:
            response.node = function
            response.success = True
        else:
            self.get_logger().warn(
                f"Requesting a non-existing function [{request.expression}]"
            )
            response.success = False
            response.error_info = "Function not found"
        return response

    def get_domain_derived_predicates_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        response.success = True
        predicates = self.domain_expert_.getDerivedPredicates()
        response.states = [self._convert_predicates_to_node(p) for p in predicates]
        return response

    def get_domain_derived_predicate_details_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        predicates = self.domain_expert.getDerivedPredicate(request.predicate)
        if predicates.size() > 0:
            response.predicates = predicates
            response.success = True
        else:
            self.get_logger().warn(
                f"Requesting a non-existing derived predicate [{request.predicate}]"
            )
            response.success = False
            response.error_info = "Derived predicate not found"
        return response

    def get_domain_service_callback(self, request, response):
        if self.domain_upf is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
            return response
        response.success = True
        response.domain = str(self.domain_upf.get_domain())
        return response

    def __on_get_state(self, request, response):
        state_id, state_label = self._state_machine.current_state
        response.current_state = State()
        response.current_state.id = state_id
        response.current_state.label = state_label
        return response

    def trigger_transition(self, transition_id: int):
        return self._state_machine.trigger_transition_by_id(transition_id, True)
