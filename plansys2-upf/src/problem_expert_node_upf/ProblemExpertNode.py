import rclpy
import rclpy.lifecycle
from rclpy.node import Node
import os
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle import TransitionCallbackReturn
import traceback
from plansys2_msgs.srv import (
    AddProblem, 
    AddProblemGoal, 
    AffectParam, 
    AffectNode, 
    GetProblemGoal, 
    GetProblemInstanceDetails, 
    GetProblemInstances, 
    GetNodeDetails, 
    GetStates, 
    GetProblem, 
    IsProblemGoalSatisfied, 
    RemoveProblemGoal, 
    ClearProblemKnowledge, 
    AffectParam, 
    AffectNode, 
    ExistNode, 
    ValidateDomain
)

from plansys2_msgs.msg import Knowledge
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile
from .ProblemUPFReader import ProblemUPFExpert
from domain_expert_node_upf.DomainUPFReader import DomainUPFReader

class ProblemUPFExpertNode(LifecycleNode):
    def __init__(self):
        super().__init__('problem_expert')

        # Declaración de parámetros
        self.declare_parameter('model_file', '')
        self.declare_parameter('problem_file', '')

        self.validate_problem_callback_group = ReentrantCallbackGroup()

        # Services
        self.add_problem_service = self.create_service(
            AddProblem,
            'problem_expert/add_problem',
            self.add_problem_service_callback,
            callback_group=self.validate_problem_callback_group  # Ajusta si usas grupos diferentes
        )

        self.add_problem_goal_service = self.create_service(
            AddProblemGoal,
            'problem_expert/add_problem_goal',
            self.add_problem_goal_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.add_problem_instance_service = self.create_service(
            AffectParam,
            'problem_expert/add_problem_instance',
            self.add_problem_instance_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.add_problem_predicate_service = self.create_service(
            AffectNode,
            'problem_expert/add_problem_predicate',
            self.add_problem_predicate_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.add_problem_function_service = self.create_service(
            AffectNode,
            'problem_expert/add_problem_function',
            self.add_problem_function_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_goal_service = self.create_service(
            GetProblemGoal,
            'problem_expert/get_problem_goal',
            self.get_problem_goal_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_instance_details_service = self.create_service(
            GetProblemInstanceDetails,
            'problem_expert/get_problem_instance',
            self.get_problem_instance_details_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_instances_service = self.create_service(
            GetProblemInstances,
            'problem_expert/get_problem_instances',
            self.get_problem_instances_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_predicate_details_service = self.create_service(
            GetNodeDetails,
            'problem_expert/get_problem_predicate',
            self.get_problem_predicate_details_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_predicates_service = self.create_service(
            GetStates,
            'problem_expert/get_problem_predicates',
            self.get_problem_predicates_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_function_details_service = self.create_service(
            GetNodeDetails,
            'problem_expert/get_problem_function',
            self.get_problem_function_details_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_functions_service = self.create_service(
            GetStates,
            'problem_expert/get_problem_functions',
            self.get_problem_functions_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.get_problem_service = self.create_service(
            GetProblem,
            'problem_expert/get_problem',
            self.get_problem_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.is_problem_goal_satisfied_service = self.create_service(
            IsProblemGoalSatisfied,
            'problem_expert/is_problem_goal_satisfied',
            self.is_problem_goal_satisfied_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.remove_problem_goal_service = self.create_service(
            RemoveProblemGoal,
            'problem_expert/remove_problem_goal',
            self.remove_problem_goal_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.clear_problem_knowledge_service = self.create_service(
            ClearProblemKnowledge,
            'problem_expert/clear_problem_knowledge',
            self.clear_problem_knowledge_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.remove_problem_instance_service = self.create_service(
            AffectParam,
            'problem_expert/remove_problem_instance',
            self.remove_problem_instance_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.remove_problem_predicate_service = self.create_service(
            AffectNode,
            'problem_expert/remove_problem_predicate',
            self.remove_problem_predicate_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.remove_problem_function_service = self.create_service(
            AffectNode,
            'problem_expert/remove_problem_function',
            self.remove_problem_function_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.exist_problem_predicate_service = self.create_service(
            ExistNode,
            'problem_expert/exist_problem_predicate',
            self.exist_problem_predicate_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.exist_problem_function_service = self.create_service(
            ExistNode,
            'problem_expert/exist_problem_function',
            self.exist_problem_function_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        self.update_problem_function_service = self.create_service(
            AffectNode,
            'problem_expert/update_problem_function',
            self.update_problem_function_service_callback,
            callback_group=self.validate_problem_callback_group
        )

        # Publishers
        self.update_pub = self.create_publisher(
            Empty,
            'problem_expert/update_notify',
            100  # QoS depth
        )

        self.knowledge_pub = self.create_publisher(
            Knowledge,
            'problem_expert/knowledge',
            rclpy.qos.QoSProfile(
                depth=100,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
            )
        )

    def on_configure(self, state):
        self.get_logger().info(">>> ENTERING on_configure() <<<")
        try:
            
            model_file = self.get_parameter('model_file').value

            if not model_file:
                self.get_logger().error("Parameter 'model_file' is not set. Please provide PDDL domain file(s).")
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f"Loading model_file: {model_file}")

            model_paths = model_file.split(":")

            if not model_paths:
                self.get_logger().error('No model file specified')
                return TransitionCallbackReturn.ERROR
            
            with open(model_paths[0], 'r') as f:
                domain_first_str = f.read()

            self.domain_expert = DomainUPFReader()

            self.domain_expert.load_pddl(model_paths[0])
            print('añade el pddl')

            self.problem_expert = ProblemUPFExpert(model_paths[0])
            
            problem_file = self.get_parameter('problem_file').value
            
            if problem_file:
                with open(problem_file, 'r') as f:
                    problem_str = f.read()
                self.problem_expert.add_problem(problem_str)
            
            self.get_logger().info(f"[{self.get_name()}] Configured")
            return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f"CONFIGURE ERROR: {e}")
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.ERROR
    
    def on_activate(self, state):
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Activating...")
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Deactivating...")
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Deactivated")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state):
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Cleaning up...")
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Cleaned up")
        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state):
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Shutting down...")
        self.get_logger().info(f"[{self.domain_expert.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        self.get_logger().error(f"[{self.domain_expert.get_name()}] Error transition")
        return TransitionCallbackReturn.SUCCESS
    
#-----------------ADD SERVICES-----------------------

    def add_problem_service_callback(self, request, response):
        if self.problem_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            self.get_logger().info(f'Adding problem: \n{str(request.problem)}')
            response.success = self.problem_expert.add_problem(request.problem)
        
        if response.success:
            self.update_pub.publish(Empty())
            self.knowledge_pub.publish(self.get_knowledge_as_msg())
        
        return response
    
    def add_problem_goal_service_callback(self, request, response):
        if self.problem_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else: 
            self.get_logger().info(f'Adding goal: \n{str(request.tree)}')
            response.success = self.problem_expert.add_problem_goal(request.tree)
        
        if response.success:
            self.update_pub.publish(Empty())
            self.knowledge_pub.publish(self.problem_expert.get_knowledge_as_msg())
        
        return response

    def add_problem_instance_service_callback(self, request, response):
        if self.problem_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            self.get_logger().info(f'Adding instance: \n{str(request.param)}')
            response.success = self.problem_expert.add_instance(request.param)

        if response.success:
            self.update_pub.publish(Empty())
            self.knowledge_pub.publish(self.get_knowledge_as_msg())

        return response
    
    def add_problem_predicate_service_callback(self, request, response):
        pass

    def add_problem_function_service_callback(self, request, response):
        pass
#------------------GET SERVICES-----------------------

    def get_problem_goal_service_callback(self, request, response):
        if self.problem_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.goal = str(self.problem_expert.get_goal())
        
        return response # comprobar si debería devolver algo
    
    def get_problem_instance_details_service_callback(self, request, response):
        pass

    def get_problem_instances_service_callback(self, request, response):
        if self.problem_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)
        
        else:
            response.success = True
            response.instances = self.problem_expert.get_instances()

        return response

    def get_problem_predicate_details_service_callback(self, request, response):
        pass

    def get_problem_predicates_service_callback(self, request, response):
        if self.problem_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            response.predicates = self.problem_expert.get_predicates()

        return response


    def get_problem_function_details_service_callback(self, request, response):
        pass

    def get_problem_functions_service_callback(self, request, response):
        if self.problem_expert is None:
            response.success = False
            response.error_info = "Requesting service in non-active state"
            self.get_logger().warn(response.error_info)

        else:
            response.success = True
            response.functions = self.problem_expert.get_functions()

        return response

    def get_problem_service_callback(self, request, response):
        pass

    def is_problem_goal_satisfied_service_callback(self, request, response):
        pass

#--------------REMOVE/CLEAR SERVICES------------------

    def remove_problem_goal_service_callback(self, request, response):
        pass
    
    def clear_problem_knowledge_service_callback(self, request, response):
        pass

    def remove_problem_instance_service_callback(self, request, response):
        pass

    def remove_problem_predicate_service_callback(self, request, response):
        pass

    def remove_problem_function_service_callback(self, request, response):
        pass

    def exist_problem_predicate_service_callback(self, request, response):
        pass

    def exist_problem_function_service_callback(self, request, response):
        pass

    def update_problem_function_service_callback(self, request, response):
        pass

#------------------PUBLISHERS-------------------------

    def get_knowledge_as_msg(self):
        ret_msg = Knowledge()

        print('-----instances------')
        for instance in self.problem_expert.get_instances():
            print(instance)
            ret_msg.instances.append(str(instance))
        print('-----predicates------')
        for predicate in self.problem_expert.get_predicates():
            print(predicate)
            ret_msg.predicates.append(str(predicate))
        print('-----functions------')
        for function in self.problem_expert.get_functions():
            print(function)
            ret_msg.functions.append(str(function))
        print('-----goal------')
        goal = self.problem_expert.get_goal()
        print('llega')
        if goal:
            ret_msg.goal = str(goal)
        print('llega 2')
        return ret_msg