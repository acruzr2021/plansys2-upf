import rclpy
import rclpy.lifecycle
from rclpy.node import Node
import os
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import LifecyclePublisher
from std_msgs.msg import String
from plansys2_msgs.msg import Node as TreeNode
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle import TransitionCallbackReturn
import traceback

from rclpy.lifecycle import LifecycleNode
from rclpy.duration import Duration
from plansys2_msgs.srv import GetPlan, ValidateDomain

import importlib


class PlannerUPFNode(LifecycleNode):

    def __init__(self):

        super().__init__('planner')

        self.solvers = {}

        # parámetros por defecto
        self.declare_parameter('plan_solver_plugins', ['UPF'])
        self.declare_parameter('plan_solver_timeout', 15.0)

    def on_configure(self, state):

        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        self.solver_ids = self.get_parameter(
            "plan_solver_plugins"
        ).value

        timeout = self.get_parameter(
            "plan_solver_timeout"
        ).value

        self.solver_timeout = Duration(seconds=float(timeout))

        for solver_id in self.solver_ids:

            try:

                param_name = solver_id + ".plugin"

                if not self.has_parameter(param_name):
                    self.declare_parameter(param_name, "")

                solver_type = self.get_parameter(param_name).value

                solver = self._create_instance(solver_type)

                solver.configure(self, solver_id)

                self.solvers[solver_id] = solver

                self.get_logger().info(
                    f"Created solver {solver_id} of type {solver_type}"
                )

            except Exception as ex:

                self.get_logger().error(
                    f"Failed to create solver {solver_id}: {ex}"
                )

                return TransitionCallbackReturn.FAILURE

        self.get_plan_service = self.create_service(
            GetPlan,
            "planner/get_plan",
            self.get_plan_service_callback
        )

        self.validate_domain_service = self.create_service(
            ValidateDomain,
            "planner/validate_domain",
            self.validate_domain_service_callback
        )

        self.get_logger().info(f"[{self.get_name()}] Configured")

        return TransitionCallbackReturn.SUCCESS


    def _create_instance(self, class_path):

        if class_path == "":
            raise RuntimeError("Plugin type not specified")

        module_name, class_name = class_path.rsplit(".", 1)

        print(module_name, class_name)

        module = importlib.import_module(module_name)

        print(module)


        cls = getattr(module, class_name)
        
        print(cls)


        return cls()


    def on_activate(self, state):

        self.get_logger().info(f"[{self.get_name()}] Activated")

        return TransitionCallbackReturn.SUCCESS


    def get_plan_service_callback(self, request, response):

        solver = next(iter(self.solvers.values()))

        plan = solver.get_plan(
            request.domain,
            request.problem,
            self.get_namespace(),
            self.solver_timeout
        )

        if plan is not None:

            response.success = True
            response.plan = plan

        else:

            response.success = False
            response.error_info = "Plan not found"

        return response


    def validate_domain_service_callback(self, request, response):

        response.success = True

        return response