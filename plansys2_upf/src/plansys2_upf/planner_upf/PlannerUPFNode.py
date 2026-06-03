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
        """
        Initializes the planner node. Solver instances are created in on_configure,
        not here, so the node can be reconfigured without restarting.
        self.solvers: {solver_id: solver_instance} populated during configure.
        """
        super().__init__('planner',
                        allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        self.solvers = {}

        # parámetros por defecto
        # self.declare_parameter('plan_solver_plugins', ['UPF'])
        # self.declare_parameter('plan_solver_timeout', 15.0)
    def on_configure(self, state):
        """
        Reads plan_solver_plugins and plan_solver_timeout parameters, then instantiates
        each solver dynamically via _create_instance and calls solver.configure().
        Registers get_plan and validate_domain services.
        Returns FAILURE if any solver fails to load.
        """
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        self.solver_ids = self.get_parameter_or(
            "plan_solver_plugins",
            rclpy.parameter.Parameter(
                "plan_solver_plugins",
                rclpy.Parameter.Type.STRING_ARRAY,
                ["UPF"]
            )
        ).value

        timeout = self.get_parameter_or(
            "plan_solver_timeout",
            rclpy.parameter.Parameter(
                "plan_solver_timeout",
                rclpy.Parameter.Type.DOUBLE,
                40.0
            )
        ).value

        self.solver_timeout = Duration(seconds=float(timeout))

        for solver_id in self.solver_ids:

            try:
                param_name = f"{solver_id}.plugin"

                solver_type = self.get_parameter_or(
                    param_name,
                    rclpy.parameter.Parameter(
                        param_name,
                        rclpy.Parameter.Type.STRING,
                        ""
                    )
                ).value

                if solver_type == "":
                    raise RuntimeError(f"Plugin type not specified for {solver_id}")

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
        if not self.solvers:
            response.success = False
            response.error_info = "No solver configured"
            return response
        solver = next(iter(self.solvers.values()))
        response.success = solver.is_domain_valid(request.domain, self.get_namespace())
        if not response.success:
            response.error_info = "Invalid domain"
        return response