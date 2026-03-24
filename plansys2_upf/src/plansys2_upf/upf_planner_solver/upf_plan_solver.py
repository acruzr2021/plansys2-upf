import os
import tempfile
import traceback
from pathlib import Path 
from typing import Optional

import unified_planning
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.shortcuts import OneshotPlanner, Compiler
from unified_planning.shortcuts import get_environment
from unified_planning.engines.results import POSITIVE_OUTCOMES
import sys
from plansys2_msgs.msg import Plan, PlanItem

from rclpy.lifecycle import LifecycleNode
from rclpy.duration import Duration

from .popf_engine import register_popf


class UPFPlanSolver:

    def __init__(self):

        self.argument_parameter_name = None
        self.output_dir_parameter_name = None
        self.lc_node = None

    def create_folders(self, node_namespace: str) -> Optional[Path]:

        output_dir = self.lc_node.get_parameter(
            self.output_dir_parameter_name
        ).value

        home_dir = os.getenv("HOME")

        if output_dir and output_dir[0] == "~" and home_dir:
            output_dir = output_dir.replace("~", home_dir, 1)

        elif output_dir and output_dir[0] == "~" and not home_dir:
            self.lc_node.get_logger().error(
                f"Invalid use of ~ in path: {output_dir}"
            )
            return None

        output_path = Path(output_dir)

        if node_namespace != "":

            namespace_path = Path(node_namespace)

            for part in namespace_path.parts:

                if part != namespace_path.root:
                    output_path /= part

            try:
                output_path.mkdir(parents=True, exist_ok=True)

            except OSError as err:

                self.lc_node.get_logger().error(
                    f"Error creating directories: {err}"
                )
                return None

        return output_path

    def configure(self, lc_node: LifecycleNode, plugin_name: str):
        
        print("llega al configure de upf")

        self.lc_node = lc_node

        self.argument_parameter_name = plugin_name + ".arguments"
        self.output_dir_parameter_name = plugin_name + ".output_dir"
        self.preferred_planner_param = plugin_name + ".preferred_planner"

        self.lc_node.declare_parameter(
            self.output_dir_parameter_name,
            tempfile.gettempdir()
        )

        self.lc_node.declare_parameter(
            self.preferred_planner_param,
            "default"
        )

        self.preferred_planner = self.lc_node.get_parameter(
            self.preferred_planner_param
        ).value

        self.lc_node.get_logger().info(
            f"Preferred planner: {self.preferred_planner}"
        )

        self.lc_node.get_logger().info(
            f"UPFPlanSolver configured with plugin name: {plugin_name}"
        )

    def get_plan(
        self,
        domain: str,
        problem: str,
        node_namespace: str,
        timeout: Duration
    ):

        try:

            print("Python executable:", sys.executable)
            print(unified_planning.__version__)
            print("Available engines:", get_environment().factory.engines)
            reader = PDDLReader()
            self.lc_node.get_logger().info(
                "PDDL reader created"
            )

            self.lc_node.get_logger().info(
                f"domain string {domain}"
            )

            self.lc_node.get_logger().info(
                f"problem string {problem}"
            )

            tmp_dir = Path("/tmp")

            domain_file = tmp_dir / "test_domain_ros.pddl"
            problem_file = tmp_dir / "test_problem_ros.pddl"

            # guardar archivos
            domain_file.write_text(domain)
            problem_file.write_text(problem)

            self.lc_node.get_logger().info(f"Domain saved to {domain_file}")
            self.lc_node.get_logger().info(f"Problem saved to {problem_file}")

            reader = PDDLReader()

            # leer desde archivo (como tu script que funciona)
            upf_problem = reader.parse_problem(
                str(domain_file),
                str(problem_file)
            )

            upf_problem = clean_problem(upf_problem)
            print(upf_problem)
            reader2 = PDDLWriter(upf_problem)
            self.lc_node.get_logger().info(f"{reader2.get_problem()}")
            self.lc_node.get_logger().info(f"{reader2.get_domain()}")


            print('sale del parse')

            # upf_problem = reader.parse_problem_string(domain, problem)

            self.lc_node.get_logger().info(
                "PDDL parsed successfully"
            )

            self.lc_node.get_logger().info(
                f"Engines disponibles: {get_environment().factory.engines}"
            )

            if self.preferred_planner == "default":

                print("Using default planner")

                with Compiler(problem_kind=upf_problem.kind) as compiler:
                    compiled_problem = compiler.compile(upf_problem).problem
                    with OneshotPlanner(problem_kind=compiled_problem.kind) as planner:
                        result = planner.solve(compiled_problem)

                        if result.status in unified_planning.engines.results.POSITIVE_OUTCOMES:
                            plan = result.plan
                            self.lc_node.get_logger().info(
                                f"Plan found: {plan}"
                            )
                            plan = self._calculate_plan(result.plan)
                            return plan

                        else:
                            self.lc_node.get_logger().error(
                                f"Planner failed with status: {result.status}"
                            )
    
            else:
                self.lc_node.get_logger().info(
                    "Using preferred planner"
                )

                with OneshotPlanner(name=self.preferred_planner) as planner:
                    result = planner.solve(upf_problem)

                    if result.status in unified_planning.engines.results.POSITIVE_OUTCOMES:
                        plan = self._calculate_plan(result.plan)
                        return plan
                        #return str(result.plan)

                    else:
                        self.lc_node.get_logger().error(
                            f"Planner failed with status: {result.status}"
                        )
                        return None
        except Exception as e:
            
            self.lc_node.get_logger().error(
                f"UPF planning error: {e}"
            )

        ## ------ decidir como hacer para no contaminar el entorno de popf----------
        # try:
        #     self.lc_node.get_logger().info(
        #         "Trying POPF planner"
        #     )

        #     from .popf_engine import register_popf
        #     register_popf()

        #     with OneshotPlanner(name="popf") as planner:
        #         result = planner.solve(upf_problem)

        #         if result.status in unified_planning.engines.results.POSITIVE_OUTCOMES:
        #             return str(result.plan)

        # except Exception as e:

        #     self.lc_node.get_logger().error(
        #         f"UPF planning error: {e}"
        #     )

            #traceback.print_exc()

        ##-------------------------------------------------------------------------------

            return None

    def is_domain_valid(self, domain: str, node_namespace: str):

        try:

            reader = PDDLReader()

            reader.parse_problem_string(domain, "")

            return True

        except Exception as e:

            self.lc_node.get_logger().error(
                f"Invalid domain: {e}"
            )

            return False
        
    def _calculate_plan(self, plan):

        ros_plan = Plan()

        for start, action_instance, duration in plan.timed_actions:

            item = PlanItem()

            item.time = float(start)
            item.duration = float(duration)

            name = action_instance.action.name
            params = [str(p) for p in action_instance.actual_parameters]

            item.action = f"({name} {' '.join(params)})"

            ros_plan.items.append(item)

        return ros_plan
    
def clean_problem(problem):

    from unified_planning.model import Fluent

    used_fluents = set()

    def extract_from_expression(expr):
        try:
            return set(expr.fluents())
        except Exception:
            return set()

    for action in problem.actions:

        if hasattr(action, "duration") and action.duration is not None:
            used_fluents |= extract_from_expression(action.duration)

        if hasattr(action, "conditions"):
            for timing, cond_list in action.conditions.items():
                for cond in cond_list:
                    used_fluents |= extract_from_expression(cond)

        if hasattr(action, "effects"):
            for effect in action.effects:
                try:
                    used_fluents |= extract_from_expression(effect.fluent)
                    if effect.value is not None:
                        used_fluents |= extract_from_expression(effect.value)
                except Exception:
                    pass

    if problem.goals is not None:
        used_fluents |= extract_from_expression(problem.goals)

    for fexp in problem.initial_values:
        used_fluents |= extract_from_expression(fexp)

    all_fluents = set(problem.fluents)
    unused_fluents = all_fluents - used_fluents

    unused_numeric = {
        f for f in unused_fluents if not f.type.is_bool_type()
    }

    if not unused_numeric:
        return problem

    for f in unused_numeric:
        print(f"  - {f}")

    to_delete = []
    for fexp in problem.initial_values:
        if fexp.fluent() in unused_numeric:
            to_delete.append(fexp)

    for fexp in to_delete:
        try:
            del problem._initial_value[fexp]
        except Exception:
            pass

    for f in unused_numeric:
        try:
            problem._fluents.remove(f)
        except Exception:
            pass

    return problem
