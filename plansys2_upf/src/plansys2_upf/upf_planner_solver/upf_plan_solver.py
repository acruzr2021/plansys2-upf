import os
import tempfile
from pathlib import Path
from typing import Optional

import unified_planning
from unified_planning.io import PDDLReader
from unified_planning.shortcuts import OneshotPlanner, get_environment
from unified_planning.engines.results import POSITIVE_OUTCOMES
from plansys2_msgs.msg import Plan, PlanItem

from rclpy.lifecycle import LifecycleNode
from rclpy.duration import Duration

from .popf_engine import register_popf
import re


class UPFPlanSolver:

    def __init__(self):
        """
        Solver plugin that bridges the ROS 2 planning service and UPF.
        Reads its configuration from ROS 2 parameters prefixed with the plugin name.
        preferred_planner controls whether to auto-select the best engine or use a specific one.
        """
        self.argument_parameter_name = None
        self.output_dir_parameter_name = None
        self.lc_node = None

    def create_folders(self, node_namespace: str) -> Optional[Path]:
        """
        Resolves the output directory from the output_dir parameter and creates it if needed.
        Expands ~ to the HOME environment variable. Returns the Path or None on error.
        """
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
        """
        Called by PlannerUPFNode during on_configure. Stores the node reference and reads
        the preferred_planner parameter (<plugin_name>.preferred_planner).
        'default' enables auto-selection; any other value names a specific UPF engine.
        """
        self.lc_node = lc_node

        self.argument_parameter_name = plugin_name + ".arguments"
        self.output_dir_parameter_name = plugin_name + ".output_dir"
        self.preferred_planner_param = plugin_name + ".preferred_planner"

        self.lc_node.declare_parameter(
            self.output_dir_parameter_name,
            tempfile.gettempdir()
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
        """
        Main planning method. Writes domain and problem to /tmp, patches the domain name to
        match the problem's (:domain) declaration, parses with PDDLReader, then runs the planner.
        In 'default' mode: tries all compatible engines, returns the plan with lowest makespan.
        In specific mode: uses the named engine directly.
        Returns a PlanSys2 Plan message, or None if no solution is found.
        """
        try:
            tmp_dir = Path("/tmp")

            domain_file = tmp_dir / "test_domain_ros.pddl"
            problem_file = tmp_dir / "test_problem_ros.pddl"

            domain_file.write_text(domain)
            problem_file.write_text(problem)

            problem_content = problem_file.read_text()
            problem_domain_match = re.search(r'\(:domain\s+([\w-]+)\)', problem_content)
            if problem_domain_match:
                problem_domain_name = problem_domain_match.group(1)
                domain_content = domain_file.read_text()
                domain_content = re.sub(
                    r'\(define\s+\(domain\s+[\w-]+\)',
                    f'(define (domain {problem_domain_name})',
                    domain_content
                )
                domain_file.write_text(domain_content)

            self.lc_node.get_logger().info(f"Domain saved to {domain_file}")
            self.lc_node.get_logger().info(f"Problem saved to {problem_file}")

            reader = PDDLReader()

            upf_problem = reader.parse_problem(
                str(domain_file),
                str(problem_file)
            )

            self.lc_node.get_logger().info("PDDL parsed successfully")

            if self.preferred_planner == "default":
                env = get_environment()
                candidates = []

                for name in env.factory.engines:
                    try:
                        with OneshotPlanner(name=name) as planner:
                            if planner.supports(upf_problem.kind):
                                self.lc_node.get_logger().info(f"Compatible planner: {name}")
                                result = planner.solve(upf_problem)
                                if result.status in POSITIVE_OUTCOMES and result.plan is not None:
                                    self.lc_node.get_logger().info(f"Plan found by: {name}")
                                    candidates.append((name, result.plan))
                                else:
                                    self.lc_node.get_logger().info(f"No plan from: {name}")
                            else:
                                self.lc_node.get_logger().info(f"Incompatible planner: {name}")
                    except Exception as e:
                        self.lc_node.get_logger().info(f"Exception with planner {name}: {e}")

                if not candidates:
                    self.lc_node.get_logger().error("No planner found a solution")
                    return None

                def plan_makespan(plan):
                    try:
                        return float(plan.makespan)
                    except Exception:
                        try:
                            return max(
                                float(start) + float(duration)
                                for start, _, duration in plan.timed_actions
                            )
                        except Exception:
                            try:
                                return float(len(plan.actions))
                            except Exception:
                                return float("inf")

                best_name, best_plan = min(candidates, key=lambda x: plan_makespan(x[1]))
                self.lc_node.get_logger().info(f"Selected planner: {best_name}")
                self.lc_node.get_logger().info(f"Makespan: {plan_makespan(best_plan)}")

                return self._calculate_plan(best_plan)

            else:
                self.lc_node.get_logger().info("Using preferred planner")

                with OneshotPlanner(name=self.preferred_planner) as planner:
                    result = planner.solve(upf_problem)

                    if result.status in unified_planning.engines.results.POSITIVE_OUTCOMES:
                        return self._calculate_plan(result.plan)
                    else:
                        self.lc_node.get_logger().error(
                            f"Planner failed with status: {result.status}"
                        )
                        return None

        except Exception as e:
            self.lc_node.get_logger().error(f"UPF planning error: {e}")
            return None

    def is_domain_valid(self, domain: str, node_namespace: str):
        """Tries to parse the domain string with PDDLReader. Returns True if it succeeds."""
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
        """
        Converts a UPF plan to a PlanSys2 Plan message.
        TimeTriggeredPlan: copies real start times and durations.
        SequentialPlan: assigns incremental integer times (0, 1, 2...) and duration 1.0.
        """
        ros_plan = Plan()

        # Plan temporal (TimeTriggeredPlan)
        if hasattr(plan, 'timed_actions'):
            for start, action_instance, duration in plan.timed_actions:
                item = PlanItem()
                item.time = float(start) if start is not None else 0.0
                item.duration = float(duration) if duration is not None else 0.0
                name = action_instance.action.name
                params = [str(p) for p in action_instance.actual_parameters]
                item.action = f"({name} {' '.join(params)})"
                ros_plan.items.append(item)

        # Plan secuencial (SequentialPlan)
        elif hasattr(plan, 'actions'):
            for i, action_instance in enumerate(plan.actions):
                item = PlanItem()
                item.time = float(i)
                item.duration = 1.0
                name = action_instance.action.name
                params = [str(p) for p in action_instance.actual_parameters]
                item.action = f"({name} {' '.join(params)})"
                ros_plan.items.append(item)

        return ros_plan
    
def clean_problem(problem):
    """
    Removes unused numeric fluents from the problem's initial state.
    A fluent is considered unused if it doesn't appear in any action condition,
    effect, duration expression, goal, or initial value expression.
    Modifies the problem in place and returns it.
    """
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
