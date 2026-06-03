import unified_planning as up
from unified_planning.io import PDDLWriter
from unified_planning.engines import PlanGenerationResult, PlanGenerationResultStatus
from unified_planning.plans import TimeTriggeredPlan, ActionInstance
import subprocess
import tempfile
import os

def register_popf():
    """Registers POPFPlanner as 'popf' in the UPF engine factory. Must be called before use."""
    env = up.environment.get_environment()
    env.factory.add_engine(
        "popf",
        __name__,
        "POPFPlanner"
    )


class POPFPlanner(up.engines.Engine,
                   up.engines.mixins.OneshotPlannerMixin):
    def __init__(self, **options):
        """UPF engine adapter that runs the POPF binary as an external process."""
        # Read known user-options and store them for using in the `solve` method
        up.engines.Engine.__init__(self)
        up.engines.mixins.OneshotPlannerMixin.__init__(self)
        self.max_tries = options.get('max_tries', None)
        self.restart_probability = options.get('restart_probability', 0.00001)

    @property
    def name(self) -> str:
        return "POPFPlanner"

    @staticmethod
    def supported_kind():
        """Declares the problem features POPF can handle (temporal, numeric, hierarchical typing, etc.)."""
        supported_kind = up.model.ProblemKind(version=3)
        supported_kind.set_problem_class("ACTION_BASED")
        supported_kind.set_problem_type("GENERAL_NUMERIC_PLANNING")
        supported_kind.set_typing('FLAT_TYPING')
        supported_kind.set_typing('HIERARCHICAL_TYPING')
        supported_kind.set_numbers('CONTINUOUS_NUMBERS')
        supported_kind.set_numbers('DISCRETE_NUMBERS')
        supported_kind.set_fluents_type('NUMERIC_FLUENTS')
        supported_kind.set_numbers('BOUNDED_TYPES')
        supported_kind.set_fluents_type('OBJECT_FLUENTS')
        supported_kind.set_conditions_kind('NEGATIVE_CONDITIONS')
        supported_kind.set_conditions_kind('DISJUNCTIVE_CONDITIONS')
        supported_kind.set_conditions_kind('EQUALITIES')
        supported_kind.set_conditions_kind('EXISTENTIAL_CONDITIONS')
        supported_kind.set_conditions_kind('UNIVERSAL_CONDITIONS')
        supported_kind.set_effects_kind('CONDITIONAL_EFFECTS')
        supported_kind.set_effects_kind('INCREASE_EFFECTS')
        supported_kind.set_effects_kind('DECREASE_EFFECTS')
        supported_kind.set_effects_kind('FLUENTS_IN_NUMERIC_ASSIGNMENTS')
        supported_kind.set_time("CONTINUOUS_TIME")
        supported_kind.set_expression_duration("INT_TYPE_DURATIONS")

        return supported_kind

    @staticmethod
    def supports(problem_kind):
        """Returns True if problem_kind is a subset of what POPF supports."""
        return problem_kind <= POPFPlanner.supported_kind()

    def _solve_with_params(
        self,
        problem: 'up.model.Problem',
        **kwargs,
    ) -> 'up.engines.PlanGenerationResult':
        """
        Writes domain and problem to a temp directory, runs POPF via subprocess,
        parses the output plan with _parse_popf_plan and returns a PlanGenerationResult.
        """
        with tempfile.TemporaryDirectory() as tmpdir:

            domain_file = os.path.join(tmpdir, "domain.pddl")
            problem_file = os.path.join(tmpdir, "problem.pddl")
            plan_file = os.path.join(tmpdir, "plan.txt")

            # Exportar a PDDL
            writer = PDDLWriter(problem)
            writer.write_domain(domain_file)
            writer.write_problem(problem_file)

            # Ejecutar POPF
            cmd = ["ros2", "run", "popf", "popf",
                   domain_file, problem_file]

            with open(plan_file, "w") as f:
                subprocess.run(cmd, stdout=f)

            # Parsear plan (esto tienes que implementarlo)
            up_plan = self._parse_popf_plan(plan_file, problem)

            if up_plan is None:
                return PlanGenerationResult(
                    PlanGenerationResultStatus.UNSOLVABLE_PROVEN,
                    None,
                    self.name
                )

            return PlanGenerationResult(
                PlanGenerationResultStatus.SOLVED_SATISFICING,
                up_plan,
                self.name
            )

    def _solve(self, problem, heuristic=None, timeout=None, output_stream=None):
        # This method is deprecated in favor of `_solve_with_params`.
        # This method is kept for backward compatibility with older versions of UPF.
        # You should use this exact override in your own solver to pass the call to `_solve_with_params`.
        return self._solve_with_params(problem, heuristic, timeout, output_stream)

    def destroy(self):
        pass

    def _parse_popf_plan(self, plan_file, problem):
        """
        Parses a POPF plan file into a UPF TimeTriggeredPlan.
        Each line has the format: '<time>: (<action> <params>) [<duration>]'.
        Returns None if no actions are found (no solution).
        """
        timed_actions = []

        with open(plan_file, "r") as f:
            for line in f:

                line = line.strip()

                if not line or line.startswith(";"):
                    continue

                if ":" not in line or "(" not in line:
                    continue

                # Ejemplo:
                # 0.000: (move r1 room1 room2) [5.000]

                start_time = float(line.split(":")[0])
                duration = float(line.split("[")[1].split("]")[0])

                action_str = line.split("(")[1].split(")")[0]

                tokens = action_str.split()
                action_name = tokens[0]
                param_names = tokens[1:]

                action = problem.action(action_name)

                parameters = [problem.object(p) for p in param_names]

                action_inst = ActionInstance(action, tuple(parameters))

                timed_actions.append((start_time, action_inst, duration))

        if not timed_actions:
            return None

        return TimeTriggeredPlan(timed_actions)
