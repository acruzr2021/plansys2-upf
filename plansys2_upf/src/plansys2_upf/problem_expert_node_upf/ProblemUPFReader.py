from unified_planning.model import *
from unified_planning.model import Problem
from unified_planning.io import PDDLReader
from plansys2_msgs.msg import Param, Tree
from plansys2_msgs.msg import Node as TreeNode
import sys
from unified_planning.shortcuts import And, Or, Not


_EXPR_TYPE_CHECKS = [
    ('is_le', TreeNode.COMP_LE),
    ('is_lt', TreeNode.COMP_LT),
    ('is_equals', TreeNode.COMP_EQ),
    ('is_div', TreeNode.ARITH_DIV),
    ('is_times', TreeNode.ARITH_MULT),
    ('is_plus', TreeNode.ARITH_ADD),
    ('is_minus', TreeNode.ARITH_SUB),
]

_ARITH_TYPES = {TreeNode.ARITH_ADD, TreeNode.ARITH_SUB, TreeNode.ARITH_MULT, TreeNode.ARITH_DIV}


class ProblemUPFExpert:
    def __init__(self, domain_reader):
        """
        Builds an empty problem from the given DomainUPFReader.
        Keeps two parallel representations: a UPF Problem object (self.problem)
        and PlanSys2 message lists (instances, predicates, functions, goal).
        """
        self.domain_reader = domain_reader
        self.domain_pddl = domain_reader.domain_pddl
        self.domain = domain_reader.domain
        self.problem = None
        self.goal = None
        self.instances = []
        self.predicates = []
        self.functions = []
        self._type_map = {t.name: t for t in self.domain.user_types}
        self._node_map = {}

        self.add_problem()

    def add_problem(self, problem_str=None):
        """
        Loads a PDDL problem string and resets all state lists.
        If problem_str is None, creates an empty problem from the domain only.
        Populates instances, predicates, functions and goal from the parsed UPF object.
        Returns True on success, False on parse error.
        """
        reader = PDDLReader()

        if problem_str is None:
            new_problem = reader.parse_problem_string(self.domain_pddl)
        else:
            try:
                new_problem = reader.parse_problem_string(self.domain_pddl, problem_str)
            except Exception as e:
                print(f"Error: {e}", file=sys.stderr)
                return False

        self.problem = new_problem
        self.instances = []
        self.predicates = []
        self.functions = []

        for instance in self.problem.all_objects:
            if instance.type not in self.domain.user_types:
                continue
            param = Param()
            param.name = instance.name
            param.type = instance.type.name
            param.sub_types = []
            if param not in self.instances:
                self.instances.append(param)

        for pred, val in self.problem._initial_value.items():
            try:
                if pred.type.is_bool_type():
                    try:
                        is_true = val.bool_constant_value()
                    except Exception:
                        is_true = bool(val)
                    if not is_true:
                        continue
                    name = str(pred).split("(")[0]
                    pred_obj = TreeNode()
                    pred_obj.node_type = TreeNode.PREDICATE
                    pred_obj.name = name
                    pred_obj.negate = False
                    pred_obj.parameters = []
                    for arg in pred.args:
                        param = Param()
                        param.name = str(arg)
                        param.type = str(arg.type)
                        param.sub_types = []
                        pred_obj.parameters.append(param)
                    if not self.exists_predicate(pred_obj):
                        self.predicates.append(pred_obj)

                elif pred.type.is_real_type():
                    name = str(pred).split("(")[0]
                    func_obj = TreeNode()
                    func_obj.node_type = TreeNode.FUNCTION
                    func_obj.name = name
                    func_obj.value = float(val)
                    func_obj.parameters = []
                    for arg in pred.args:
                        param = Param()
                        param.name = str(arg)
                        param.type = str(arg.type)
                        param.sub_types = []
                        func_obj.parameters.append(param)
                    if not self.exists_function(func_obj):
                        self.functions.append(func_obj)

            except Exception:
                continue

        tree_goals = Tree()
        _build_tree_node(self.problem._goals, tree_goals, [])
        self.goal = tree_goals

        return True

    def add_instance(self, instance):
        """
        Adds an object instance. Validates the type exists, normalizes it to lowercase,
        and rejects duplicate names with different types. Updates both UPF and the list.
        """
        if not isinstance(instance, Param):
            return False

        instance.type = instance.type.lower()

        type_obj = self._type_map.get(instance.type)
        if type_obj is None:
            return False

        if instance in self.instances:
            return True

        if instance.name in {i.name for i in self.instances}:
            return False

        self.problem.add_object(Object(instance.name, type_obj))
        self.instances.append(instance)
        return True

    def add_predicate(self, predicate):
        """
        Adds a predicate to the initial state. Infers missing parameter types from instances,
        validates against the domain fluent (name, arity, type compatibility including inheritance),
        and sets the value to True in UPF via set_initial_value.
        Returns True if already exists (idempotent).
        """
        if not isinstance(predicate, TreeNode):
            return False

        if predicate.node_type != TreeNode.PREDICATE:
            return False

        for p in predicate.parameters:
            if p.type == "":
                inst = self.get_instance(p.name)
                if inst is None:
                    return False
                p.type = inst.type

        fluent = self._find_matching_fluent(predicate.name, predicate.parameters, bool_fluent=True)
        if fluent is None:
            return False

        if len(predicate.parameters) != len(fluent.signature):
            return False

        if self.exists_predicate(predicate):
            return True

        expr = self._build_upf_expression(fluent, predicate.parameters)
        if expr is None:
            return False

        self.problem.set_initial_value(expr, True)
        self.predicates.append(predicate)
        return True

    def _find_upf_object(self, name, type_name):
        """Finds a UPF object by name and type. Returns None if not found."""
        for obj in self.problem.all_objects:
            if obj.name == name:
                obj_type_name = obj.type if isinstance(obj.type, str) else str(obj.type.name)
                if obj_type_name == type_name:
                    return obj
        return None

    def add_function(self, function):
        """
        Adds a numeric function to the initial state. If it already exists, delegates to update_function.
        Infers missing parameter types from instances and validates against the domain fluent.
        """
        if not isinstance(function, TreeNode):
            return False

        for p in function.parameters:
            if p.type == "":
                inst = self.get_instance(p.name)
                if inst is None:
                    return False
                p.type = inst.type

        fluent = self._find_matching_fluent(function.name, function.parameters, bool_fluent=False)
        if fluent is None:
            return False

        if function.node_type != TreeNode.FUNCTION:
            return False

        if not self.exists_function(function):
            expr = self._build_upf_expression(fluent, function.parameters)
            if expr is None:
                return False
            self.problem.set_initial_value(expr, function.value)
            self.functions.append(function)
            return True

        return self.update_function(function)

    def add_goal(self, goal):
        """
        Sets the planning goal from a PlanSys2 Tree. Converts it to a UPF expression via _tree_to_upf.
        Clears any existing goal before adding the new one.
        """
        if not isinstance(goal, Tree):
            return False

        self._node_map = {n.node_id: n for n in goal.nodes}
        root = self._node_map.get(0)
        if root is None:
            return False

        upf_goal = self._tree_to_upf(root)
        if upf_goal is None:
            return False

        if self.goal is not None:
            self.remove_goal()

        self.problem.add_goal(upf_goal)
        self.goal = goal
        return True

    def get_goal(self):
        """Returns the current goal Tree, or an empty Tree if none is set."""
        return self.goal if self.goal is not None else Tree()

    def get_instance(self, instance_name):
        """Returns the Param for the given instance name, or None if not found."""
        for instance in self.instances:
            if instance.name == instance_name:
                return instance
        return None

    def get_instances(self):
        """Returns the full list of instances."""
        return self.instances

    def get_predicate(self, predicate):
        """
        Finds a predicate by its PDDL expression string, e.g. '(robot_at r2d2 bedroom)'.
        Returns the matching TreeNode from self.predicates, or None if not found.
        """
        pred_str = predicate.strip()
        if not (pred_str.startswith('(') and pred_str.endswith(')')):
            return None

        content = pred_str[1:-1].strip()
        parts = content.split()
        if not parts:
            return None

        predicate_name = parts[0]
        args = parts[1:]

        for predicate in self.predicates:
            if str(predicate.name) != predicate_name:
                continue
            correct_arg = 0
            for arg in args:
                for pred_arg in predicate.parameters:
                    if str(pred_arg.name) == arg:
                        correct_arg += 1
            if correct_arg == len(args):
                return predicate
        return None

    def get_predicates(self):
        """Returns the full list of active predicates."""
        return self.predicates

    def get_function(self, function):
        """
        Finds a function by its PDDL expression string.
        Accepts formats like '(speed r2d2)' or '(= (speed r2d2) 3.0)'.
        Returns the matching TreeNode from self.functions, or None if not found.
        """
        function_str = function
        if function_str.startswith('(') and function_str.endswith(')'):
            function_str = function_str[1:-1].strip()

        val = None

        if function_str.startswith('='):
            function_str = function_str.split('(')[1]
            val = function_str.split(')')[1]

            if len(val.split(' ')) > 1:
                val = val.split(' ')[-1]
            paren_end = function_str.find(')')

            if paren_end == -1:
                return None
            
            function_part = function_str[:paren_end + 1]
            if function_part.endswith(')'):
                function_part = function_part[:-1].strip()

            else:
                function_part = function_part.split(')')
                
            function_str = function_part

        function_str = function_str.split()
        function_name = function_str[0]
        args = function_str[1:]

        for function in self.functions:
            if str(function.name) != function_name:
                continue
            correct_arg = 0
            for arg in args:
                for func_arg in function.parameters:
                    if str(func_arg.name) == arg:
                        correct_arg += 1
            if val and function.value != float(val):
                continue
            if correct_arg == len(args):
                return function

        return None

    def get_functions(self):
        """Returns the full list of numeric functions."""
        return self.functions

    def get_problem(self):
        """
        Serializes the problem state to a PDDL string manually (not via PDDLWriter).
        :objects comes from self.instances; :init predicates and functions come from
        UPF's _initial_value; :goal comes from self.goal.
        """
        lines = [
            f"(define (problem {self.problem.name})",
            f"  (:domain {self.domain.name})",
            "  (:objects",
        ]
        for inst in self.instances:
            lines.append(f"    {inst.name} - {inst.type}")
        lines.append("  )")

        lines.append("  (:init")
        for pred in self._get_predicates_pddl():
            lines.append(f"    {pred}")
        for func in self._get_functions_pddl():
            lines.append(f"    {func}")
        lines.append("  )")

        goal = self._get_goal_pddl()
        if goal:
            lines.append(f"  (:goal {goal})")

        lines.append(")")
        return "\n".join(lines)

    def remove_goal(self):
        """Clears the current goal in both UPF and self.goal."""
        self.goal = None
        self.problem.clear_goals()
        return True

    def clear_knowledge(self):
        """Resets all problem state: re-parses an empty problem from the domain only."""
        if self.problem is None:
            return True

        reader = PDDLReader()
        self.problem = reader.parse_problem_string(self.domain_pddl)
        self.instances = []
        self.predicates = []
        self.functions = []
        self.goal = None
        return True

    def remove_instance(self, instance):
        """
        Removes an instance with cascade deletion: removes all predicates and functions
        referencing it, filters the goal, and rebuilds the UPF Problem without that object.
        """
        if not isinstance(instance, Param):
            return False

        if instance not in self.instances:
            return False

        for pred in list(self.predicates):
            for param in pred.parameters:
                if param.name == instance.name and param.type == instance.type:
                    self.remove_predicate(pred)
                    break

        for func in list(self.functions):
            for param in func.parameters:
                if param.name == instance.name and param.type == instance.type:
                    self.remove_function(func)
                    break

        new_goal = self._clear_goal(self.goal, instance)
        if new_goal is not None:
            self.add_goal(new_goal)

        instance_upf = next(
            (obj for obj in self.problem.all_objects
             if obj.name == instance.name and obj.type == instance.type),
            None
        )
        if instance_upf:
            self._rebuild_problem(objects=[instance_upf])

        self.instances.remove(instance)
        return True

    def _params_equal(self, a, b):
        """Returns True if two parameter lists have the same names and types."""
        if len(a) != len(b):
            return False
        return all(p1.name == p2.name and p1.type == p2.type for p1, p2 in zip(a, b))

    def remove_predicate(self, predicate):
        """
        Removes a predicate by setting its UPF value to False (UPF has no delete API).
        Returns True if already absent (idempotent).
        """
        if not isinstance(predicate, TreeNode):
            return False

        fluent = self._find_matching_fluent(predicate.name, predicate.parameters, True)
        if fluent is None:
            return False

        if len(predicate.parameters) != len(fluent.signature):
            return False

        if not self.exists_predicate(predicate):
            return True

        expr = self._build_upf_expression(fluent, predicate.parameters)
        if expr is None:
            return False

        self.problem.set_initial_value(expr, False)
        self.predicates = [
            p for p in self.predicates
            if not (p.name == predicate.name and self._params_equal(p.parameters, predicate.parameters))
        ]
        return True

    def remove_function(self, function):
        """
        Removes a numeric function. Since UPF has no direct delete API for initial values,
        this rebuilds the entire Problem object excluding the target function.
        """
        if not isinstance(function, TreeNode):
            return False

        if not self.exists_function(function):
            return False

        fluent = self._find_matching_fluent(function.name, function.parameters, False)
        if fluent is None:
            return False

        expr = self._build_upf_expression(fluent, function.parameters)
        if expr is None:
            return False

        self._rebuild_problem(functions=[expr])
        self.functions = [
            f for f in self.functions
            if not (f.name == function.name and self._params_equal(f.parameters, function.parameters))
        ]
        return True

    def exists_function(self, function):
        """Returns True if an equivalent function exists in self.functions."""
        if not isinstance(function, TreeNode):
            return False
        return any(check_equality_node(f, function) for f in self.functions)

    def exists_predicate(self, predicate):
        """Checks if a predicate is true in UPF's _initial_value (authoritative source)."""
        if not isinstance(predicate, TreeNode):
            return False

        fluent = self._find_matching_fluent(predicate.name, predicate.parameters, True)
        if fluent is None:
            return False

        expr = self._build_upf_expression(fluent, predicate.parameters)
        if expr is None:
            return False

        val = self.problem._initial_value.get(expr)
        if val is None:
            return False

        try:
            return val.bool_constant_value()
        except Exception:
            return bool(val)

    def is_problem_goal_satisfied(self, goal):
        """Returns True if the given goal Tree is satisfied by the current state."""
        if not isinstance(goal, Tree):
            return False
        return check(goal, self.predicates, self.functions)

    def update_function(self, function):
        """Updates the value of an existing function in both self.functions and UPF."""
        if not isinstance(function, TreeNode):
            return False

        if function.node_type != TreeNode.FUNCTION:
            return False

        if not self.exists_function(function):
            return False

        self.functions = [
            f for f in self.functions
            if not (f.name == function.name and self._params_equal(f.parameters, function.parameters))
        ]
        self.functions.append(function)

        fluent = self._find_matching_fluent(function.name, function.parameters, False)
        expr = self._build_upf_expression(fluent, function.parameters)
        if expr is None:
            return False

        self.problem.set_initial_value(expr, function.value)
        return True

    def _get_temporal_problem(self, expression):
        """Parses a PDDL problem string combined with the domain. Returns None on error."""
        if expression is None:
            return None
        reader = PDDLReader()
        try:
            return reader.parse_problem_string(self.domain_pddl, expression)
        except Exception as e:
            print(f"Error loading PDDL: {e}", file=sys.stderr)
            return None

    def _find_matching_fluent(self, name, parameters, bool_fluent=None):
        """
        Finds a domain fluent matching the given name, arity and parameter types.
        bool_fluent=True restricts to predicates; False to numeric functions; None matches both.
        Type compatibility includes inheritance (checked via _types_compatible).
        """
        for fluent in self.problem.fluents:
            if bool_fluent is True and not fluent.type.is_bool_type():
                continue
            if bool_fluent is False and fluent.type.is_bool_type():
                continue
            if fluent.name != name:
                continue
            if len(fluent.signature) != len(parameters):
                continue
            if all(self._types_compatible(p.type, f.type) for p, f in zip(parameters, fluent.signature)):
                return fluent
        return None

    def is_valid_goal(self, goal: Tree) -> bool:
        """Returns True if all predicates and functions in the goal tree exist in the domain."""
        if not isinstance(goal, Tree) or not goal.nodes:
            return False
        self._node_map = {n.node_id: n for n in goal.nodes}
        return self._check_goal_node(0)

    def _check_goal_node(self, node_id: int) -> bool:
        node = self._node_map.get(node_id)
        if node is None:
            return False

        if node.node_type in (TreeNode.AND, TreeNode.OR):
            return all(self._check_goal_node(child_id) for child_id in node.children)

        if node.node_type == TreeNode.NOT:
            return bool(node.children) and self._check_goal_node(node.children[0])

        if node.node_type == TreeNode.PREDICATE:
            return self.is_valid_predicate(node)

        if node.node_type == TreeNode.FUNCTION:
            return self.is_valid_function(node)

        if node.node_type == TreeNode.EXPRESSION:
            return all(self._check_goal_node(child_id) for child_id in node.children)

        return node.node_type == TreeNode.NUMBER

    def _is_valid_fluent(self, node, is_predicate: bool) -> bool:
        fluent = self._find_matching_fluent(node.name, node.parameters, is_predicate)
        if fluent is None or len(node.parameters) != len(fluent.signature):
            return False
        return all(str(p.type) == str(f.type) for p, f in zip(node.parameters, fluent.signature))

    def is_valid_predicate(self, node) -> bool:
        return self._is_valid_fluent(node, True)

    def is_valid_function(self, node) -> bool:
        return self._is_valid_fluent(node, False)

    def _types_compatible(self, param_type_name, fluent_type):
        """Returns True if param_type_name matches fluent_type or any of its ancestors."""
        if param_type_name.lower() == fluent_type.name.lower():
            return True

        t = self._type_map.get(param_type_name)
        while t is not None:
            if t.name.lower() == fluent_type.name.lower():
                return True
            t = t.father
        return False

    def _build_upf_expression(self, fluent, parameters):
        """Instantiates a UPF fluent with the given parameter objects. Returns None if any object is missing."""
        objs = []
        for p in parameters:
            obj = self._find_upf_object(p.name, p.type)
            if obj is None:
                return None
            objs.append(obj)
        return fluent(*objs)

    def _tree_to_upf(self, node):
        """
        Recursively converts a PlanSys2 Tree node to a UPF expression.
        PREDICATE/FUNCTION -> fluent expression; AND/OR -> And/Or; NOT -> Not.
        Returns None if any node cannot be resolved.
        """
        if node.node_type in (TreeNode.PREDICATE, TreeNode.FUNCTION):
            is_bool = node.node_type == TreeNode.PREDICATE
            for p in node.parameters:
                if p.type == "":
                    inst = self.get_instance(p.name)
                    if inst is None:
                        return None
                    p.type = inst.type
            fluent = self._find_matching_fluent(node.name, node.parameters, is_bool)
            if fluent is None:
                return None
            expr = self._build_upf_expression(fluent, node.parameters)
            if expr is None:
                return None
            return Not(expr) if node.negate else expr

        if node.node_type in (TreeNode.AND, TreeNode.OR):
            combiner = And if node.node_type == TreeNode.AND else Or
            children = []
            for cid in node.children:
                child_expr = self._tree_to_upf(self._node_map[cid])
                if child_expr is None:
                    return None
                children.append(child_expr)
            return combiner(*children)

        if node.node_type == TreeNode.NOT:
            child_expr = self._tree_to_upf(self._node_map[node.children[0]])
            if child_expr is None:
                return None
            return Not(child_expr)

        return None

    def _build_tree(self, tree_msg):
        node_map = {n.node_id: n for n in tree_msg.nodes}

        def build(node_id):
            node = node_map[node_id]
            node.children = [build(cid) for cid in node.children]
            return node

        return build(0)

    def _rebuild_problem(self, *, objects=None, predicates=None, functions=None, goal=None):
        """
        Reconstructs the UPF Problem from scratch, excluding the specified objects/predicates/functions.
        Used when UPF's API doesn't support direct deletion of initial values or objects.
        Replaces self.problem with the new object.
        """
        old = self.problem
        new = Problem(old.name)

        for t in old.user_types:
            new._add_user_type(t)

        for f in old.fluents:
            new.add_fluent(f)

        kept_objects = (
            old.all_objects if objects is None
            else [o for o in old.all_objects if o not in objects]
        )
        for obj in kept_objects:
            new.add_object(obj)

        fluents_to_skip = (predicates or []) + (functions or [])
        for expr, value in old._initial_value.items():
            skip = any(
                expr.fluent().name == f.fluent().name
                and len(expr.args) == len(f.args)
                and all(str(a) == str(b) for a, b in zip(expr.args, f.args))
                for f in fluents_to_skip
            )
            if not skip:
                new.set_initial_value(expr, value)

        for g in (old.goals if goal is None else [goal]):
            new.add_goal(g)

        self.problem = new

    def _clear_goal(self, goal: Tree, instance: Param):
        """
        Returns a new goal Tree with all predicates referencing the given instance removed.
        Returns None if no valid predicates remain.
        """
        valid_predicates = [
            node for node in goal.nodes
            if node.node_type == TreeNode.PREDICATE
            and not any(p.name == instance.name and p.type == instance.type for p in node.parameters)
        ]

        if not valid_predicates:
            return None

        root = TreeNode()
        root.node_type = TreeNode.AND
        root.node_id = 0
        root.children = list(range(1, len(valid_predicates) + 1))

        nodes = [root]
        for i, pred in enumerate(valid_predicates, start=1):
            n = TreeNode()
            n.node_type = TreeNode.PREDICATE
            n.node_id = i
            n.name = pred.name
            n.parameters = pred.parameters
            n.negate = pred.negate
            n.children = []
            nodes.append(n)

        return Tree(nodes=nodes)

    def _get_goal_pddl(self):
        """Serializes self.goal to a PDDL goal expression string. Returns empty string if no goal."""
        if not self.problem or not self.problem.goals:
            return ""
        if not self.goal or not self.goal.nodes:
            return ""

        node_by_id = {n.node_id: n for n in self.goal.nodes}

        def node_to_pddl(node_id):
            node = node_by_id.get(node_id)
            if node is None:
                return ""
            if node.node_type == TreeNode.AND:
                return f"(and {' '.join(node_to_pddl(c) for c in node.children)})"
            if node.node_type == TreeNode.OR:
                return f"(or {' '.join(node_to_pddl(c) for c in node.children)})"
            if node.node_type == TreeNode.NOT:
                return f"(not {node_to_pddl(node.children[0])})"
            if node.node_type == TreeNode.PREDICATE:
                args = " ".join(p.name for p in node.parameters)
                return f"({node.name} {args})" if args else f"({node.name})"
            return ""

        return node_to_pddl(0)

    def _get_predicates_pddl(self):
        """Returns a list of PDDL predicate strings from UPF's _initial_value (true boolean fluents only)."""
        predicates = []
        for fluent, value in self.problem._initial_value.items():
            if not fluent.type.is_bool_type():
                continue
            try:
                is_true = value.bool_constant_value()
            except Exception:
                is_true = bool(value)
            if not is_true:
                continue
            args = " ".join(str(a) for a in fluent.args)
            predicates.append(f"({fluent.fluent().name} {args})")
        return predicates

    def _get_functions_pddl(self):
        """Returns a list of PDDL function assignment strings from UPF's _initial_value (numeric fluents only)."""
        if self.problem is None:
            return []
        functions = []
        for fluent, value in self.problem._initial_value.items():
            if not fluent.type.is_real_type():
                continue
            val = value.constant_value() if hasattr(value, "constant_value") else value
            args = " ".join(str(a) for a in fluent.args)
            functions.append(f"(= ({fluent.fluent().name} {args}) {float(val)})")
        return functions


def from_string_param(name: str, type_: str) -> Param:
    """Creates a Param with the given name and type."""
    p = Param()
    p.name = name
    p.type = type_
    p.sub_types = []
    return p


def check_equality_node(tree_a, tree_b):
    """Returns True if two TreeNodes have the same type, name, parameters and children."""
    if tree_a.node_type != tree_b.node_type:
        return False
    if tree_a.name != tree_b.name:
        return False
    if len(tree_a.parameters) != len(tree_b.parameters):
        return False
    for p1, p2 in zip(tree_a.parameters, tree_b.parameters):
        if p1.name != p2.name or p1.type != p2.type:
            return False
    if len(tree_a.children) != len(tree_b.children):
        return False
    return all(a == b for a, b in zip(tree_a.children, tree_b.children))


def check_node(node_id, tree, predicates, functions):
    """Recursively evaluates a tree node against the current state. Used by check()."""
    node = tree.nodes[node_id]

    if node.node_type == TreeNode.AND:
        return all(check_node(c, tree, predicates, functions) for c in node.children)

    if node.node_type == TreeNode.OR:
        return any(check_node(c, tree, predicates, functions) for c in node.children)

    if node.node_type == TreeNode.NOT:
        return not check_node(node.children[0], tree, predicates, functions)

    if node.node_type == TreeNode.EXPRESSION:
        left_val = evaluate_value(node.children[0], tree, predicates, functions)
        right_val = evaluate_value(node.children[1], tree, predicates, functions)
        return compare(left_val, right_val, node.expression_type)

    if node.node_type == TreeNode.PREDICATE:
        return any(
            p.name == node.name and p.parameters == node.parameters
            for p in predicates
        )

    if node.node_type == TreeNode.FUNCTION:
        current_value = next(
            (f.value for f in functions if f.name == node.name and f.parameters == node.parameters),
            None
        )
        if current_value is None:
            return False
        if node.value != 0.0:
            return compare(current_value, node.value, node.expression_type)
        if node.children:
            return compare(
                current_value,
                evaluate_value(node.children[0], tree, predicates, functions),
                node.expression_type,
            )
        return True

    if node.node_type == TreeNode.NUMBER:
        return node.value

    return False


def evaluate_value(node_id, tree, predicates, functions):
    """Evaluates a numeric expression node to a float. Handles NUMBER, FUNCTION and arithmetic."""
    node = tree.nodes[node_id]

    if node.node_type == TreeNode.NUMBER:
        return node.value

    if node.node_type == TreeNode.FUNCTION:
        return next(
            (f.value for f in functions if f.name == node.name and f.parameters == node.parameters),
            0.0,
        )

    if node.node_type == TreeNode.EXPRESSION and node.expression_type in _ARITH_TYPES:
        left_val = evaluate_value(node.children[0], tree, predicates, functions)
        right_val = evaluate_value(node.children[1], tree, predicates, functions)
        if node.expression_type == TreeNode.ARITH_ADD:
            return left_val + right_val
        if node.expression_type == TreeNode.ARITH_SUB:
            return left_val - right_val
        if node.expression_type == TreeNode.ARITH_MULT:
            return left_val * right_val
        if node.expression_type == TreeNode.ARITH_DIV:
            return left_val / right_val if right_val != 0 else 0.0

    return 0.0


def check(tree, predicates, functions):
    """Returns True if the goal Tree is satisfied by the given predicates and functions."""
    return check_node(0, tree, predicates, functions)


def compare(a, b, cmp):
    """Applies a comparison operator (COMP_EQ, COMP_GT, etc.) to two numeric values."""
    if cmp == TreeNode.COMP_EQ:
        return a == b
    if cmp == TreeNode.COMP_GT:
        return a > b
    if cmp == TreeNode.COMP_GE:
        return a >= b
    if cmp == TreeNode.COMP_LT:
        return a < b
    if cmp == TreeNode.COMP_LE:
        return a <= b
    raise ValueError(f"Unknown comparison type: {cmp}")


def make_node(tree, node_type, name="", value=0.0, negate=False, parameters=None, expression_type=0, modifier_type=0):
    """Creates a TreeNode and appends it to the tree. node_id is assigned as its position in the list."""
    node = TreeNode()
    node.node_id = len(tree.nodes)
    node.node_type = node_type
    node.name = str(name).split("(")[0]
    node.value = value
    node.negate = negate
    node.expression_type = expression_type
    node.modifier_type = modifier_type
    node.parameters = parameters or []
    node.children = []
    tree.nodes.append(node)
    return node


def _build_tree_node(expr, tree: Tree, params) -> int:
    """
    Recursively converts a UPF FNode (or list of FNodes) to a PlanSys2 Tree.
    Used to convert UPF goal expressions to the Tree format expected by PlanSys2.
    Returns the node_id of the created root node.
    """
    if not tree.nodes:
        node = make_node(tree, TreeNode.AND)
        if not isinstance(expr, list):
            expr = [expr]
        for arg in expr:
            cid = _build_tree_node(arg, tree, params)
            if cid != 0:
                tree.nodes[node.node_id].children.append(cid)
        return node.node_id

    if expr.is_and():
        node = tree.nodes[0] if len(tree.nodes) == 1 else make_node(tree, TreeNode.AND)
        for arg in expr.args:
            tree.nodes[node.node_id].children.append(_build_tree_node(arg, tree, params))
        return node.node_id

    if expr.is_or():
        node = make_node(tree, TreeNode.OR)
        for arg in expr.args:
            tree.nodes[node.node_id].children.append(_build_tree_node(arg, tree, params))
        return node.node_id

    if expr.is_not():
        node = make_node(tree, TreeNode.NOT)
        tree.nodes[node.node_id].children.append(_build_tree_node(expr.arg(0), tree, params))
        return node.node_id

    for method_name, expr_type in _EXPR_TYPE_CHECKS:
        if getattr(expr, method_name)():
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=expr_type)
            for a in expr.args:
                tree.nodes[node.node_id].children.append(_build_tree_node(a, tree, params))
            return node.node_id

    if expr.is_int_constant():
        return make_node(tree, TreeNode.NUMBER, value=float(expr.int_constant_value())).node_id

    if expr.is_real_constant():
        return make_node(tree, TreeNode.NUMBER, value=float(expr.real_constant_value())).node_id

    if isinstance(expr, FNode):
        fluent_name = str(expr).split("(")[0]
        node_type = TreeNode.PREDICATE if expr.type.is_bool_type() else TreeNode.FUNCTION
        node = make_node(tree, node_type, name=fluent_name)
        for a in expr.args:
            param = Param()
            param.name = str(a)
            param.type = str(a.type)
            param.sub_types = []
            node.parameters.append(param)
        return node.node_id

    raise NotImplementedError(f"Unsupported UPF node type: {expr}")
