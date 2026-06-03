from unified_planning.model import *
from unified_planning.io import PDDLReader, PDDLWriter
from plansys2_msgs.msg import Action, Param, Tree, DurativeAction
from plansys2_msgs.msg import Node as TreeNode
import re
import sys
from collections import defaultdict
from pathlib import Path


_EXPR_TYPE_CHECKS = [
    ('is_le', TreeNode.COMP_LE),
    ('is_lt', TreeNode.COMP_LT),
    ('is_equals', TreeNode.COMP_EQ),
    ('is_div', TreeNode.ARITH_DIV),
    ('is_times', TreeNode.ARITH_MULT),
    ('is_plus', TreeNode.ARITH_ADD),
    ('is_minus', TreeNode.ARITH_SUB),
]

_EFFECT_MODIFIER_MAP = {
    EffectKind.INCREASE: TreeNode.INCREASE,
    EffectKind.DECREASE: TreeNode.DECREASE,
    EffectKind.ASSIGN: TreeNode.ASSIGN,
}


class DomainUPFReader:
    def __init__(self, logger=None):
        """
        domain: UPF Problem object representing the loaded domain.
        domain_pddl: raw PDDL string, used by ProblemUPFExpert to parse domain+problem together.
        children_map: {type_name: [direct subtypes]}, built after loading.
        _type_map: {type_name: UPF type object}, used when building action parameters.
        """
        self.domain = None
        self.domain_pddl = None
        self.children_map = None
        self._type_map = {}
        self.logger = logger or print

    def _parse_pddl_string(self, pddl_string):
        """
        If pddl_string has no newlines, treats it as a file path and reads it.
        Then strips comments and empty sections before parsing.
        """
        if '\n' not in pddl_string:
            try:
                path = Path(pddl_string)
                if path.exists():
                    pddl_string = path.read_text()
            except (OSError, ValueError):
                pass
        return remove_comments(remove_empty_sections(pddl_string))

    def load_pddl(self, pddl_string):
        """
        Loads a PDDL domain. Accepts a file path or a PDDL string.
        Returns True on success, False on error.
        """
        reader = PDDLReader()
        try:
            self.domain_pddl = self._parse_pddl_string(pddl_string)
            self.domain = reader.parse_problem_string(self.domain_pddl)
            self._build_maps()
            return True
        except Exception as e:
            print(f"Error loading PDDL file: {e}", file=sys.stderr)
            return False

    def extend_domain(self, pddl_string):
        """
        Merges a second PDDL domain into the current one.
        Only adds types, fluents and actions not already present (additive merge).
        Regenerates domain_pddl with PDDLWriter after merging.
        Returns True on success, False on error.
        """
        reader = PDDLReader()
        try:
            new_dom = reader.parse_problem_string(self._parse_pddl_string(pddl_string))
        except Exception as e:
            print(f"Error loading PDDL file: {e}", file=sys.stderr)
            return False

        if self.domain is None:
            self.domain = new_dom
            self._build_maps()
            return True

        def _merge(existing_names, new_items, add_fn):
            for item in new_items:
                if item.name not in existing_names:
                    add_fn(item)

        _merge({t.name for t in self.domain.user_types}, new_dom.user_types, self.domain._add_user_type)
        _merge({f.name for f in self.domain.fluents}, new_dom.fluents, self.domain.add_fluent)
        _merge({a.name for a in self.domain.actions}, new_dom.actions, self.domain._actions.append)

        self._build_maps()
        self.domain_pddl = PDDLWriter(self.domain).get_domain()
        return True

    def get_name(self):
        """Returns the domain name, or empty string if no domain is loaded."""
        return str(self.domain.name) if self.domain else ""

    def get_types(self):
        """
        Returns the list of user type names.
        Always inserts 'object' as the first element, as required by PlanSys2.
        """
        types = []
        for x in self.domain.user_types:
            s = str(x)
            types.append(s.split(" - ")[0] if " - " in s else s)
        if "object" not in types:
            types.insert(0, "object")
        return types

    def get_constants(self, type_name):
        """Returns domain constants of the given type as a list of strings."""
        if not type_name:
            return []
        return [str(x) for x in self.domain.all_objects if str(x.type) == type_name]

    def get_predicates(self):
        """Returns boolean fluents as a list of PREDICATE TreeNodes."""
        nodes = []
        for f in self.domain.fluents:
            if f.type.is_bool_type():
                node = TreeNode()
                node.node_type = TreeNode.PREDICATE
                node.name = f.name
                nodes.append(node)
        return nodes

    def get_functions(self):
        """Returns non-boolean fluents as a list of FUNCTION TreeNodes."""
        nodes = []
        for f in self.domain.fluents:
            if not f.type.is_bool_type():
                node = TreeNode()
                node.node_type = TreeNode.FUNCTION
                node.name = f.name
                nodes.append(node)
        return nodes

    def get_predicate(self, name):
        """Returns a PREDICATE TreeNode with parameters for the given name (case-insensitive)."""
        return self._get_fluent_node(name, TreeNode.PREDICATE)

    def get_function(self, name):
        """Returns a FUNCTION TreeNode with parameters for the given name (case-insensitive)."""
        return self._get_fluent_node(name, TreeNode.FUNCTION)

    def get_domain(self):
        """Regenerates and returns the domain as a PDDL string using PDDLWriter."""
        writer = PDDLWriter(self.domain)
        return writer.get_domain()

    def get_actions(self):
        """Returns the names of all InstantaneousAction in the domain."""
        return [
            str(a).split("(")[0].split(" ")[-1]
            for a in self.domain.actions
            if isinstance(a, InstantaneousAction)
        ]

    def get_action(self, action_name, params=None):
        """
        Returns an instantaneous action as a PlanSys2 Action message.
        params: optional list of concrete parameter names; positional names (?0, ?1...) used otherwise.
        Returns None if the action does not exist.
        """
        for action_obj in self.domain.actions:
            if not isinstance(action_obj, InstantaneousAction):
                continue
            action_str = str(action_obj)
            before_paren, rest = action_str.split("(", 1)
            name = before_paren.split(" ")[-1]
            if name != action_name:
                continue

            params_action = re.split(r"\s*,\s*", rest.split(")")[0])
            parameters, params_list, param_index_map = self._build_params(
                params_action, action_obj.parameters, params
            )

            action_return = Action()
            action_return.name = name
            action_return.parameters = parameters
            action_return.preconditions = preconditions_to_tree(
                action_obj._preconditions, params_list, param_index_map
            )
            action_return.effects = effects_to_tree(action_obj.effects, params_list, param_index_map)
            return action_return
        return None

    def get_durative_actions(self):
        """Returns the names of all durative (non-instantaneous) actions in the domain."""
        return [
            str(a).split("(")[0].split(" ")[-1]
            for a in self.domain.actions
            if not isinstance(a, InstantaneousAction)
        ]

    def get_durative_action(self, action_name, params=None):
        """
        Returns a durative action as a PlanSys2 DurativeAction message with five Tree fields:
        at_start_requirements, over_all_requirements, at_end_requirements,
        at_start_effects, at_end_effects.
        UPF conditions are indexed by time intervals; these are classified into start/over_all/end.
        Returns None if the action does not exist.
        """
        for action_obj in self.domain.actions:
            if isinstance(action_obj, InstantaneousAction):
                continue
            action_str = str(action_obj)
            before_paren, rest = action_str.split("(", 1)
            name = before_paren.split(" ")[-1]
            if name != action_name:
                continue

            params_action = re.split(r"\s*,\s*", rest.split(")")[0])
            parameters, params_list, param_index_map = self._build_params(
                params_action, action_obj.parameters, params
            )

            action_return = DurativeAction()
            action_return.name = name
            action_return.parameters = parameters

            action_return.at_start_requirements = Tree()
            make_node(action_return.at_start_requirements, TreeNode.AND)
            action_return.over_all_requirements = Tree()
            make_node(action_return.over_all_requirements, TreeNode.AND)
            action_return.at_end_requirements = Tree()
            make_node(action_return.at_end_requirements, TreeNode.AND)
            action_return.at_start_effects = Tree()
            make_node(action_return.at_start_effects, TreeNode.AND)
            action_return.at_end_effects = Tree()
            make_node(action_return.at_end_effects, TreeNode.AND)

            for interval, conds in action_obj._conditions.items():
                initial_time = str(interval.lower)
                final_time = str(interval.upper)
                tree = preconditions_to_tree(conds, params_list, param_index_map)
                if initial_time == final_time == 'start':
                    self.merge_trees(action_return.at_start_requirements, tree)
                elif initial_time == 'start' and final_time == 'end':
                    self.merge_trees(action_return.over_all_requirements, tree)
                elif initial_time == 'end':
                    self.merge_trees(action_return.at_end_requirements, tree)

            for time, eff in action_obj._effects.items():
                tree = effects_to_tree(eff, params_list, param_index_map)
                if str(time) == 'start':
                    self.merge_trees(action_return.at_start_effects, tree)
                elif str(time) == 'end':
                    self.merge_trees(action_return.at_end_effects, tree)

            return action_return
        return None

    def merge_trees(self, dst: Tree, src: Tree):
        """
        Merges src into dst by appending src's nodes as children of dst's root.
        Node IDs from src are offset by len(dst.nodes)-1 to avoid collisions,
        since PlanSys2 references nodes by their index in a flat list.
        """
        if src is None or not src.nodes:
            return

        if not dst.nodes:
            make_node(dst, TreeNode.AND)

        dst_root = dst.nodes[0]
        src_root = src.nodes[0]
        offset = len(dst.nodes) - 1  #skip src's root node

        for i, node in enumerate(src.nodes):
            if i == 0:
                continue
            new_node = TreeNode()
            new_node.node_type = node.node_type
            new_node.node_id = offset + i
            new_node.name = node.name
            new_node.value = node.value
            new_node.negate = node.negate
            new_node.expression_type = node.expression_type
            new_node.modifier_type = node.modifier_type
            new_node.parameters = node.parameters
            new_node.children = [(c + offset) if c > 0 else c for c in node.children]
            dst.nodes.append(new_node)

        for child in src_root.children:
            dst_root.children.append(child + offset)

    def get_children(self, type_name):
        """Returns direct subtypes of a given type, or empty list if none."""
        if self.children_map is None:
            return []
        return self.children_map.get(type_name, [])

    def _get_fluent_node(self, name, node_type):
        """
        Finds a fluent by name and returns it as a TreeNode with parameters.
        Each parameter gets a positional name (?type0, ?type1...) and its sub_types list.
        Returns None if the fluent does not exist.
        """
        name_lower = name.lower()
        for fluent in self.domain.fluents:
            if fluent.name == name_lower:
                node = TreeNode()
                node.node_type = node_type
                node.name = fluent.name
                node.parameters = []
                for arg in fluent.signature:
                    param = Param()
                    param.type = str(arg).split(" ")[0]
                    param.name = "?" + param.type + str(len(node.parameters))
                    upf_type = self._type_map.get(param.type)
                    param.sub_types = (
                        [str(c) for c in self.get_children(str(upf_type))] if upf_type else []
                    )
                    node.parameters.append(param)
                return node
        return None

    def _build_params(self, params_action, action_obj_params, params):
        """
        Builds the parameter list for a PlanSys2 action message.
        Combines formal types from the action string with concrete names (params),
        falling back to positional names (?0, ?1...) if not provided.
        Returns (list of Param, list of names, {upf_param_name: index} map).
        """
        param_index_map = {p.name: i for i, p in enumerate(action_obj_params)}
        params_list = []
        result = []
        for i, param_str in enumerate(params_action):
            param_type = re.split(r"\s+", param_str)[0]
            p = Param()
            if params is not None and i < len(params):
                p.name = params[i]
                params_list.append(params[i])
            else:
                p.name = "?" + str(i)
                params_list.append("?" + str(i))
            p.type = param_type
            upf_type = self._type_map.get(param_type)
            p.sub_types = [str(c) for c in self.get_children(str(upf_type))] if upf_type else []
            result.append(p)
        return result, params_list, param_index_map

    def _build_maps(self):
        """
        Builds _type_map and children_map from the loaded domain.
        Called after every load or extend operation.
        """
        children = defaultdict(list)
        self._type_map = {}
        for t in self.domain.user_types:
            self._type_map[t.name] = t
            if t.father is not None:
                children[t.father.name].append(t.name)
        self.children_map = children


def _build_tree_node(expr, tree: Tree, params, param_index_map) -> int:
    """
    Recursively converts a UPF FNode into a PlanSys2 Tree node.
    Logical operators -> AND/OR/NOT nodes.
    Arithmetic/comparison operators -> EXPRESSION nodes with expression_type set.
    Numeric constants -> NUMBER nodes.
    Boolean fluents -> PREDICATE; numeric fluents -> FUNCTION.
    Returns the node_id of the created node.
    """
    if expr.is_and():
        node = make_node(tree, TreeNode.AND)
        for arg in expr.args:
            tree.nodes[node.node_id].children.append(_build_tree_node(arg, tree, params, param_index_map))
        return node.node_id

    if expr.is_or():
        node = make_node(tree, TreeNode.OR)
        for arg in expr.args:
            tree.nodes[node.node_id].children.append(_build_tree_node(arg, tree, params, param_index_map))
        return node.node_id

    if expr.is_not():
        node = make_node(tree, TreeNode.NOT)
        tree.nodes[node.node_id].children.append(
            _build_tree_node(expr.arg(0), tree, params, param_index_map)
        )
        return node.node_id

    for method_name, expr_type in _EXPR_TYPE_CHECKS:
        if getattr(expr, method_name)():
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=expr_type)
            for a in expr.args:
                tree.nodes[node.node_id].children.append(_build_tree_node(a, tree, params, param_index_map))
            return node.node_id

    if expr.is_int_constant():
        return make_node(tree, TreeNode.NUMBER, value=float(expr.int_constant_value())).node_id

    if expr.is_real_constant():
        return make_node(tree, TreeNode.NUMBER, value=float(expr.real_constant_value())).node_id

    if isinstance(expr, FNode):
        fluent_name = expr.fluent().name if expr.is_fluent_exp() else str(expr)
        node_type = TreeNode.PREDICATE if expr.type.is_bool_type() else TreeNode.FUNCTION
        node = make_node(tree, node_type, name=fluent_name)

        for a in expr.args:
            param = Param()
            if a.is_parameter_exp():
                index = param_index_map.get(a.parameter().name, -1)
            else:
                index = -1
            param.name = params[index] if index != -1 and index < len(params) else str(a)
            param.type = str(a.type)
            param.sub_types = []
            node.parameters.append(param)

        return node.node_id

    raise NotImplementedError(f"Unsupported expression type: {expr}")


def preconditions_to_tree(expr, params, param_index_map) -> Tree:
    """
    Converts a list of UPF preconditions into a PlanSys2 Tree with an AND root.
    Top-level AND expressions are flattened into the root to avoid unnecessary nesting.
    """
    tree = Tree()
    root = make_node(tree, TreeNode.AND)

    exprs = expr if isinstance(expr, list) else [expr]
    for e in exprs:
        if e.is_and():
            for arg in e.args:
                tree.nodes[root.node_id].children.append(_build_tree_node(arg, tree, params, param_index_map))
        else:
            tree.nodes[root.node_id].children.append(_build_tree_node(e, tree, params, param_index_map))

    return tree


def make_node(tree, node_type, name="", value=0.0, negate=False, parameters=None, expression_type=0, modifier_type=0):
    """
    Creates a TreeNode and appends it to the tree.
    node_id is assigned as the current length of tree.nodes, ensuring unique consecutive IDs.
    """
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


def make_predicate_node(tree, fluent, params):
    """
    Creates a PREDICATE node from an instantiated UPF fluent.
    Resolves each argument to a positional name (?index) by looking it up in params.
    """
    parameters = []
    for a in fluent.args:
        if a.is_object_exp():
            arg_name = a.object().name
        elif a.is_parameter_exp():
            arg_name = a.parameter().name
        else:
            arg_name = str(a)
        index = next((i for i, item in enumerate(params) if item == arg_name), -1)
        param = Param()
        param.name = f"?{index}"
        param.type = str(a.type)
        param.sub_types = []
        parameters.append(param)
    return make_node(tree, TreeNode.PREDICATE, name=str(fluent), parameters=parameters)


def effects_to_tree(effects, params, param_index_map) -> Tree:
    """
    Converts a list of UPF effects into a PlanSys2 Tree with an AND root.
    True effects -> PREDICATE child.
    False effects -> NOT > PREDICATE.
    Numeric modifiers (increase/decrease/assign) -> FUNCTION_MODIFIER with fluent and value as children.
    """
    tree = Tree()
    root = make_node(tree, TreeNode.AND)

    for eff in effects:
        if eff.value.is_true():
            cid = _build_tree_node(eff.fluent, tree, params, param_index_map)
            tree.nodes[root.node_id].children.append(cid)
        elif eff.value.is_false():
            not_node = make_node(tree, TreeNode.NOT)
            cid = _build_tree_node(eff.fluent, tree, params, param_index_map)
            tree.nodes[not_node.node_id].children.append(cid)
            tree.nodes[root.node_id].children.append(not_node.node_id)
        elif eff.kind in _EFFECT_MODIFIER_MAP:
            node = make_node(tree, TreeNode.FUNCTION_MODIFIER, modifier_type=_EFFECT_MODIFIER_MAP[eff.kind])
            tree.nodes[root.node_id].children.append(node.node_id)
            fid = _build_tree_node(eff.fluent, tree, params, param_index_map)
            vid = _build_tree_node(eff.value, tree, params, param_index_map)
            tree.nodes[node.node_id].children.extend([fid, vid])

    return tree


def remove_empty_sections(pddl: str) -> str:
    """
    Removes empty PDDL sections (e.g. (:requirements) with no content).
    These cause parse errors in the UPF PDDLReader.
    """
    pattern = r'\(\s*:[a-zA-Z0-9_-]+(?:\s|;[^\n]*\n)*\s*\)'

    def is_empty(match):
        block = match.group(0)
        block_no_comments = re.sub(r";[^\n]*", "", block)
        inside = re.sub(r'^\(\s*:[a-zA-Z0-9_-]+', '', block_no_comments)
        inside = inside[:-1]
        return inside.strip() == ""

    def replacer(match):
        return "" if is_empty(match) else match.group(0)

    return re.sub(pattern, replacer, pddl)


def remove_comments(pddl: str) -> str:
    """Removes PDDL comments (semicolon to end of line)."""
    return re.sub(r";[^\n]*", "", pddl)
