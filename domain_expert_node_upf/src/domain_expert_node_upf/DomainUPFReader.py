from unified_planning.model import *
from unified_planning.io import PDDLReader
from dataclasses import dataclass, field
from plansys2_msgs.msg import Action, Param, Tree, DurativeAction
from plansys2_msgs.msg import Node as TreeNode
from rclpy.node import Node
import re
from typing import List
import sys
import re
import os
import inspect
import importlib.util
import unified_planning.model as up_model
from collections import defaultdict

from unified_planning.model.timing import (
    TimeInterval,
    TimePointInterval,
    OpenTimeInterval,
    StartTiming,
    EndTiming,
)

from unified_planning.model.timing import StartTiming, EndTiming, ClosedTimeInterval


@dataclass

class DomainUPFReader:
    def __init__(self, logger=None):
        self.domain = None
        self.childrens = None
        if logger is None:
            self.debug = False
            self.logger = print
        else:
            self.debug = True
            self.logger = logger

    def load_pddl(self, pddl_path):
        reader = PDDLReader()
        try:
            self.domain = reader.parse_problem(pddl_path)
            self.children_map = self._build_children_map()
            return self.domain
        except Exception as e:
            print(f"Error al cargar el archivo PDDL: {e}", file=sys.stderr)


    def get_name(self):
        return str(self.domain.name) if self.domain else ""
    
    def add_domain(self, domain: Problem):

        if not domain:
            print('Empty domain', file=sys.stderr)
            return

        new_domain = Domain()

        new_domain.domain = self._execute_upf_problem(domain)

        new_domain.name = self.get_name(new_domain.domain)
        new_domain.requirements = self.get_requirements(new_domain.domain)
        new_domain.types = self.get_types(new_domain.domain)
        new_domain.constants = self.get_constants(new_domain.domain)
        new_domain.predicates = self.get_predicates(new_domain.domain)
        new_domain.functions = self.get_functions(new_domain.domain)
        new_domain.derived_predicates = self.get_derived_predicates(new_domain.domain)
        new_domain.actions = self.get_actions(new_domain.domain)

        self._domains.append(new_domain)

        return new_domain

    def get_joint_domain(self): # combinar múltiples dominios pddl en un único dominio.
        ret = ''
        
        pass

    def get_end_block(self):
        ## parser(?)
        pass

    # def get_name(self, domain: Problem):
    #     # match = re.search(r'problem\s*\(\s*["\'](.*?)["\']', domain)
        
    #     # if match:
    #     #     return match.group(1)
    #     # else:
    #     #     return ""
    #     return domain.name

    def get_requirements(self): # en upf .kind
        # if not domain:
        #     return []
        
        return self.domain.kind

    def get_types(self): # usertypes
        # if not domain:
        #     return []
        
        return [str(x) for x in self.domain.user_types]
    
    def get_constants(self, domain): # object
        if not domain:
            return []
        
        return self.domain.all_objects

    def get_predicates(self): # Fluent con booltype
        nodes = []
        for fluent in self.domain.fluents:
            if fluent.type.is_bool_type():
                node = TreeNode()
                node.node_type = TreeNode.PREDICATE
                node.name = str(fluent._name)
                nodes.append(node)

        return nodes

    def get_functions(self): # fluent con realtype
        nodes =  []
        for fluent in self.domain.fluents:
            if not fluent.type.is_bool_type():
                node = TreeNode()
                node.node_type = TreeNode.FUNCTION
                node.name = str(fluent._name)
                nodes.append(node)

        return nodes
    
    def get_predicate(self, name):

        for predicate in self.domain.fluents:
            if predicate._name == name:
                node = TreeNode()
                node.node_type = TreeNode.PREDICATE
                node.name = str(predicate._name)
                node.parameters = []
                print(vars(predicate))
                for arg in predicate._signature:
                    param = Param()
                    param.name = "?" + f'{len(node.parameters)}'
                    param.type = str(arg).split(" ")[0]
                    param.sub_types = []
                    node.parameters.append(param)

                return node
            
        return None
          
    def get_function(self, name):
        for predicate in self.domain.fluents:
            if predicate._name == name:
                node = TreeNode()
                node.node_type = TreeNode.FUNCTION
                node.name = str(predicate._name)
                node.parameters = []
                print(vars(predicate))
                for arg in predicate._signature:
                    param = Param()
                    param.name = "?" + f'{len(node.parameters)}'
                    param.type = str(arg).split(" ")[0]
                    param.sub_types = []
                    node.parameters.append(param)

                return node
            
        return None
      
    def get_domain(self):
        return str(self.domain)

    def get_derived_predicates(self): 
        '''
        reachable = Fluent("reachable", BoolType(), [room_t, room_t])
        x = Variable("x", room_t)
        y = Variable("y", room_t)
        z = Variable("z", room_t)

        problem.add_derived_fluent(
            reachable(x, y),
            Or(connected(x, y), Exists([z], And(connected(x, z), reachable(z, y))))
        )'''
 
        return self.domain.derived_predicates

    def get_actions(self):
        '''
        InstantaneousAction
        DurativeAction
        Task
        '''
    
        actions = [
            str(a) for a in self.domain.actions
            if isinstance(a, (InstantaneousAction))
        ]

        self.logger.warn(f"actions: {actions}")
        self.logger.warn(f"{self.domain.actions}")

        return actions

        #return [str(action) for action in self.domain.actions]

    def get_type_by_name(self, type_name):
        for t in self.domain.user_types:
            if t.name == type_name:
                return t
        return None


    def get_action(self, action_name, params):
        
        for action_obj in self.domain.actions:
            action = str(action_obj)
            
            name = action.split("(")[0].split(" ")[-1]
            params_action = re.split(r"\s*,\s*", action.split("(")[1].split(")")[0]) # revisar si eso es lo que quiero
            self.logger.warn(f"name: i{name}i")
            
            if name != action_name:
                self.logger.warn(f"name: i{name}i != action_name: i{action_name}i")
                continue
            

            if params and (len(params) != len(params_action)):
                self.logger.info(f"params: {len(params)}")
                self.logger.warn(f"params_action: {len(params_action)}")

                continue
            
            if params:
                params_action_name = [re.split(r"\s+", param_a)[1] for param_a in params_action]    
                self.logger.info(f"params_action_name: {params_action_name}")

                for param in params:
                    if param not in params_action_name:
                        self.logger.warn(f"param: i{param}i not in params_action: {params_action_name}")
                        return

            same_params = True
            params_list = []

            for param in params:
                self.logger.info(f"param: {param}")

                if param not in params_action:
                    self.logger.warn(f"param: i{param}i not in params_action: {params_action}")
                    same_params = False
                    break
            
            if same_params:
                self.logger.info(f"action: {action}")
                self.logger.info(f"name: {name}")
                action_return = Action()
                self.logger.info(f"action_return:")
                action_return.name = name
                action_return.parameters = []
                self.logger.info(f"action_return:")
                
                for param in params_action:
                    param_list = re.split(r"\s+", param)
                    self.logger.info(f"param_list: {param_list}")

                    print('added to the list: ', param_list[1])
                    params_list.append(param_list[1])

                    param_return = Param()
                    param_return.name = "?" + f'{len(params_list) - 1}'
                    param_return.type = str(param_list[0])

                    upf_type = self.get_type_by_name(param_list[0])
                    print(upf_type)
                    
                    if upf_type is None:
                        param_return.sub_types = []
                    else:
                        param_return.sub_types = [child.name for child in self.get_children(upf_type.name)]

                        self.logger.info(f"param_return children: {param_return.sub_types}")

                    action_return.parameters.append(param_return)

                print(action_obj)

                print(type(action_obj))
                action_return.preconditions = preconditions_to_tree(action_obj._preconditions, params_list)   # FNode
                #eff_tree = effects_to_tree(effects) 

                # effects
                print('\n\n\n--------------------EFFECTS-------------------\n\n\n')
                action_return.effects = effects_to_tree(action_obj.effects, params_list)
                print('sale de los efectos')
                #action_return
                return action_return
            
        return None

    def get_durative_actions(self):
        actions = [
            str(a) for a in self.domain.actions
            if isinstance(a, (DurativeAction))
        ]

        self.logger.warn(f"actions: {actions}")
        self.logger.warn(f"{self.domain.actions}")

        return actions

    def get_durative_action(self, action_name, params):

        self.logger.info(f'action name: {action_name}')

        self.logger.info(f"{self.domain.actions}")

        for action_obj in self.domain.actions:

            # if not isinstance(action_obj, (DurativeAction)):
            #     continue
            action = str(action_obj)

            self.logger.info(f'vars(action_obj)')
            
            name = action.split("(")[0].split(" ")[-1]
            params_action = re.split(r"\s*,\s*", action.split("(")[1].split(")")[0])
            self.logger.warn(f"name: i{name}i")


            if name != action_name:
                self.logger.warn(f"name: i{name}i != action_name: i{action_name}i")
                continue
            
            if params and (len(params) != len(params_action)):
                self.logger.info(f"params: {len(params)}")
                self.logger.warn(f"params_action: {len(params_action)}")

                continue
            
            self.logger.info("llega")

            if params:
                params_action_name = [re.split(r"\s+", param_a)[1] for param_a in params_action]    
                self.logger.info(f"params_action_name: {params_action_name}")

                for param in params:
                    if param not in params_action_name:
                        self.logger.warn(f"param: i{param}i not in params_action: {params_action_name}")
                        return

            self.logger.info(f"params: {params}")
            
            same_params = True
            params_list = []

            for param in params:
                self.logger.info(f"param: {param}")

                if param not in params_action:
                    self.logger.warn(f"param: i{param}i not in params_action: {params_action}")
                    same_params = False
                    break

            self.logger.info(f"same_params: {same_params}")

            if same_params:
                self.logger.info(f"action: {action}")
                self.logger.info(f"name: {name}")
                action_return = DurativeAction()
                self.logger.info(f"action_return:")
                action_return.name = name
                action_return.parameters = []
                self.logger.info(f"action_return:")
                
                for param in params_action:
                    param_list = re.split(r"\s+", param)
                    self.logger.info(f"param_list: {param_list}")

                    print('added to the list: ', param_list[1])
                    params_list.append(param_list[1])

                    param_return = Param()
                    param_return.name = "?" + f'{len(params_list) - 1}'
                    param_return.type = str(param_list[0])

                    upf_type = self.get_type_by_name(param_list[0])
                    print(upf_type)
                    
                    if upf_type is None:
                        param_return.sub_types = []
                    else:
                        param_return.sub_types = [child.name for child in self.get_children(upf_type.name)]

                        self.logger.info(f"param_return children: {param_return.sub_types}")

                    action_return.parameters.append(param_return)

                print(action_obj)

                self.logger.info(f"action_obj:")
                self.logger.info(f'{type(action_obj)}')
                self.logger.info(f'{vars(action_obj)}')

                self.logger.info(f'{action_obj.conditions}')
                self.logger.info(f'{type(action_obj.conditions)}')
                #self.logger.info(f'{vars(action_obj.conditions)}')

                print("TimePointInterval =", TimePointInterval)
                print("type(TimePointInterval) =", type(TimePointInterval))


                START = type(StartTiming())
                END = type(EndTiming())

                for interval, conds in action_obj._conditions.items():

                    lower = interval.lower
                    upper = interval.upper

                    print(lower)
                    print(upper)
                    print(str(lower), str(upper))
                    initial_time = str(interval.lower)
                    final_time = str(interval.upper) 
                    print(conds)

                    if initial_time == final_time and initial_time == 'start':
                        print('entra en el if 1')
                        print(">>> AT START")
                        action_return.at_start_requirements = preconditions_to_tree(conds, params_list)

                    elif initial_time == 'start' and final_time == 'end':
                        print('entra en el if 2')
                        print(">>> OVER ALL")
                        action_return.over_all_requirements = preconditions_to_tree(conds, params_list)

                    elif initial_time == 'end':
                        print(">>> AT END")
                        action_return.at_end_requirements = preconditions_to_tree(conds, params_list)

                return action_return
        
        pass

    def _execute_upf_problem(self, module_path):
        if not os.path.exists(module_path):
            raise FileNotFoundError(f"No existe el archivo: {module_path}")
        
        module_name = os.path.splitext(os.path.basename(module_path))[0]
        spec = importlib.util.spec_from_file_location(module_name, module_path)
        module = importlib.util.module_from_spec(spec)

        found_problems = []
        original_problem_class = Problem

        class ProblemTracker(original_problem_class):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                if self not in found_problems:
                    found_problems.append(self)
                    print(f"[DEBUG] Problem creado dinámicamente: {self.name}")

        up_model.Problem = ProblemTracker

        try:
            spec.loader.exec_module(module)
        finally:
            up_model.Problem = original_problem_class

        for name, value in inspect.getmembers(module):
            if isinstance(value, original_problem_class) and value not in found_problems:
                print(f"[INFO] Problema encontrado como variable global '{name}'")
                found_problems.append(value)

        for name, func in inspect.getmembers(module, inspect.isfunction):
            try:
                sig = inspect.signature(func)
                if len(sig.parameters) == 0:
                    result = func()
                    if isinstance(result, original_problem_class) and result not in found_problems:
                        print(f"[INFO] Problema devuelto por función '{name}()'")
                        found_problems.append(result)
            except Exception as e:
                print(f"[WARN] No se pudo ejecutar '{name}()': {e}")

        if not found_problems:
            print('There is not a UPF domain in file {domain}', file=sys.stderr)
            return
        
        return found_problems[0]
    
    def get_tree(self, action_string):
        tree = Tree()

        
        node = Node()
        return tree

    def get_children(self, type_name):
        print(type_name)
        if self.children_map is None:
            return []
        return self.children_map.get(type_name, [])
 
    def _build_children_map(self):
 
        children = defaultdict(list)

 
        for t in self.domain.user_types:
            if t.father is not None:
                children[t.father].append(t)

                children[t.father.name].append(t)
 
        for parent, childs in children.items():
            print(parent, " => hijos:", childs)
        
        return children

    def childrens(self):

        children = defaultdict(list)

        for t in self.domain.user_types:
            if t.father is not None:
                children[t.father].append(t)

        for parent, childs in children.items():
            print(parent, " => hijos:", childs)
        
        return children


class NodeType:
    UNKNOWN = 0
    AND = 1
    OR = 2
    NOT = 3
    ACTION = 4
    PREDICATE = 5
    FUNCTION = 6
    EXPRESSION = 7
    FUNCTION_MODIFIER = 8
    NUMBER = 9
    CONSTANT = 10
    PARAMETER = 11
    EXISTS = 12



def _build_tree_node(expr, tree:Tree, params) -> int:
    """Crea nodos Tree.Node recursivamente igual que get_tree de C++, 
    pero en una sola función y para UPF."""

    # ---------- Operadores lógicos ----------
    print(expr)
    print(type(expr))

    attrs = [
        '_content', '_env', '_node_id', 'agent', 'arg', 'args', 'bool_constant_value',
        'constant_value', 'environment', 'fluent', 'get_contained_names',
        'get_nary_expression_string', 'int_constant_value', 'is_always', 'is_and',
        'is_at_most_once', 'is_bool_constant', 'is_constant', 'is_div', 'is_dot',
        'is_equals', 'is_exists', 'is_false', 'is_fluent_exp', 'is_forall', 'is_iff',
        'is_implies', 'is_int_constant', 'is_le', 'is_lt', 'is_minus', 'is_not',
        'is_object_exp', 'is_or', 'is_parameter_exp', 'is_plus', 'is_real_constant',
        'is_sometime', 'is_sometime_after', 'is_sometime_before', 'is_times',
        'is_timing_exp', 'is_true', 'is_variable_exp', 'node_id', 'node_type',
        'object', 'parameter', 'real_constant_value', 'simplify', 'substitute',
        'timing', 'type', 'variable', 'variables'
    ]

    print(type(expr))
    print(expr)

    if tree.nodes is None or len(tree.nodes) == 0:
        print('-----------------------AND EXPR-----------------------')
        node = TreeNode()
        node.node_id = len(tree.nodes)
        node.node_type = NodeType.AND
        node.expression_type = 0
        node.modifier_type = 0
        node.name = ''
        node.parameters = []
        node.value = 0.0
        node.negate = False
        node.children = []
        tree.nodes.append(node)

        if not isinstance(expr, list):
            print('is not a list instance')
            expr = [expr]

        for arg in expr:
            cid =_build_tree_node(arg, tree, params)
            if cid == 0:
                continue
            
            tree.nodes[node.node_id].children.append(cid)
        
        return node.node_id


    
    print('\n\n\n--------------------EXPR-------------------')
    print(expr)
    print(type(expr))


    if expr.is_and():
        if len(tree.nodes) != 1:
            print('entra and, len != 1')
            node = TreeNode()
            node.node_id = len(tree.nodes)
            node.node_type = NodeType.AND
            node.expression_type = 0
            node.modifier_type = 0
            node.name = ''
            node.parameters = []
            node.value = 0.0
            node.negate = False
            node.children = []
            tree.nodes.append(node)
        else:
            print('entra and, len == 1')
            node = tree.nodes[0]

        for arg in expr.args:
            cid =_build_tree_node(arg, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        
        return node.node_id

    if expr.is_or():
        node = TreeNode()
        node.node_id = len(tree.nodes)
        node.node_type = NodeType.OR
        node.expression_type = 0
        node.modifier_type = 0
        node.name = ''
        node.parameters = []
        node.value = 0.0
        node.negate = False
        node.children = []
        tree.nodes.append(node)
        for arg in expr.args:
            cid = _build_tree_node(arg, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        return node.node_id

    if expr.is_not():
        print('------------------entra not--------------------')
        node = TreeNode()
        node.node_id = len(tree.nodes)
        node.node_type = NodeType.NOT
        node.children = []
        tree.nodes.append(node)
        cid = _build_tree_node(expr.arg(0), tree, params)
        tree.nodes[node.node_id].children.append(cid)
        return node.node_id

    if expr.is_le() or expr.is_lt() or expr.is_equals():
        node = TreeNode()
        node.node_id = len(tree.nodes)
        node.node_type = TreeNode.EXPRESSION
        
        if expr.is_le(): node.expression_type = TreeNode.COMP_LE
        elif expr.is_lt(): node.expression_type = TreeNode.COMP_LT
        elif expr.is_equals(): node.expression_type = TreeNode.COMP_EQ
        
        node.children = []
        tree.nodes.append(node)
        for a in expr.args:
            cid = _build_tree_node(a, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        return node.node_id
        
    if expr.is_div() or expr.is_times() or expr.is_plus() or expr.is_minus():
        print("----------ENTRA DOT----------")
        node = TreeNode()
        node.node_id = len(tree.nodes)
        node.node_type = TreeNode.EXPRESSION
        if expr.is_div(): node.expression_type = TreeNode.ARITH_DIV
        elif expr.is_times(): node.expression_type = TreeNode.ARITH_MULT
        elif expr.is_plus(): node.expression_type = TreeNode.ARITH_ADD
        elif expr.is_minus(): node.expression_type = TreeNode.ARITH_SUB
        
        tree.nodes.append(node)
        node.children = []
        for a in expr.args:
            cid = _build_tree_node(a, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        return node.node_id

    if expr.is_int_constant() or expr.is_real_constant():
        print("-------------entra number-------------")
        node = TreeNode()
        node.node_id = len(tree.nodes)
        node.node_type = TreeNode.NUMBER

        if expr.is_int_constant():
            node.value = float(expr.int_constant_value())
        else:
            node.value = float(expr.real_constant_value())

        tree.nodes.append(node)
        return node.node_id

    if type(expr) == FNode:
        print(NumericExpression)
        fluent_name = str(expr).split("(")[0]
        print(fluent_name)

        node = TreeNode()

        if expr.type.is_bool_type():
            print('entra predicate')
            node.node_type = TreeNode.PREDICATE
        else:
            print('entra function')
            node.node_type = TreeNode.FUNCTION

        node.node_id = len(tree.nodes)
        node.expression_type = 0
        node.modifier_type = 0
        node.name = fluent_name
        node.children = []
        node.parameters = []
        for a in expr.args:
            param = Param()
            print(a)
            print(params)
            index = next((i for i, item in enumerate(params) if item == str(a)), -1)
            param.name = "?" + f'{index}'
            param.type = str(a.type)
            param.sub_types = []
            node.parameters.append(param)

        tree.nodes.append(node)
        return node.node_id

    raise NotImplementedError(f"No está implementado este tipo de nodo UPF: {expr}")

def preconditions_to_tree(expr, params) -> Tree:
    tree = Tree()
    _build_tree_node(expr, tree, params)   # genera nodos y pone root
    return tree

def make_node(node_type, name="", value=0.0, negate=False, parameters=None, expression_type=0, modifier_type=0):
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

def make_predicate_node(fluent, params):
    parameters = []
    for a in fluent.args:
        index = next((i for i, item in enumerate(params) if item == str(a)), -1)
        param = Param()
        param.name = f"?{index}"
        param.type = str(a.type)
        param.sub_types = []
        parameters.append(param)
    return make_node(TreeNode.PREDICATE, name=str(fluent), parameters=parameters)

def effects_to_tree(effects, params) -> Tree:
    global tree
    tree = Tree()

    and_node = make_node(TreeNode.AND)
    print(effects)

    for eff in effects:
        print(eff)
        print(type(eff))
        print(vars(eff))

        if eff.value.is_false():
            not_node = make_node(TreeNode.NOT)
            and_node.children.append(not_node.node_id)

            child_node = make_predicate_node(eff.fluent, params)
            not_node.children.append(child_node.node_id)
        
        elif eff.value.is_true():
            child_node = make_predicate_node(eff.fluent, params)
            and_node.children.append(child_node.node_id)

        elif eff._kind == EffectKind.INCREASE or eff._kind == EffectKind.DECREASE or eff._kind == EffectKind.ASSIGN:
            
            print("entra modificadores")
            if eff._kind == EffectKind.INCREASE:
                node = make_node(TreeNode.FUNCTION_MODIFIER, modifier_type=TreeNode.INCREASE)
            elif eff._kind == EffectKind.DECREASE:
                node = make_node(TreeNode.FUNCTION_MODIFIER, modifier_type=TreeNode.DECREASE)
            elif eff._kind == EffectKind.ASSIGN:
                node = make_node(TreeNode.FUNCTION_MODIFIER, modifier_type=TreeNode.ASSIGN)

            and_node.children.append(node.node_id)

            id = _build_tree_node(eff._fluent, tree, params)
            
            node.children.append(id)

            id = _build_tree_node(eff._value, tree, params)

            node.children.append(id)

            print(type(eff._value))

            

    return tree


        # if expr.is_bool_constant():
        #     node.node_type = Node.PREDICATE
        #     node.type = str(expr.bool_constant_value())
        # pass