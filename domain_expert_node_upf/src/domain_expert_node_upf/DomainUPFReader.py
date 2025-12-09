from unified_planning.model import *
from unified_planning.io import PDDLReader
from dataclasses import dataclass, field
from plansys2_msgs.msg import Action, Param, Tree
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
        # if not domain:
        #     return []
        
        return [f for f in self.domain.fluents if f.type.is_bool_type()]

    def get_functions(self): # fluent con realtype
        # if not domain:
        #     return []
        
        return [f for f in self.domain.fluents if f.type.is_numeric_type()]
    
    def get_domain(self):
        return str(self.domain) #??


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

    def get_durative_action(self):
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
    
# class ParamExpression():

    # NODE_TYPES = {
    #     "and": Node.AND,
    #     "or": Node.OR,
    #     "not": Node.NOT,
    #     "predicate": Node.PREDICATE,
    #     "function": Node.FUNCTION,
    #     "equals": Node.EQUALS,
    # }

    # def __init__():
    #     pass
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

    if not isinstance(expr, list):
        expr = [expr]

    for x in expr:
        print('\n\n\n--------------------EXPR-------------------')
        print(x)
        print(type(x))


        if x.is_and():
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

            for arg in x.args:
                cid =_build_tree_node(arg, tree, params)
                tree.nodes[node.node_id].children.append(cid)
            
            return node.node_id

        if x.is_or():
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
            for arg in x.args:
                cid = _build_tree_node(arg, tree, params)
                tree.nodes[node.node_id].children.append(cid)
            return node.node_id

        if x.is_not():
            node = TreeNode()
            node.node_id = len(tree.nodes)
            node.node_type = NodeType.NOT
            node.children = []
            tree.nodes.append(node)
            cid = _build_tree_node(x.arg(0), tree, params)
            tree.nodes[node.node_id].children.append(cid)
            return node.node_id

        if x.is_le() or x.is_lt() or x.is_equals():
            node = TreeNode()
            node.node_id = len(tree.nodes)
            node.node_type = TreeNode.EXPRESSION
            
            if x.is_le(): node.expression_type = TreeNode.COMP_LE
            elif x.is_lt(): node.expression_type = TreeNode.COMP_LT
            elif x.is_equals(): node.expression_type = TreeNode.COMP_EQ
            
            node.children = []
            tree.nodes.append(node)
            for a in x.args:
                cid = _build_tree_node(a, tree, params)
                tree.nodes[node.node_id].children.append(cid)
            return node.node_id
            
        if x.is_div() or x.is_times() or x.is_plus() or x.is_minus():
            print("----------ENTRA DOT----------")
            node = TreeNode()
            node.node_id = len(tree.nodes)
            node.node_type = TreeNode.EXPRESSION
            if x.is_div(): node.expression_type = TreeNode.DIV
            elif x.is_times(): node.expression_type = TreeNode.MULT
            elif x.is_plus(): node.expression_type = TreeNode.ADD
            elif x.is_minus(): node.expression_type = TreeNode.SUB
            
            tree.nodes.append(node)
            node.children = []
            for a in x.args:
                cid = _build_tree_node(a, tree, params)
                tree.nodes[node.node_id].children.append(cid)
            return node.node_id

        if x.is_int_constant() or x.is_real_constant():
            print("-------------entra number-------------")
            node = TreeNode()
            node.node_id = len(tree.nodes)
            node.node_type = TreeNode.NUMBER

            if x.is_int_constant():
                node.value = float(x.int_constant_value())
            else:
                node.value = float(x.real_constant_value())

            tree.nodes.append(node)
            return node.node_id

        if type(x) == FNode:
            print(x)
            fluent_name = str(x).split("(")[0]
            print(fluent_name)

            node = TreeNode()

            if x.type.is_bool_type():
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
            for a in x.args:
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

# def effects_to_tree(effects, params) -> Tree:
#     tree = Tree()
    

#     print(effects)
#     print(type(effects))
#     #print(vars(effects))

#     and_node = TreeNode()
#     and_node.node_id = len(tree.nodes)
#     and_node.node_type = TreeNode.AND
#     and_node.name = ""
#     and_node.parameters = []
#     and_node.value = 0.0
#     and_node.negate = False
#     and_node.children = []
#     tree.nodes.append(and_node)

#     for eff in effects:
#         print(eff)
#         print(type(eff))
#         print(vars(eff))
#         print(eff._fluent)
#         print("\n\n\n")

#         if eff._value.is_false():
#             print('falso')
#             node = TreeNode()
#             node.node_id = len(tree.nodes)
#             node.node_type = TreeNode.NOT
#             node.name = ""
#             node.parameters = []
#             node.value = 0.0
#             node.negate = False
#             node.children = []
#             tree.nodes.append(node)
#             tree.nodes[and_node.node_id].children.append(node.node_id)


#             child_node = TreeNode()
#             child_node.node_id = len(tree.nodes)
#             child_node.node_type = TreeNode.PREDICATE
#             child_node.name = str(eff._fluent)
#             child_node.value = 0.0
#             child_node.negate = False
#             child_node.parameters = []
#             for a in eff._fluent.args:
#                 param = Param()
#                 index = next((i for i, item in enumerate(params) if item == str(a)), -1)
#                 param.name = "?" + f'{index}'
#                 param.type = str(a.type)
#                 param.sub_types = []
#                 child_node.parameters.append(param)
#             tree.nodes.append(child_node)
#             tree.nodes[node.node_id].children.append(child_node.node_id)
#             continue

#         else:
#             print(eff._value)
#             print('verdadero')
#             child_node = TreeNode()
#             child_node.node_id = len(tree.nodes)
#             child_node.node_type = TreeNode.PREDICATE
#             child_node.name = str(eff._fluent)
#             child_node.value = 0.0
#             child_node.negate = False
#             child_node.parameters = []
#             for a in eff._fluent.args:
#                 param = Param()
#                 index = next((i for i, item in enumerate(params) if item == str(a)), -1)
#                 param.name = "?" + f'{index}'
#                 print(a.type)
#                 param.type = str(a.type)
#                 param.sub_types = []
#                 child_node.parameters.append(param)
#             tree.nodes.append(child_node)
#             tree.nodes[and_node.node_id].children.append(child_node.node_id)

#     return tree

def make_node(node_type, name="", value=0.0, negate=False, parameters=None):
    node = TreeNode()
    node.node_id = len(tree.nodes)
    node.node_type = node_type
    node.name = name
    node.value = value
    node.negate = negate
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
        if eff.value.is_false():
            not_node = make_node(TreeNode.NOT)
            and_node.children.append(not_node.node_id)

            child_node = make_predicate_node(eff.fluent, params)
            not_node.children.append(child_node.node_id)
        
        else:
            child_node = make_predicate_node(eff.fluent, params)
            and_node.children.append(child_node.node_id)

    return tree


        # if expr.is_bool_constant():
        #     node.node_type = Node.PREDICATE
        #     node.type = str(expr.bool_constant_value())
        # pass