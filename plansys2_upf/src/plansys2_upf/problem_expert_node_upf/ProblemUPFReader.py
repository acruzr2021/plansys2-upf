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
from unified_planning.shortcuts import And, Or, Not
from unified_planning.shortcuts import FluentExp
from unified_planning.model.timing import StartTiming, EndTiming, ClosedTimeInterval

class ProblemUPFExpert():
    def __init__(self, domain_path=None):
        self.problem = None
        self.domain = None
        with open(domain_path, "r") as f:
            self.domain_pddl = f.read()
        self.goal = None
        self.instances = []
        self.predicates = []
        self.functions = []

    def add_problem(self, problem_str):

        if problem_str is None:
            return None
        
        reader = PDDLReader()

        try:
            self.problem = reader.parse_problem_string(self.domain_pddl, problem_str)
            self.domain = reader.parse_problem_string(self.domain_pddl)

        except Exception as e:
            print(f"Error al cargar el archivo PDDL: {e}", file=sys.stderr)

        print(self.domain_pddl)

        for instance in self.problem.all_objects:
            
            if instance.type not in self.domain.user_types:
                continue

            param = Param()
            param.name = instance.name
            param.type = instance.type.name
            param.sub_types = []

            print(f'Adding instance: {instance.name}')
            if param not in self.instances:
                self.instances.append(param)
        
        print(dir(self.problem))
        print(self.problem._initial_value)
        try: 
            for pred, val in self.problem._initial_value.items():
                try: 
                    print(f'Adding predicate/function: {pred}')
                    if pred.type.is_bool_type():
                        name = str(pred)
                        name = name.split("(")[0]
                        print(name)
                        pred_obj = TreeNode()
                        pred_obj.node_type = TreeNode.PREDICATE
                        pred_obj.name = name
                        if val == 'true':   
                            pred_obj.negate = True
                        else:
                            pred_obj.negate = False
                        pred_obj.parameters = []
                        
                        for i, arg in enumerate(pred.args):
                            print(arg)
                            print(dir(arg))
                            param = Param()
                            param.name = str(arg)
                            param.type = str(arg.type)
                            param.sub_types = []
                            pred_obj.parameters.append(param)
                            print('param: ', param)

                        print(pred_obj)
                        
                        if pred_obj not in self.predicates:
                            print('predicado para añadir')
                            self.predicates.append(pred_obj)

                    elif pred.type.is_real_type():
                        print('entra function', val, type(val))
                        name = str(pred)
                        print(f"name {name}\n")
                        name = name.split('(')[0]
                        print(name)
                        pred_obj = TreeNode(name=name.replace("(", "_").replace(")", "_").replace(",", "_").replace(" ", "_"))
                        pred_obj.node_type = TreeNode.FUNCTION
                        pred_obj.value = float(str(val))
                        pred_obj.parameters = []
                        for i, arg in enumerate(pred.args):
                            param = Param()
                            param.name = str(arg)
                            param.type = str(arg.type)
                            param.sub_types = []
                            pred_obj.parameters.append(param)
                            print('param: ', param)
                                               
                        if pred_obj not in self.predicates:
                            print('predicado para añadir')
                            self.functions.append(pred_obj)
                    else:
                        print(type(pred))
                except Exception as e:
                    print(f'error 2: {e}')
                    continue

        except Exception as e:
            print(f'error 1: {e}')

        print(self.problem._goals)
        tree_goals = Tree()

        _build_tree_node(self.problem._goals, tree_goals, [])
        print(f'goal: {tree_goals}')
        
        print(tree_goals)

        self.goal = tree_goals

        return True

    def add_instance(self, instance):
        if type(instance) != Param:
            return False
        
        available_types = [t.name for t in self.domain.user_types]
        
        if instance.type not in available_types:
            print(f"Tipo '{instance.type}' no encontrado. Tipos disponibles: {available_types}")
            return False
        
        if instance not in self.instances:
            type_obj = None
            for user_type in self.domain.user_types:
                if user_type.name == instance.type:
                    type_obj = user_type
                    break
            
            if type_obj is None:
                return False
            
            instance_obj = Object(instance.name, type_obj)
            self.problem.add_object(instance_obj)
            self.instances.append(instance)
            return True
        
        return False

    def add_predicate(self, predicate):
        if not isinstance(predicate, TreeNode):
            return False
        if predicate.node_type != TreeNode.PREDICATE:
            return False

        fluent = self._find_matching_fluent(
            predicate.name, predicate.parameters, bool_fluent=True
        )
        if fluent is None:
            return False

        if predicate in self.predicates:
            return True

        expr = self._build_upf_expression(fluent, predicate.parameters)
        if expr is None:
            return False

        self.problem.set_initial_value(expr, True)
        print(self.problem._initial_value)
        self.predicates.append(predicate)
        return True


    def _find_upf_object(self, name, type_name):
        for obj in self.problem.all_objects:
            if obj.name == name:
                
                obj_type_name = obj.type if isinstance(obj.type, str) else str(obj.type)
                if obj_type_name == type_name:
                    return obj
                print(obj_type_name, type_name)

        return None

    def add_function(self, function):
        if not isinstance(function, TreeNode):
            return False
        
        if function.node_type != TreeNode.FUNCTION:
            return False

        fluent = self._find_matching_fluent(
            function.name, function.parameters, bool_fluent=False
        )

        if fluent is None:
            return False

        if function in self.functions:
            return True

        expr = self._build_upf_expression(fluent, function.parameters)
        if expr is None:
            return False

        self.problem.set_initial_value(expr, function.value)
        print(self.problem._initial_value)
        self.functions.append(function)

        return True

    def add_goal(self, goal):
        if not isinstance(goal, Tree):
            return False

        if self.goal is not None:
            self.remove_goal()

        self._node_map = {n.node_id: n for n in goal.nodes}
        root = self._node_map[0]
        upf_goal = self._tree_to_upf(root)

        if upf_goal is None:
            return False

        self.problem.add_goal(upf_goal)
        self.goal = goal
        return True


        

    def get_goal(self):
        if self.goal is None:
            return Tree()
        
        print(self.goal, type(self.goal))      

        return self.goal

    def get_instance(self, instance_name):
        for instance in self.instances:
            if instance.name == instance_name:
                return instance
        return None


    def get_instances(self):
        return self.instances

    def get_predicate(self, predicate):

        pred_str = predicate.strip()
        if not (pred_str.startswith('(') and pred_str.endswith(')')):
            print(f"Formato inválido, debe empezar y terminar con paréntesis: {pred_str}")
            return None
        
        content = pred_str[1:-1].strip()
        parts = content.split()
        if not parts:
            print(f"Predicato vacío: {pred_str}")
            return None
        

        predicate_name = parts[0]
        print(predicate_name)
        # argumentos
        args = parts[1:]
        print(f'args: {args}')

        for predicate in self.predicates:
            print(predicate)
            print(predicate.name)

        for predicate in self.predicates:
            print(predicate, predicate_name)
            if str(predicate.name) != predicate_name:
                continue
            print('nombre coincide')
            correct_arg = 0
            for arg in args:
                for pred_arg in predicate.parameters:
                    if str(pred_arg.name) == arg:
                        print('argumento correcto')
                        correct_arg += 1
            if correct_arg == len(args):
                print('todo correcto')
                return predicate
        print('falso')
        return None


    def get_predicates(self):
        print(self.predicates)
        print(type(self.predicates))
        if type(self.predicates) == list:
            for predicate in self.predicates:
                print(predicate)
                print(type(predicate))

        return self.predicates

    def get_function(self, function):
        
        function_str = function
        if function_str.startswith('(') and function_str.endswith(')'):
            function_str = function_str[1:-1].strip()
            print('sin parentesis', function_str)
        
        val = None
        if function_str.startswith('='):
            function_str = function_str.split('(')[1]  # "(speed r2d2) 30"
            #val = function_str.split(' ')[-1]
            val = function_str.split(')')[1]
            if len(val.split(' ')) > 1:
                val = val.split(' ')[-1]
            print('value', val)
            paren_end = function_str.find(')')
            if paren_end == -1:
                print('error: invalid sintaxis')
                return None
            
            function_part = function_str[:paren_end+1]  # "(speed r2d2)"
            print(f'function i{function_part}i')
            
            if function_part.endswith(')'):
                function_part = function_part[:-1].strip()
            else:
                function_part = function_part.split(')')[1]
            function_str = function_part
                

        print(f"Procesado: {function_str}") 
        function_str = function_str.split()
        function_name = function_str[0]
        args = function_str[1:]
        print(function_name)
        print(args)

        for function in self.functions:
            print(function)
            print(function.name)
            if str(function.name) != function_name:
                continue
            print('nombre coincide')
            correct_arg = 0
            for arg in args:
                for func_arg in function.parameters:
                    if str(func_arg.name) == arg:
                        print('argumento correcto')
                        correct_arg += 1

            if val and function.value != float(val):
                continue

            if correct_arg == len(args):
                return function

        return None

    def get_functions(self):
        return self.functions
    
    def get_problem(self):
        return str(self.problem)
    
    def remove_goal(self):
        if self.goal == None:
            self.problem.clear_goals()
            return True     

        self.goal.nodes = self.goal.nodes[:1]
        self.problem.clear_goals()
        return True
    
    def clear_knowledge(self): # eliminar también del problem
        if self.problem == None:
            return True
        
        self.predicates = []
        self.functions = []
        self.remove_goal()
        return True
    
    def remove_instance(self):
        pass
        

    def exists_function(self, function):
        if type(function) != TreeNode:
            return False

        if function in self.functions:
            print('existe la función')    
            return True
        
        print('no existe la función')
        return False


    def exists_predicate(self, predicate):
        if type(predicate) != TreeNode:
            return False
        
        if predicate in self.predicates:
            print('existe predicado')
            return True
        
        print('no existe predicado')
        return False
        
    def is_problem_goal_satisfied(self, goal):
        if type(goal) != Tree:
            print(f'type: {type(goal)}')
            return False
        
        return check(goal, self.predicates, self.functions)
        


    def _get_temporal_problem(self, expression):
        if expression is None:
            return None
        reader = PDDLReader()

        try:
            temp_problem = reader.parse_problem_string(self.domain_pddl, expression)

        except Exception as e:
            print(f"Error al cargar el archivo PDDL: {e}", file=sys.stderr)
            return None

        return temp_problem

    def _find_matching_fluent(self, name, parameters, bool_fluent=None):
        """
        bool_fluent:
        True  -> solo predicados
        False -> solo funciones
        None  -> ambos
        """
        for fluent in self.problem.fluents:
            if bool_fluent is True and not fluent.type.is_bool_type():
                continue
            if bool_fluent is False and not fluent.type.is_real_type():
                continue

            if fluent.name != name:
                continue
            if len(fluent.signature) != len(parameters):
                continue

            ok = True
            for p, f in zip(parameters, fluent.signature):
                if str(p.type) != str(f.type):
                    ok = False
                    break

            if ok:
                return fluent

        return None
    
    def _build_upf_expression(self, fluent, parameters):
        objs = []
        for p in parameters:
            obj = self._find_upf_object(p.name, p.type)
            if obj is None:
                return None
            objs.append(obj)
        return fluent(*objs)
    
    def _tree_to_upf(self, node):
        if node.node_type == TreeNode.PREDICATE:
            fluent = self._find_matching_fluent(node.name, node.parameters, True)
            expr = self._build_upf_expression(fluent, node.parameters)
            return Not(expr) if node.negate else expr

        elif node.node_type == TreeNode.AND:
            return And(*[self._tree_to_upf(self._node_map[cid]) for cid in node.children])

        elif node.node_type == TreeNode.OR:
            return Or(*[self._tree_to_upf(self._node_map[cid]) for cid in node.children])

        elif node.node_type == TreeNode.NOT:
            return Not(self._tree_to_upf(self._node_map[node.children[0]]))

    
    def _build_tree(self, tree_msg):
        node_map = {n.node_id: n for n in tree_msg.nodes}

        def build(node_id):
            node = node_map[node_id]
            node.children = [build(cid) for cid in node.children]
            return node

        return build(0)   # root siempre es node_id 0


    
def check_node(node_id, tree, predicates, functions):
    node = tree.nodes[node_id]

    print(f'----- CHECKING NODE {node} -----')

    if node.node_type == TreeNode.AND:
        print(f'>>>>>>> NODE AND')
        return all(check_node(c, tree, predicates, functions) for c in node.children)

    if node.node_type == TreeNode.OR:
        print(f'>>>>>>> NODE OR')
        return any(check_node(c, tree, predicates, functions) for c in node.children)

    if node.node_type == TreeNode.NOT:
        print(f'>>>>>>> NODE NOT')
        return not check_node(node.children[0], tree, predicates, functions)

    if node.node_type == TreeNode.EXPRESSION:
        print(f'>>>>>>> NODE EXPRESSION')
        # Evaluar ambos lados de la expresión
        left_val = evaluate_value(node.children[0], tree, predicates, functions)
        right_val = evaluate_value(node.children[1], tree, predicates, functions)
        return compare(left_val, right_val, node.expression_type)

    if node.node_type == TreeNode.PREDICATE:
        print(f'>>>>>>> NODE PREDICATE')
        return any(
            p.name == node.name and p.parameters == node.parameters
            for p in predicates
        )

    if node.node_type == TreeNode.FUNCTION:
        print(f'>>>>>>> NODE FUNCTION')
        current_value = None
        for f in functions:
            if f.name == node.name and f.parameters == node.parameters:
                current_value = f.value
                break
        
        if current_value is None:
            return False
        
        if node.value != 0.0:
            return compare(current_value, node.value, node.expression_type)
        
        elif node.children:

            target_value = evaluate_value(node.children[0], tree, predicates, functions)
            return compare(current_value, target_value, node.expression_type)
        
        return True

    if node.node_type == TreeNode.NUMBER:
        return node.value

    return False

def evaluate_value(node_id, tree, predicates, functions):
    """Evalúa un nodo que puede ser número, función o expresión aritmética"""
    node = tree.nodes[node_id]
    
    if node.node_type == TreeNode.NUMBER:
        return node.value
    
    if node.node_type == TreeNode.FUNCTION:
        for f in functions:
            if f.name == node.name and f.parameters == node.parameters:
                return f.value
        return 0.0
    
    if node.node_type == TreeNode.EXPRESSION and node.expression_type in [18, 19, 20, 21]:
        left_val = evaluate_value(node.children[0], tree, predicates, functions)
        right_val = evaluate_value(node.children[1], tree, predicates, functions)
        
        if node.expression_type == TreeNode.ARITH_ADD:
            return left_val + right_val
        elif node.expression_type == TreeNode.ARITH_SUB:
            return left_val - right_val
        elif node.expression_type == TreeNode.ARITH_MULT:
            return left_val * right_val
        elif node.expression_type == TreeNode.ARITH_DIV:
            return left_val / right_val if right_val != 0 else 0.0
    
    return 0.0


def check(tree, predicates, functions):
    return check_node(0, tree, predicates, functions)

def compare(a, b, cmp):
    if cmp == TreeNode.COMP_EQ:
        return a == b
    elif cmp == TreeNode.COMP_GT:
        return a > b
    elif cmp == TreeNode.COMP_GE:
        return a >= b
    elif cmp == TreeNode.COMP_LT:
        return a < b
    elif cmp == TreeNode.COMP_LE:
        return a <= b
    else:
        raise ValueError(f"Unknown comparison type: {cmp}")


def make_node(tree, node_type, name="", value=0.0, negate=False, parameters=None, expression_type=0, modifier_type=0):
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

def _build_tree_node(expr, tree:Tree, params) -> int:
    """Crea nodos Tree.Node recursivamente igual que get_tree de C++, 
    pero en una sola función y para UPF."""

    # ---------- Operadores lógicos ----------
    print(expr)
    print(type(expr))

    if tree.nodes is None or len(tree.nodes) == 0:
        print('-----------------------AND EXPR-----------------------')
        node = make_node(tree, TreeNode.AND)

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
            node = make_node(tree, TreeNode.AND)
    
        else:
            print('entra and, len == 1')
            node = tree.nodes[0]

        for arg in expr.args:
            cid =_build_tree_node(arg, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        
        return node.node_id

    if expr.is_or():
        node = make_node(tree, TreeNode.OR)

        for arg in expr.args:
            cid = _build_tree_node(arg, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        return node.node_id

    if expr.is_not():
        print('------------------entra not--------------------')
        node = make_node(tree, TreeNode.NOT)
        cid = _build_tree_node(expr.arg(0), tree, params)
        tree.nodes[node.node_id].children.append(cid)
        return node.node_id

    if expr.is_le() or expr.is_lt() or expr.is_equals():
        
        if expr.is_le(): 
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=TreeNode.COMP_LE)
        elif expr.is_lt(): 
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=TreeNode.COMP_LT)
        elif expr.is_equals(): 
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=TreeNode.COMP_EQ)
        
        for a in expr.args:
            cid = _build_tree_node(a, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        return node.node_id
        
    if expr.is_div() or expr.is_times() or expr.is_plus() or expr.is_minus():
        print("----------ENTRA DOT----------")

        if expr.is_div(): 
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=TreeNode.ARITH_DIV)
        elif expr.is_times(): 
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=TreeNode.ARITH_MULT)
        elif expr.is_plus(): 
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=TreeNode.ARITH_ADD)
        elif expr.is_minus(): 
            node = make_node(tree, TreeNode.EXPRESSION, expression_type=TreeNode.ARITH_SUB)
        
        node.children = []
        for a in expr.args:
            cid = _build_tree_node(a, tree, params)
            tree.nodes[node.node_id].children.append(cid)
        return node.node_id

    if expr.is_int_constant() or expr.is_real_constant():
        print("-------------entra number-------------")

        if expr.is_int_constant():
            node = make_node(tree, TreeNode.NUMBER, value=float(expr.int_constant_value()))
        else:
            node = make_node(tree, TreeNode.NUMBER, value=float(expr.real_constant_value()))

        return node.node_id

    if type(expr) == FNode:
        print(NumericExpression)
        fluent_name = str(expr).split("(")[0]
        print(fluent_name)

        if expr.type.is_bool_type():
            print('entra predicate')
            node = make_node(tree, TreeNode.PREDICATE, name=fluent_name)
        else:
            print('entra function')
            node = make_node(tree, TreeNode.FUNCTION, name=fluent_name)

        for a in expr.args:
            param = Param()
            print(a)
            print(params)
            index = next((i for i, item in enumerate(params) if item == str(a)), -1)
            param.name = str(a)
            param.type = str(a.type)
            param.sub_types = []
            node.parameters.append(param)

        return node.node_id

    raise NotImplementedError(f"No está implementado este tipo de nodo UPF: {expr}")
