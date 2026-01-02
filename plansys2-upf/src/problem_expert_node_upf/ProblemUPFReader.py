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

class ProblemUPFExpert():
    def __init__(self, domain_path=None):
        self.problem = None
        self.domain = None
        with open(domain_path, "r") as f:
            self.domain_pddl = f.read()
        #self.domain_expert = domain_expert
        self.goal = None
        self.instances = []
        self.predicates = []
        self.functions = []

    def add_problem(self, problem_str):
        # si está en pddl, pasar a upf

        if problem_str is None:
            return None
        
        reader = PDDLReader()

        # print(self.domain_expert)
        # print("HAS domain_pddl:", hasattr(self.domain_expert, "domain_pddl"))
        # domain_pddl = self.domain_expert.domain_pddl
        try:
            self.problem = reader.parse_problem_string(self.domain_pddl, problem_str)

        except Exception as e:
            print(f"Error al cargar el archivo PDDL: {e}", file=sys.stderr)

        print(self.domain_pddl)

        # add instances and constants
        for instance in self.problem.all_objects:
            
            #imitando Param de affectedParam
            param = Param()
            param.name = instance.name
            param.type = instance.type.name
            param.sub_types = []

            # instance_obj = {
            #     'name': instance.name,
            #     'type': instance.type.name,
            #     'sub_types': []
            # }
            # instance_obj.name = str(instance)
            # instance_obj.type = str(instance.type)
            # instance_obj.sub_types = []
            # self.instances.append(instance_obj)
            print(f'Adding instance: {instance.name}')
            self.add_instance(param)
        

        # predicates and functions
        print(dir(self.problem))
        print(self.problem._initial_value)
        try: 
            for pred, val in self.problem._initial_value.items():
                try: 
                    print(f'Adding predicate/function: {pred}')
                    if pred.type.is_bool_type():
                        name = str(pred)
                        pred_obj = Node(node_name=name.replace("(", "_").replace(")", "_").replace(",", "_").replace(" ", "_"))
                        pred_obj.node_type = TreeNode.PREDICATE
                        #pred_obj.name = str(pred.name)
                        pred_obj.negate = val
                        pred_obj.parameters = []
                        
                        for arg in pred.args:
                            param = Param()
                            param.name = "?" + f'{len(pred_obj.parameters)}'
                            param.type = str(arg.type)
                            param.sub_types = []
                            pred_obj.parameters.append(param)

                        print(pred_obj)
                        
                        self.add_predicate(pred_obj)
                    elif pred.type.is_real_type():
                        print(dir(pred))
                        name = str(pred)
                        pred_obj = Node(node_name=name.replace("(", "_").replace(")", "_").replace(",", "_").replace(" ", "_"))
                        pred_obj.node_type = TreeNode.FUNCTION
                        #pred_obj.name = str(pred.name)
                        pred_obj.value = val
                        pred_obj.parameters = []
                        for arg in pred.args:
                            param = Param()
                            param.name = "?" + f'{len(pred_obj.parameters)}'
                            param.type = str(arg.type)
                            param.sub_types = []
                            pred_obj.parameters.append(param)
                        
                        self.add_function(pred)
                except Exception as e:
                    print(f'error 2: {e}')
                    continue

        except Exception as e:
            print(f'error 1: {e}')

        print(self.problem._goals)
        tree_goals = Tree()

        _build_tree_node(self.problem._goals, tree_goals, [])
        
        print(tree_goals)

        self.goal = tree_goals

        return True

        

        # instances = self.problem.all_objects
        # predicates = self.problem.fluents
        # functions = self.problem.functions



    def add_instance(self, instance):
        if type(instance) != Param:
            return False
        
        if instance not in self.instances:
            self.instances.append(instance)
            return True
        
        return False

    def add_predicate(self, predicate):
        if type(predicate) != Node:
            return False
        
        if predicate not in self.predicates:
            self.predicates.append(predicate)
            return True
        
        return False
        

    def add_function(self, function):
        if type(function) != Node:
            return False
        
        if function not in self.functions:
            self.functions.append(function)
            return True
        
        return False

    def add_problem_instance(self, instance):
        pass

    def add_problem_predicate(self, predicate):
        pass

    def add_problem_function(self, function):
        pass

    def add_problem_goal(self, goal):
        pass

    def get_goal(self):
        return str(self.goal)

    def get_instance(self, instance_name):
        for instance in self.instances:
            if instance.name == instance_name:
                return instance
        return None


    def get_instances(self):
        return self.instances

    def get_predicate(self, predicate_name):
        pass

    def get_predicates(self): # en plansys es bastante más grande --> revisar
        return self.predicates

    def get_function(self):
        pass

    def get_functions(self):
        return self.predicates

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
            param.name = "?" + f'{index}'
            param.type = str(a.type)
            param.sub_types = []
            node.parameters.append(param)

        return node.node_id

    raise NotImplementedError(f"No está implementado este tipo de nodo UPF: {expr}")
