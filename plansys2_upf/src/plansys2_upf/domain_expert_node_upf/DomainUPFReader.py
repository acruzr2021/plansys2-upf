from unified_planning.model import *
from unified_planning.io import PDDLReader, PDDLWriter
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
from pathlib import Path 
import os

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
        self.domains = []
        self.domain = None
        self.domain_pddl = None
        self.childrens = None
        if logger is None:
            self.debug = False
            self.logger = print
        else:
            self.debug = True
            self.logger = logger

    def load_pddl(self, pddl_string):
        '''Returns a UPF domain from a PDDL domain'''

        reader = PDDLReader()
        print(pddl_string)
        try:
            if Path(pddl_string).exists():
                print('path')
                with open(pddl_string, 'r') as file:
                    pddl_string = file.read()
            
            print("string")
            self.domain_pddl = pddl_string
            self.domain = reader.parse_problem_string(pddl_string)
            # self.domain = reader.parse_problem(pddl_path)
            self.children_map = self._build_children_map()
            print('sale de load_pddl')
            return True
        except Exception as e:
            print(f"Error loading PDDL file: {e}", file=sys.stderr)
            return False
        

    def extend_domain(self, pddl_string):
        '''Returns a UPF domain extended with the given PDDL domain'''

        reader = PDDLReader()
        if isinstance(pddl_string, str) and os.path.exists(pddl_string):
            with open(pddl_string, 'r') as file:
                pddl_string = file.read()
        else:
            pddl_string = pddl_string
        new_dom = None
        try: 
            
            new_dom = reader.parse_problem_string(pddl_string)
        except Exception as e:
            print(f"Error loading PDDL file: {e}", file=sys.stderr)
            return False

        if new_dom is None:
            return False

        if self.domain is None:
            self.domain = new_dom
            return True

        existing_type_names = {t.name for t in self.domain.user_types}

        for t in new_dom.user_types:
            if t.name not in existing_type_names:
                self.domain._add_user_type(t)

        existing_fluent_names = {f.name for f in self.domain.fluents}

        for f in new_dom.fluents:
            if f.name not in existing_fluent_names:
                self.domain.add_fluent(f)

        existing_action_names = {a.name for a in self.domain.actions}

        for a in new_dom.actions:
            if a.name not in existing_action_names:
                self.domain._actions.append(a)

        print("types", self.domain.all_objects)
        print("fluents", self.domain.fluents)
        print("actions", self.domain.actions)
        print("types", self.domain.user_types)


        self.children_map = self._build_children_map()
        writer = PDDLWriter(self.domain)
        domain_str = writer.get_domain()
        self.domain_pddl = domain_str

        return True

    def get_name(self):
        '''Returns the name of the domain'''

        return str(self.domain.name) if self.domain else ""

    def get_types(self):
        '''Returns the types of the domain'''

        types = [str(x) for x in self.domain.user_types]
        if "object" not in types:
            types.insert(0, "object")

        for t in types:
            if " - " in t:
                types.remove(t)
                types.append(t.split(" - ")[0])
            print(t)

        return types
    
    def get_constants(self, type):
        '''Returns the constants of the domain'''

        if not type:
            return []
                
        constant =[str(x) for x in self.domain.all_objects if str(x.type) == type]
        return constant


    def get_predicates(self):
        '''Returns the predicates of the domain'''

        nodes = []
        for fluent in self.domain.fluents:
            if fluent.type.is_bool_type():
                node = TreeNode()
                node.node_type = TreeNode.PREDICATE
                node.name = str(fluent._name)
                nodes.append(node)

        return nodes

    def get_functions(self):
        '''Returns the functions of the domain'''

        nodes =  []
        for fluent in self.domain.fluents:
            if not fluent.type.is_bool_type():
                node = TreeNode()
                node.node_type = TreeNode.FUNCTION
                node.name = str(fluent._name)
                nodes.append(node)

        return nodes
    
    def get_predicate(self, name):
        '''Returns the UPF predicate in a Tree plansys msg format'''

        for predicate in self.domain.fluents:
            if predicate._name == name.lower():
                node = TreeNode()
                node.node_type = TreeNode.PREDICATE
                node.name = str(predicate._name)
                node.parameters = []
                print(vars(predicate))
                for arg in predicate._signature:
                    param = Param()
                    param.type = str(arg).split(" ")[0]
                    param.name = "?" + f'{param.type}' +f'{len(node.parameters)}'
                    upf_type = self._get_type_by_name(param.type)
                    if upf_type is None:
                        param.sub_types = []
                    else:
                        param.sub_types = [str(child) for child in self.get_children(str(upf_type))]
                    node.parameters.append(param)

                return node
            
        return None
          
    def get_function(self, name):
        '''Returns the UPF function in a Tree plansys msg format'''

        for predicate in self.domain.fluents:
            if predicate._name == name.lower():
                node = TreeNode()
                node.node_type = TreeNode.FUNCTION
                node.name = str(predicate._name)
                node.parameters = []
                print(vars(predicate))
                for arg in predicate._signature:
                    param = Param()
                    param.type = str(arg).split(" ")[0]
                    param.name = "?" + f'{param.type}' + f'{len(node.parameters)}'
                    upf_type = self._get_type_by_name(param.type)
                    if upf_type is None:
                        param.sub_types = []
                    else:
                        param.sub_types = [str(child) for child in self.get_children(str(upf_type))]
                    node.parameters.append(param)

                return node
            
        return None
      
    def get_domain(self):
        '''Returns the UPF domain in PDDL format'''
        writer = PDDLWriter(self.domain)
        domain_str = writer.get_domain()
        return domain_str

    def get_actions(self):
        '''Returns a list of action names'''
        
        actions = []
        for a in self.domain.actions:
            if isinstance(a, (InstantaneousAction)):
                actions.append(str(a).split("(")[0].split(" ")[-1])
                print(actions)
        return actions

    def _get_type_by_name(self, type_name):
        '''Returns the UPF type by name'''
        for t in self.domain.user_types:
            if t.name == type_name:
                return t
        return None

    def get_action(self, action_name, params = None):
        '''Returns the UPF action by name and parameters (parameters are optional)'''

        for action_obj in self.domain.actions:
            action = str(action_obj)
            
            name = action.split("(")[0].split(" ")[-1]
            params_action = re.split(r"\s*,\s*", action.split("(")[1].split(")")[0])
            #self.logger.warn(f"name: i{name}i")
            
            if name != action_name:
                print('nope', name, action_name, type(name), type(action_name))
                #self.logger.warn(f"name: i{name}i != action_name: i{action_name}i")
                continue
            

            if params and (len(params) != len(params_action)):
                # self.logger.info(f"params: {len(params)}")
                # self.logger.warn(f"params_action: {len(params_action)}")

                continue
            
            if params:
                params_action_name = [re.split(r"\s+", param_a)[1] for param_a in params_action]    
                #self.logger.info(f"params_action_name: {params_action_name}")

                for param in params:
                    if param not in params_action_name:
                        #self.logger.warn(f"param: i{param}i not in params_action: {params_action_name}")
                        return

            same_params = True
            params_list = []

            if params is not None:
                for param in params:
                    #self.logger.info(f"param: {param}")

                    if param not in params_action:
                        #self.logger.warn(f"param: i{param}i not in params_action: {params_action}")
                        same_params = False
                        break
            
            if same_params or not params:
                print('entra en el if', action_obj)
                action_return = Action()
                action_return.name = name
                action_return.parameters = []
                
                for param in params_action:
                    print("param", param)
                    param_list = re.split(r"\s+", param)

                    print('added to the list: ', param_list[1])
                    params_list.append(param_list[1])

                    param_return = Param()
                    param_return.name = "?" + f'{len(params_list) - 1}'
                    param_return.type = str(param_list[0])

                    upf_type = self._get_type_by_name(param_list[0])
                    print(upf_type)
                    
                    if upf_type is None:
                        param_return.sub_types = []
                    else:
                        param_return.sub_types = [str(child) for child in self.get_children(str(upf_type))]

                    action_return.parameters.append(param_return)

                print(action_obj)

                print(type(action_obj))
                action_return.preconditions = preconditions_to_tree(action_obj._preconditions, params_list)

                # effects
                print('\n\n\n--------------------EFFECTS-------------------\n\n\n')
                action_return.effects = effects_to_tree(action_obj.effects, params_list)
                print('sale de los efectos')

                return action_return
            
        return None

    def get_durative_actions(self):
        '''Returns a list of durative action names'''

        actions = [
            (str(a).split("(")[0].split(" ")[-1]) for a in self.domain.actions
            if not isinstance(a, InstantaneousAction)
        ]
        print(actions)

        return actions

    def get_durative_action(self, action_name, params=None):
        '''Returns the UPF durative action by name and parameters (parameters are optional)'''

        for action_obj in self.domain.actions:

            action = str(action_obj)
            
            name = action.split("(")[0].split(" ")[-1]
            params_action = re.split(r"\s*,\s*", action.split("(")[1].split(")")[0])

            if name != action_name:
               # selg.logger.warn(f"name: i{name}i != action_name: i{action_name}i")
                continue
            
            if params and (len(params) != len(params_action)):
               # selg.logger.info(f"params: {len(params)}")
               # selg.logger.warn(f"params_action: {len(params_action)}")

                continue

            if params:
                params_action_name = [re.split(r"\s+", param_a)[1] for param_a in params_action]    
               # selg.logger.info(f"params_action_name: {params_action_name}")

                for param in params:
                    if param not in params_action_name:
                       # selg.logger.warn(f"param: i{param}i not in params_action: {params_action_name}")
                        return

           # selg.logger.info(f"params: {params}")
            
            same_params = True
            params_list = []

            if params is not None:
                for param in params:
                # selg.logger.info(f"param: {param}")

                    if param not in params_action:
                    # selg.logger.warn(f"param: i{param}i not in params_action: {params_action}")
                        same_params = False
                        break

           # selg.logger.info(f"same_params: {same_params}")

            if same_params or params is None:
               # selg.logger.info(f"action: {action}")
               # selg.logger.info(f"name: {name}")
                action_return = DurativeAction()
               # selg.logger.info(f"action_return:")
                action_return.name = name
                action_return.parameters = []
               # selg.logger.info(f"action_return:")
                
                for param in params_action:
                    param_list = re.split(r"\s+", param)
                   # selg.logger.info(f"param_list: {param_list}")

                    print('added to the list: ', param_list[1])
                    params_list.append(param_list[1])

                    param_return = Param()
                    param_return.name = "?" + f'{len(params_list) - 1}'
                    param_return.type = str(param_list[0])

                    upf_type = self._get_type_by_name(param_list[0])
                    print(upf_type, type(upf_type), dir(upf_type))
                    
                    if upf_type is None:
                        param_return.sub_types = []
                    else:
                        param_return.sub_types = [str(child) for child in self.get_children(str(upf_type))]

                       # selg.logger.info(f"param_return children: {param_return.sub_types}")

                    action_return.parameters.append(param_return)

                print(action_obj)

               # selg.logger.info(f"action_obj:")
               # selg.logger.info(f'{type(action_obj)}')
               # selg.logger.info(f'{vars(action_obj)}')

               # selg.logger.info(f'{action_obj.conditions}')
               # selg.logger.info(f'{type(action_obj.conditions)}')

                print("TimePointInterval =", TimePointInterval)
                print("type(TimePointInterval) =", type(TimePointInterval))


                START = type(StartTiming())
                END = type(EndTiming())

                action_return.at_start_requirements = Tree()
                action_return.over_all_requirements = Tree()
                action_return.at_end_requirements = Tree()

                action_return.at_start_effects = Tree()
                action_return.at_end_effects = Tree()

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


                if len(action_return.at_start_requirements.nodes) == 0:
                   # selg.logger.info('************ENTRA EN NO START REQUIREMENT***********')
                    action_return.at_start_requirements = Tree()
                    make_node(action_return.at_start_requirements, TreeNode.AND)
                if len(action_return.at_end_requirements.nodes) == 0:
                   # selg.logger.info('************ENTRA EN NO END REQUIREMENT***********')
                    action_return.at_end_requirements = Tree()
                    make_node(action_return.at_end_requirements, TreeNode.AND)
                if len(action_return.over_all_requirements.nodes) == 0:
                   # selg.logger.info('************ENTRA EN NO OVER ALL REQUIREMENT***********')
                    action_return.over_all_requirements = Tree()
                    make_node(action_return.over_all_requirements, TreeNode.AND)

                # effects
               # selg.logger.info(f"action_return: {action_obj._effects}")

                for time, eff in action_obj._effects.items():
                   # selg.logger.info(f"time: {time}, eff: {eff}")
                   # selg.logger.info(f"{type(time)}")


                    if str(time) == 'start':
                       # selg.logger.info(f">>> AT START")
                        action_return.at_start_effects = effects_to_tree(eff, params_list)
                    elif str(time) == 'end':
                       # selg.logger.info(f">>> AT END")
                        action_return.at_end_effects = effects_to_tree(eff, params_list)
                    
                if len(action_return.at_start_effects.nodes) == 0:
                   # selg.logger.info('************ENTRA EN NO START EFFECTS***********')
                    action_return.at_start_effects = Tree()
                    make_node(action_return.at_start_effects, TreeNode.AND)
                if len(action_return.at_end_effects.nodes) == 0:
                   # selg.logger.info('************ENTRA EN NO END EFFECTS***********')
                    action_return.at_end_effects = Tree()
                    make_node(action_return.at_end_effects, TreeNode.AND)
                print(action_return)    
                return action_return
        
        return None

    def get_children(self, type_name):
        '''Returns the children of a type'''

        print(type_name)
        if self.children_map is None:
            return []
        return self.children_map.get(type_name, [])
 
    def _build_children_map(self):
        '''Returns a dictionary with the children of each type'''
 
        children = defaultdict(list)

        for t in self.domain.user_types:
            if t.father is not None:
                children[t.father.name].append(t.name)

        for parent, childs in children.items():
            print(parent, " => hijos:", childs)
        
        return children

def _build_tree_node(expr, tree:Tree, params) -> int:
    """Returns Tree nodes recursively using UPF sintax"""

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

def preconditions_to_tree(expr, params) -> Tree:
    '''Returns a Tree plansys msg from a precondition'''

    tree = Tree()
    _build_tree_node(expr, tree, params)
    return tree

def make_node(tree, node_type, name="", value=0.0, negate=False, parameters=None, expression_type=0, modifier_type=0):
    '''Builds a Node plansys msg'''

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
    '''Builds a Node plansys msg from a predicate'''

    parameters = []
    for a in fluent.args:
        index = next((i for i, item in enumerate(params) if item == str(a)), -1)
        param = Param()
        param.name = f"?{index}"
        param.type = str(a.type)
        param.sub_types = []
        parameters.append(param)
    return make_node(tree, TreeNode.PREDICATE, name=str(fluent), parameters=parameters)

def effects_to_tree(effects, params) -> Tree:
    '''Returns a Tree plansys msg from a list of effects'''
    global tree
    tree = Tree()

    and_node = make_node(tree, TreeNode.AND)
    print(effects)

    for eff in effects:
        print(eff)
        print(type(eff))
        print(vars(eff))

        if eff.value.is_false():
            not_node = make_node(tree, TreeNode.NOT)
            and_node.children.append(not_node.node_id)

            child_node = make_predicate_node(tree, eff.fluent, params)
            not_node.children.append(child_node.node_id)
        
        elif eff.value.is_true():
            child_node = make_predicate_node(tree, eff.fluent, params)
            and_node.children.append(child_node.node_id)

        elif eff._kind == EffectKind.INCREASE or eff._kind == EffectKind.DECREASE or eff._kind == EffectKind.ASSIGN:
            
            print("entra modificadores")
            if eff._kind == EffectKind.INCREASE:
                node = make_node(tree, TreeNode.FUNCTION_MODIFIER, modifier_type=TreeNode.INCREASE)
            elif eff._kind == EffectKind.DECREASE:
                node = make_node(tree, TreeNode.FUNCTION_MODIFIER, modifier_type=TreeNode.DECREASE)
            elif eff._kind == EffectKind.ASSIGN:
                node = make_node(tree, TreeNode.FUNCTION_MODIFIER, modifier_type=TreeNode.ASSIGN)

            and_node.children.append(node.node_id)

            id = _build_tree_node(eff._fluent, tree, params)
            
            node.children.append(id)

            id = _build_tree_node(eff._value, tree, params)

            node.children.append(id)

            print(type(eff._value))            

    return tree

