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

class ProblemUPFExpert:
    def __init__(self, domain_reader):
        self.domain_reader = domain_reader
        self.domain_pddl = domain_reader.domain_pddl
        self.domain = domain_reader.domain
        self.problem = None
        self.goal = None
        self.instances = []
        self.predicates = []
        self.functions = []

        self.add_problem()
        print(self.problem)

    def add_problem(self, problem_str=None):

        reader = PDDLReader()

        if problem_str is None:
            new_problem = reader.parse_problem_string(self.domain_pddl)
        else:
            try:
                new_problem = reader.parse_problem_string(
                    self.domain_pddl,
                    problem_str
                )
            except Exception as e:
                print(f"Error al cargar el archivo PDDL: {e}", file=sys.stderr)
                return False

        if self.problem is None:
            self.problem = new_problem
            self.instances = []
            self.predicates = []
            self.functions = []

        else:
            # Fusionar objetos
            for obj in new_problem.all_objects:
                if obj not in self.problem.all_objects:
                    self.problem.add_object(obj)

            # Fusionar estado inicial
            for fluent, value in new_problem._initial_value.items():
                self.problem.set_initial_value(fluent, value)

            # Fusionar goal (sobrescribe)
            self.problem._goals = new_problem._goals

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
                # ---------- BOOLEAN → Predicate ----------
                if pred.type.is_bool_type():

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

                # ---------- REAL → Function ----------
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

            except Exception as e:
                print(f"Error procesando fluent: {e}")
                continue

        tree_goals = Tree()
        _build_tree_node(self.problem._goals, tree_goals, [])
        self.goal = tree_goals

        return True


    def add_instance(self, instance):
        if type(instance) != Param:
            return False
        
        available_types = [t.name for t in self.domain.user_types]
        
        if instance.type not in available_types:
            print(f"Tipo '{instance.type}' no encontrado. Tipos disponibles: {available_types}")
            return False
        
        if instance in self.instances:
            return True
        
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

    def add_predicate(self, predicate):
        if not isinstance(predicate, TreeNode):
            return False
        if predicate.node_type != TreeNode.PREDICATE:
            return False
        
        if self.exists_predicate(predicate):
            return True

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
        
        fluent = self._find_matching_fluent(
            function.name, function.parameters, bool_fluent=False
        )
        
        if fluent is None:
            return False
        
        if function.node_type != TreeNode.FUNCTION:
            return False

        if not self.exists_function(function):
            print('la función no existe')

            expr = self._build_upf_expression(fluent, function.parameters)
            if expr is None:
                return False

            self.problem.set_initial_value(expr, function.value)
            print(self.problem._initial_value)
            print('\n')
            self.functions.append(function)

            return True
        
        else:
            print('la función existe')
            return self.update_function(function) 

    def add_goal(self, goal):
        if not isinstance(goal, Tree):
            return False

        if self.goal is not None:
            self.remove_goal()

        self._node_map = {n.node_id: n for n in goal.nodes}
        root = self._node_map[0]
        print(root)
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
    
    def clear_knowledge(self):

        if self.problem is None:
            return True

        from unified_planning.io import PDDLReader
        reader = PDDLReader()

        # reconstruir problema vacío SOLO con dominio
        self.problem = reader.parse_problem_string(self.domain_pddl)

        self.instances = []
        self.predicates = []
        self.functions = []
        self.goal = None

        return True

    
    def remove_instance(self, instance): # eliminar también predicados/funciones y goals que sean invalidos
        print('\n\n\n ----------------REMOVE INSTANCE------------------\n\n\n')
        if not isinstance(instance, Param):
            return False
        
        if instance not in self.instances:
            return False
        
        pred_to_eliminate = []
        function_to_eliminate = []
        functions_upf_to_remove = []
        goals_to_eliminate = []
        goals_upf_to_remove = []

        print('\t------------PREDICADOS A ELIMINAR---------')
        for pred in self.predicates:
            print(pred, pred.name, pred.parameters)
            # if instance not in pred.parameters:
            #     print('no coincide')
            #     continue
            for param in pred.parameters:
                print(param.name, instance.name, param.type, instance.type)
                if param.name == instance.name and param.type == instance.type:
                    pred_to_eliminate.append(pred) # luego es eliminarlo de la lista local y poner en el problem a false
                    self.remove_predicate(pred)
                    break
        
        print(f'\tpredicados a eliminar {pred_to_eliminate}\n')
        print(f'\tpredicados finales: {self.predicates}\n')
        print(f'\tproblem: {self.problem._initial_value}\n')

        print('\t------------FUNCIONES A ELIMINAR---------')

        for func in self.functions:
            # if instance.name not in func.parameters:
            #     continue
            for param in func.parameters:
                if param.name == instance.name and param.type == instance.type:
                    function_to_eliminate.append(func)
                    self.remove_function(func)
                    break
        
        print(f'funciones a eliminar {function_to_eliminate}')
        print(f'funciones finales: {self.functions}')
        print(f'problem: {self.problem._initial_value}')

        print('\t------------GOALS A ELIMINAR---------')
        # valid_goals = []

        # for g in self.problem.goals:
        #     invalid = False
        #     print(g)
        #     for subgoal in g.args:
        #         print(subgoal)
        #         for a in subgoal.args:
        #             print(a)
        #             if str(a) == instance.name and a.type == instance.type:
        #                 invalid = True
        #                 break
        #         if not invalid:
        #             valid_goals.append(subgoal)

        # print(f'goals validos {valid_goals}')

        

        new_goal = self._clear_goal(self.goal, instance)

        if new_goal is not None:
            self.add_goal(new_goal)

        print(new_goal)
        print(self.problem.goals)


        instance_upf = ''
        for inst in self.problem.all_objects:
            if inst.name == instance.name and inst.type == instance.type:
                instance_upf = inst
                break

        print(f'instance a eliminar upf {instance_upf}')
                
        
        self._rebuild_problem(objects=[instance_upf])
        
        print("fluents", self.problem._initial_value)
        print("objetos", self.problem.all_objects)
        print("goals", self.problem.goals)
        print('\n')

        self.instances.remove(instance)
        # self.problem.remove_object(instance) #eliminar instancia

        return True

    def remove_predicate(self, predicate):
        if not isinstance(predicate, TreeNode):
            return False

        if not self.exists_predicate(predicate):
            return False

        fluent = self._find_matching_fluent(predicate.name, predicate.parameters, True)
        expr = self._build_upf_expression(fluent, predicate.parameters)
        if expr is None:
            return False

        self.problem.set_initial_value(expr, False)

        self.predicates = [
            p for p in self.predicates
            if not (p.name == predicate.name and p.parameters == predicate.parameters)
        ]

        print(self.problem._initial_value)
        print('\n')
        return True

    
    def remove_function(self, function):
        if type(function) != TreeNode:
            return False
        
        if not self.exists_function(function):
            return False
        
        fluent = self._tree_to_upf(function)

        print(fluent)

        # functions_upf = [f for f in self.problem._initial_value if (f.name != function.name and f.parameters != function.parameters and f.is_real_type())]
        # print(functions_upf)
        # predicates_upf = [p for p in self.problem._initial_value if p.is_bool_type()]
        
        # instances_upf = [i for i in self.all_objects]

        self._rebuild_problem(functions=[fluent])
        self.functions = [
            f for f in self.functions
            if not (f.name == function.name and f.parameters == function.parameters)]

        print(self.problem._initial_value)
        print('\n')
        
        return True

    def exists_function(self, function):
        if type(function) != TreeNode:
            return False

        for f in self.functions:
            print(function)
            print(function.name)
         
            if check_equality_node(f, function):
                print('existe la función')
                return True
            
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
        
        print("PREDICATES ACTUALES:")
        for p in self.predicates:
            print(p.name, [(param.name, param.type) for param in p.parameters])
        
        return check(goal, self.predicates, self.functions)
    
    def update_function(self, function):
        if not isinstance(function, TreeNode):
            return False
        
        if function.node_type != TreeNode.FUNCTION:
            return False
        
        if not self.exists_function(function):
            return False
        
        # actualizar valor en lista local
        self.functions = [f for f in self.functions if (f.name != function.name and f.parameters != function.parameters)]
        self.functions.append(function)

        #actualizar valor en upf
        fluent = self._find_matching_fluent(function.name, function.parameters, False)
        expr = self._build_upf_expression(fluent, function.parameters)

        if expr is None:
            return False
        
        self.problem.set_initial_value(expr, function.value)
        print(self.problem._initial_value)

        return True

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
        True  -> search on predicates
        False -> search functions
        None  -> search both
        """
        print(f"params: {name}, {parameters}, {bool_fluent}")
        for fluent in self.problem.fluents:
            print(fluent, fluent.type, str(fluent))
            print(bool_fluent)
            if bool_fluent is True and not fluent.type.is_bool_type():
                print('no coincide tipo 1')
                continue
            if bool_fluent is False and fluent.type.is_bool_type():
                print('no coincide tipo 2')
                continue

            if fluent.name != name:
                print('no coincide nombre')
                continue
            print('coincide nombre')
            if len(fluent.signature) != len(parameters):
                print('no coincide longitud')
                continue

            ok = True
            for p, f in zip(parameters, fluent.signature):
                print(p, f)
                if str(p.type) != str(f.type):
                    ok = False
                    break

            if ok:
                print(f'coincide todo, {fluent}')
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
        
        if node.node_type == TreeNode.FUNCTION:
            fluent = self._find_matching_fluent(node.name, node.parameters, False)
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
    
    from unified_planning.model import Problem

    def _rebuild_problem(self, *,
                        objects=None,
                        predicates=None,
                        functions=None,
                        goal=None):

        old = self.problem
        new = Problem(old.name)

        # 1. Dominio
        for t in old.user_types:
            new._add_user_type(t)

        for f in old.fluents:
            new.add_fluent(f)

        # 2. Objetos
        for obj in (old.all_objects if objects is None else [o for o in old.all_objects if o not in objects]):
            new.add_object(obj)


        #unir lista predicado y funciones
        fluents = []
        if predicates:
            fluents += predicates
        if functions:
            fluents += functions

        # 3. Estado inicial
        # 3. Estado inicial
        for expr, value in old._initial_value.items():

            skip = False

            for f in fluents:
                if (
                    expr.fluent().name == f.fluent().name and
                    len(expr.args) == len(f.args) and
                    all(str(a) == str(b) for a, b in zip(expr.args, f.args))
                ):
                    skip = True
                    break

            if skip:
                continue

            new.set_initial_value(expr, value)


        # 4. Goal
        if goal is not None:
            new.add_goal(goal)
        else:
            for g in old.goals:
                new.add_goal(g)

        self.problem = new

    def _clear_goal(self, goal: Tree, instance: Param):
        valid_predicates = []

        for node in goal.nodes:
            if node.node_type != TreeNode.PREDICATE:
                continue

            invalid = False
            for p in node.parameters:
                if p.name == instance.name and p.type == instance.type:
                    invalid = True
                    break

            if not invalid:
                valid_predicates.append(node)

        # si no queda ningún goal → no hay goal
        if not valid_predicates:
            return None

        # reconstruir Tree NUEVO
        nodes = []

        # root AND
        root = TreeNode()
        root.node_type = TreeNode.AND
        root.node_id = 0
        root.children = list(range(1, len(valid_predicates) + 1))
        nodes.append(root)

        # hojas
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


    # def _get_goal_pddl(self):

    #     if self.problem is None:
    #         return ""

    #     if not self.problem.goals:
    #         return ""

    #     writer = PDDLWriter(self.problem)
    #     problem_str = writer.get_problem()

    #     # Extraer solo la sección (:goal ...)
    #     start = problem_str.find("(:goal")
    #     if start == -1:
    #         return ""

    #     goal_str = problem_str[start:]
    #     goal_str = goal_str.split(")", 1)[0] + ")"

    #     return goal_str.strip()
    def _get_goal_pddl(self):

        if self.problem is None:
            return ""

        if not self.problem.goals:
            return ""

        # Normalmente hay un solo goal (con and)
        goal_expr = self.problem.goals[0]

        return str(goal_expr)


    def _get_predicates_pddl(self):
        predicates = []

        for fluent, value in self.problem._initial_value.items():
        
            if fluent.type.is_bool_type() and bool(value):
                args = " ".join(str(a) for a in fluent.args)
                predicates.append(f"({fluent.fluent().name} {args})")

        return predicates
    
    def _get_functions_pddl(self):
        functions = []

        if self.problem is None:
            return functions

        for fluent, value in self.problem._initial_value.items():

            if fluent.type.is_real_type():

                args = " ".join(str(a) for a in fluent.args)

                # Extraer valor real correctamente
                if hasattr(value, "constant_value"):
                    val = value.constant_value()
                else:
                    val = value

                functions.append(
                    f"(= ({fluent.fluent().name} {args}) {float(val)})"
                )

        return functions





def check_equality_node(tree_a: TreeNode, tree_b: TreeNode):
    if tree_a.node_type != tree_b.node_type:
        return False
    
    if tree_a.name != tree_b.name:
        return False
    
    if tree_a.parameters != tree_b.parameters:
        return False
        
    if len(tree_a.children) != len(tree_b.children):
        return False
        
    for a, b in zip(tree_a.children, tree_b.children):
        if a != b:
            return False
        
    return True

    
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
