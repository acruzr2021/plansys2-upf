from unified_planning.model import *
from unified_planning.io import PDDLReader
from dataclasses import dataclass, field
from typing import List
import sys
import re
import os
import inspect
import importlib.util
import unified_planning.model as up_model

@dataclass

class DomainUPFReader:
    def __init__(self):
        self.domain = None

    def load_pddl(self, pddl_path):
        reader = PDDLReader()
        try:
            self.domain = reader.parse_problem(pddl_path)
            return self.domain
        except Exception as e:
            print(f"Error al cargar el archivo PDDL: {e}", file=sys.stderr)


    def get_name(self):
        return self.domain.name if self.domain else ""
    
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
        
        return self.domain.user_types
    
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
        if not domain:
            return []
        return domain.derived_predicates

    def get_actions(self, domain):
        '''
        InstantaneousAction
        DurativeAction
        Task
        '''
        if not domain:
            return []
        return domain.actions


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