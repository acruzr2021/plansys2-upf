from typing import Optional, List
import DomainUPFReader
import unified_planning.model as up_model

from plansys2_msgs.msg import (
    GetDomainName,
    GetDomainTypes,
    GetDomainActions,
    GetDomainActionDetails,
    GetDomainDurativeActions,
    GetDomainDurativeActionDetails,
    GetStates,
    GetNodeDetails,
    GetStates,
    GetNodeDetails,
    GetStates,
    GetDomainDerivedPredicateDetails,
    GetDomain,
    ValidateDomain
)

class DomainExpertInterface():
    pass

class DomainExpert(DomainExpertInterface):
    
    def __init__(self, domain: str):
        self._domains: DomainUPFReader = DomainUPFReader()
        self._domain: Optional[Problem()] = None
        self.extend_domain(domain)
        pass

    def extendDomain(self, domain: str): # revisar
        self._domains.add_domain(domain)
        self._domain = PDDLDomain()
        
        try:
            joint_domain_string = self._domains.get_joint_domain()
            self._domain.parse(joint_domain_string)
        except Exception as e:
            print(f'Error parsing PDDL: {e}')
        
    def getName(self):
        return self._domain.name
    
    def getTypes(self):
        if self._domain and self._domain.typed:
            return [type_obj.name for type_obj in self._domain.types]
        else:
            return []
        
    def getConstants(self, type: str):
        pass

    def getPredicates(self):
        pass

    def getPredicate(self, predicate: str):
        pass

    def getFunctions(self):
        pass

    def getFunction(self, function: str):
        pass

    def getDerivedPredicates(self):
        pass

    def getDerivedPredicate(self, predicate: str):
        pass

    def getActions(self):
        pass

    def getAction(self, action: str, parameters: List[str]):
        pass

    def getDurativeActions(self):
        pass

    def getDurativeAction(self, action: str, parameters: List[str]):
        pass

    def getDomain(self):
        pass

    def existDomain(self, domain_name: str):
        for domain in self._domains:
            if domain.name == domain_name:
                return True
            
        return False