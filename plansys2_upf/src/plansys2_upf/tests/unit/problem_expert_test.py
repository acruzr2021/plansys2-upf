import pathlib
import pytest

from ament_index_python.packages import get_package_share_directory

from plansys2_msgs.msg import Node as NodeMsg, Tree, Param

from plansys2_upf.domain_expert_node_upf.DomainUPFReader import DomainUPFReader
from plansys2_upf.problem_expert_node_upf.ProblemExpertNode import ProblemUPFExpert


# from plansys2_pddl_parser import pddl

def test_problem_expert_addget_instances():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    # ---- ADD INSTANCES ----
    assert problem_expert.add_instance(Param(name="Paco", type="person"))
    assert problem_expert.add_instance(Param(name="Paco", type="person"))   # duplicado permitido
    assert not problem_expert.add_instance(Param(name="Paco", type="room"))
    assert not problem_expert.add_instance(Param(name="Paco", type="SCIENTIFIC"))

    assert problem_expert.add_instance(Param(name="r2d2", type="robot"))
    assert problem_expert.add_instance(Param(name="ur5e", type="Robot"))  # case-insensitive

    instances = problem_expert.get_instances()
    assert len(instances) == 3

    assert instances[0].name == "Paco"
    assert instances[0].type == "person"
    assert instances[1].name == "r2d2"
    assert instances[1].type == "robot"
    assert instances[2].name == "ur5e"
    assert instances[2].type == "robot"

    # ---- REMOVE INSTANCE ----
    assert problem_expert.remove_instance(Param(name="Paco", type="person"))

    instances = problem_expert.get_instances()
    assert len(instances) == 2

    assert instances[0].name == "r2d2"
    assert instances[0].type == "robot"
    assert instances[1].name == "ur5e"
    assert instances[1].type == "robot"

    # ---- GET INSTANCE ----
    assert problem_expert.get_instance("Paco") is None

    r2d2 = problem_expert.get_instance("r2d2")
    assert r2d2 is not None
    assert r2d2.name == "r2d2"
    assert r2d2.type == "robot"

    ur5e = problem_expert.get_instance("ur5e")
    assert ur5e is not None
    assert ur5e.name == "ur5e"
    assert ur5e.type == "robot"

def test_problem_expert_add_functions():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    # ---- ADD INSTANCES ----
    assert problem_expert.add_instance(Param(name="bedroom", type="room"))
    assert problem_expert.add_instance(Param(name="kitchen", type="room_with_teleporter"))

    # ---- FUNCTION 1 ----
    f1 = NodeMsg()
    f1.node_type = NodeMsg.FUNCTION
    f1.name = "room_distance"
    f1.parameters.append(Param(name="bedroom", type="room"))
    f1.parameters.append(Param(name="kitchen", type="room_with_teleporter"))
    f1.value = 1.23

    assert f1.name == "room_distance"
    assert len(f1.parameters) == 2
    assert f1.parameters[0].name == "bedroom"
    assert f1.parameters[0].type == "room"
    assert f1.parameters[1].name == "kitchen"
    assert f1.parameters[1].type == "room_with_teleporter"
    assert f1.value == 1.23

    assert problem_expert.add_function(f1)

    expected_1 = (
        "( define ( problem problem_1 )\n"
        "( :domain simple )\n"
        "( :objects\n"
        "\tbedroom - room\n"
        "\tkitchen - room_with_teleporter\n"
        ")\n"
        "( :init\n"
        "\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n"
        ")\n"
        "( :goal\n"
        "\t( and\n"
        "\t))\n"
        ")\n"
    )

    assert problem_expert.get_problem() == expected_1

    # ---- FUNCTION 2 ----
    f2 = NodeMsg()
    f2.node_type = NodeMsg.FUNCTION
    f2.name = "room_distance"
    f2.parameters.append(Param(name="kitchen", type="room_with_teleporter"))
    f2.parameters.append(Param(name="bedroom", type="room"))
    f2.value = 2.34

    assert problem_expert.add_function(f2)

    expected_2 = (
        "( define ( problem problem_1 )\n"
        "( :domain simple )\n"
        "( :objects\n"
        "\tbedroom - room\n"
        "\tkitchen - room_with_teleporter\n"
        ")\n"
        "( :init\n"
        "\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n"
        "\t( = ( room_distance kitchen bedroom ) 2.3400000000 )\n"
        ")\n"
        "( :goal\n"
        "\t( and\n"
        "\t))\n"
        ")\n"
    )

    assert problem_expert.get_problem() == expected_2

    # ---- UPDATE FUNCTION 2 ----
    f2.value = 3.45
    assert problem_expert.add_function(f2)

    expected_3 = (
        "( define ( problem problem_1 )\n"
        "( :domain simple )\n"
        "( :objects\n"
        "\tbedroom - room\n"
        "\tkitchen - room_with_teleporter\n"
        ")\n"
        "( :init\n"
        "\t( = ( room_distance bedroom kitchen ) 1.2300000000 )\n"
        "\t( = ( room_distance kitchen bedroom ) 3.4500000000 )\n"
        ")\n"
        "( :goal\n"
        "\t( and\n"
        "\t))\n"
        ")\n"
    )

    assert problem_expert.get_problem() == expected_3

    # ---- INVALID FUNCTION ----
    f3 = NodeMsg()
    f3.node_type = NodeMsg.FUNCTION
    f3.name = "room_temperature"
    f3.parameters.append(Param(name="bedroom", type="room"))
    f3.parameters.append(Param(name="kitchen", type="room_with_teleporter"))
    f3.value = 2.34

    assert not problem_expert.add_function(f3)
    assert not problem_expert.remove_function(f3)

    assert problem_expert.remove_instance(Param(name="kitchen", type="room_with_teleporter"))

def test_problem_expert_addget_predicates():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    # ---- DEFINE PREDICATES ----
    p1 = NodeMsg()
    p1.node_type = NodeMsg.PREDICATE
    p1.name = "robot_at"
    p1.parameters.append(Param(name="r2d2", type="robot"))
    p1.parameters.append(Param(name="bedroom", type="room"))

    p2 = NodeMsg()
    p2.node_type = NodeMsg.PREDICATE
    p2.name = "robot_at"
    p2.parameters.append(Param(name="r2d2", type="robot"))
    p2.parameters.append(Param(name="kitchen", type="room"))

    p3 = NodeMsg()
    p3.node_type = NodeMsg.PREDICATE
    p3.name = "person_at"
    p3.parameters.append(Param(name="paco", type="person"))
    p3.parameters.append(Param(name="bedroom", type="room"))

    p4 = NodeMsg()
    p4.node_type = NodeMsg.PREDICATE
    p4.name = "person_at"
    p4.parameters.append(Param(name="paco", type="person"))
    p4.parameters.append(Param(name="kitchen", type="room"))

    p5 = NodeMsg()
    p5.node_type = NodeMsg.PREDICATE
    p5.name = "person_at"
    p5.parameters.append(Param(name="paco", type="person"))
    p5.parameters.append(Param(name="kitchen", type="room"))
    p5.parameters.append(Param(name="r2d2", type="robot"))
    p5.parameters.append(Param(name="bedroom", type="room"))

    p6 = NodeMsg()
    p6.node_type = NodeMsg.PREDICATE
    p6.name = "person_at"
    p6.parameters.append(Param(name="kitchen", type="room"))
    p6.parameters.append(Param(name="paco", type="person"))

    # ---- ADD INSTANCES ----
    assert problem_expert.add_instance(Param(name="paco", type="person"))
    assert problem_expert.add_instance(Param(name="r2d2", type="robot"))
    assert problem_expert.add_instance(Param(name="bedroom", type="room"))
    assert problem_expert.add_instance(Param(name="kitchen", type="room"))

    assert problem_expert.get_predicates() == []

    assert problem_expert.add_predicate(p1)
    assert problem_expert.add_predicate(p1)
    assert problem_expert.add_predicate(p2)
    assert problem_expert.add_predicate(p3)
    assert problem_expert.add_predicate(p4)
    assert not problem_expert.add_predicate(p5)
    assert not problem_expert.add_predicate(p6)

    predicates = problem_expert.get_predicates()
    assert len(predicates) == 4

    pred_2 = problem_expert.get_predicate("(robot_at r2d2 kitchen)")
    assert pred_2 is not None
    assert pred_2.name == "robot_at"
    assert len(pred_2.parameters) == 2
    assert pred_2.parameters[0].name == "r2d2"
    assert pred_2.parameters[0].type == "robot"
    assert pred_2.parameters[1].name == "kitchen"
    assert pred_2.parameters[1].type == "room"

    assert not problem_expert.remove_predicate(p5)
    assert problem_expert.remove_predicate(p4)
    assert problem_expert.remove_predicate(p4)

    predicates = problem_expert.get_predicates()
    assert len(predicates) == 3

    assert problem_expert.add_instance(Param(name="bathroom", type="room_with_teleporter"))

    p7 = NodeMsg()
    p7.node_type = NodeMsg.PREDICATE
    p7.name = "is_teleporter_enabled"
    p7.parameters.append(Param(name="bathroom", type="room_with_teleporter"))

    assert problem_expert.add_predicate(p7)

    p8 = NodeMsg()
    p8.node_type = NodeMsg.PREDICATE
    p8.name = "is_teleporter_destination"
    p8.parameters.append(Param(name="bathroom", type="room_with_teleporter"))

    assert problem_expert.add_predicate(p8)

    assert problem_expert.remove_instance(Param(name="bathroom", type="room_with_teleporter"))

def test_problem_expert_addget_functions():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_charging.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    # ---------- function_1 ----------
    f1 = NodeMsg()
    f1.node_type = NodeMsg.FUNCTION
    f1.name = "speed"
    f1.parameters.append(Param(name="r2d2", type="robot"))
    f1.value = 3

    assert f1.name == "speed"
    assert len(f1.parameters) == 1
    assert f1.parameters[0].name == "r2d2"
    assert f1.parameters[0].type == "robot"
    assert f1.value == 3

    # ---------- function_2 ----------
    f2 = NodeMsg()
    f2.node_type = NodeMsg.FUNCTION
    f2.name = "distance"
    f2.parameters.append(Param(name="wp1", type="waypoint"))
    f2.parameters.append(Param(name="wp2", type="waypoint"))
    f2.value = 15

    assert f2.name == "distance"
    assert len(f2.parameters) == 2
    assert f2.parameters[0].name == "wp1"
    assert f2.parameters[0].type == "waypoint"
    assert f2.parameters[1].name == "wp2"
    assert f2.parameters[1].type == "waypoint"
    assert f2.value == 15

    # ---------- function_3 (inválida) ----------
    f3 = NodeMsg()
    f3.node_type = NodeMsg.FUNCTION
    f3.name = "speed"
    f3.parameters.append(Param(name="r2d2", type="robot"))
    f3.parameters.append(Param(name="wp1", type="waypoint"))

    # ---------- function_4 (inválida) ----------
    f4 = NodeMsg()
    f4.node_type = NodeMsg.FUNCTION
    f4.name = "distance"
    f4.parameters.append(Param(name="r2d2", type="robot"))
    f4.parameters.append(Param(name="wp1", type="waypoint"))

    # ---------- add instances ----------
    assert problem_expert.add_instance(Param(name="r2d2", type="robot"))
    assert problem_expert.add_instance(Param(name="wp1", type="waypoint"))
    assert problem_expert.add_instance(Param(name="wp2", type="waypoint"))

    functions = problem_expert.get_functions()
    assert functions == []

    assert problem_expert.add_function(f1)
    functions = problem_expert.get_functions()
    assert functions != []

    assert problem_expert.add_function(f1)
    assert problem_expert.add_function(f2)
    assert not problem_expert.add_function(f3)
    assert not problem_expert.add_function(f4)

    functions = problem_expert.get_functions()
    assert len(functions) == 2

    func_2 = problem_expert.get_function("(distance wp1 wp2)")
    assert func_2 is not None
    assert func_2.name == "distance"
    assert len(func_2.parameters) == 2
    assert func_2.parameters[0].name == "wp1"
    assert func_2.parameters[0].type == "waypoint"
    assert func_2.parameters[1].name == "wp2"
    assert func_2.parameters[1].type == "waypoint"
    assert func_2.value == 15

    assert not problem_expert.remove_function(f3)
    assert problem_expert.remove_function(f2)

    functions = problem_expert.get_functions()
    assert len(functions) == 1

def test_problem_expert_addget_goals():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    # ---- INSTANCES ----
    assert problem_expert.add_instance(Param(name="paco", type="person"))
    assert problem_expert.add_instance(Param(name="r2d2", type="robot"))
    assert problem_expert.add_instance(Param(name="bedroom", type="room"))
    assert problem_expert.add_instance(Param(name="kitchen", type="room"))

    goal = Tree()

    root = NodeMsg()
    root.node_type = NodeMsg.AND
    root.node_id = 0
    root.children = [1, 2]
    goal.nodes.append(root)

    g1 = NodeMsg()
    g1.node_type = NodeMsg.PREDICATE
    g1.node_id = 1
    g1.name = "robot_at"
    g1.parameters = [
        Param(name="r2d2", type="robot"),
        Param(name="bedroom", type="room"),
    ]
    goal.nodes.append(g1)

    g2 = NodeMsg()
    g2.node_type = NodeMsg.PREDICATE
    g2.node_id = 2
    g2.name = "person_at"
    g2.parameters = [
        Param(name="paco", type="person"),
        Param(name="kitchen", type="room"),
    ]
    goal.nodes.append(g2)

    assert problem_expert.add_goal(goal)

    returned_goal = problem_expert.get_goal()
    assert returned_goal is not None
    assert len(returned_goal.nodes) == 3

    goal2 = Tree()

    root = NodeMsg()
    root.node_type = NodeMsg.AND
    root.node_id = 0
    root.children = [1, 2]
    goal2.nodes.append(root)

    g1 = NodeMsg()
    g1.node_type = NodeMsg.PREDICATE
    g1.node_id = 1
    g1.name = "robot_at"
    g1.parameters = [
        Param(name="r2d2", type="robot"),
        Param(name="bedroom", type="room"),
    ]
    goal2.nodes.append(g1)

    not_node = NodeMsg()
    not_node.node_type = NodeMsg.NOT
    not_node.node_id = 2
    not_node.children = [3]
    goal2.nodes.append(not_node)

    inner = NodeMsg()
    inner.node_type = NodeMsg.PREDICATE
    inner.node_id = 3
    inner.name = "person_at"
    inner.parameters = [
        Param(name="paco", type="person"),
        Param(name="kitchen", type="room"),
    ]
    goal2.nodes.append(inner)

    assert problem_expert.add_goal(goal2)

    returned_goal = problem_expert.get_goal()
    assert returned_goal is not None
    assert len(returned_goal.nodes) == 4
    assert returned_goal.nodes[2].node_type == NodeMsg.NOT

    assert problem_expert.remove_goal()
    assert problem_expert.remove_goal()  # doble clear permitido

    empty_goal = problem_expert.get_goal()
    assert empty_goal is None or len(empty_goal.nodes) == 0

    goal4 = Tree()

    root = NodeMsg()
    root.node_type = NodeMsg.AND
    root.node_id = 0
    root.children = [1, 4]
    goal4.nodes.append(root)

    # OR
    or_node = NodeMsg()
    or_node.node_type = NodeMsg.OR
    or_node.node_id = 1
    or_node.children = [2, 3]
    goal4.nodes.append(or_node)

    r1 = NodeMsg()
    r1.node_type = NodeMsg.PREDICATE
    r1.node_id = 2
    r1.name = "robot_at"
    r1.parameters = [
        Param(name="r2d2", type="robot"),
        Param(name="bedroom", type="room"),
    ]
    goal4.nodes.append(r1)

    r2 = NodeMsg()
    r2.node_type = NodeMsg.PREDICATE
    r2.node_id = 3
    r2.name = "robot_at"
    r2.parameters = [
        Param(name="r2d2", type="robot"),
        Param(name="kitchen", type="room"),
    ]
    goal4.nodes.append(r2)

    not_node = NodeMsg()
    not_node.node_type = NodeMsg.NOT
    not_node.node_id = 4
    not_node.children = [5]
    goal4.nodes.append(not_node)

    inner = NodeMsg()
    inner.node_type = NodeMsg.PREDICATE
    inner.node_id = 5
    inner.name = "person_at"
    inner.parameters = [
        Param(name="paco", type="person"),
        Param(name="kitchen", type="room"),
    ]
    goal4.nodes.append(inner)

    assert problem_expert.add_goal(goal4)


def test_problem_expert_empty_goals():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    goal = Tree()
    assert not problem_expert.is_valid_goal(goal)

def test_problem_expert_get_problem():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    # -------- INSTANCES --------
    assert problem_expert.add_instance(Param(name="paco", type="person"))
    assert problem_expert.add_instance(Param(name="r2d2", type="robot"))
    assert problem_expert.add_instance(Param(name="bedroom", type="room"))
    assert problem_expert.add_instance(Param(name="kitchen", type="room"))

    # -------- PREDICATES --------
    def make_pred(name, *args):
        p = NodeMsg()
        p.node_type = NodeMsg.PREDICATE
        p.name = name
        for n, t in args:
            p.parameters.append(Param(name=n, type=t))
        return p

    assert problem_expert.add_predicate(
        make_pred("robot_at", ("r2d2", "robot"), ("bedroom", "room"))
    )
    assert problem_expert.add_predicate(
        make_pred("robot_at", ("r2d2", "robot"), ("kitchen", "room"))
    )
    assert problem_expert.add_predicate(
        make_pred("person_at", ("paco", "person"), ("bedroom", "room"))
    )
    assert problem_expert.add_predicate(
        make_pred("person_at", ("paco", "person"), ("kitchen", "room"))
    )

    # -------- GOAL --------
    goal = Tree()

    root = NodeMsg()
    root.node_type = NodeMsg.AND
    root.node_id = 0
    root.children = [1, 2]
    goal.nodes.append(root)

    g1 = make_pred("robot_at", ("r2d2", "robot"), ("bedroom", "room"))
    g1.node_id = 1
    goal.nodes.append(g1)

    g2 = make_pred("person_at", ("paco", "person"), ("kitchen", "room"))
    g2.node_id = 2
    goal.nodes.append(g2)

    assert problem_expert.add_goal(goal)

    # -------- GET PROBLEM --------
    problem_pddl = problem_expert.get_problem()

    assert "(problem" in problem_pddl
    assert ":domain simple" in problem_pddl

    # objects
    assert "paco" in problem_pddl
    assert "r2d2" in problem_pddl
    assert "bedroom" in problem_pddl
    assert "kitchen" in problem_pddl

    # init predicates
    assert "(robot_at r2d2 bedroom)" in problem_pddl
    assert "(robot_at r2d2 kitchen)" in problem_pddl
    assert "(person_at paco bedroom)" in problem_pddl
    assert "(person_at paco kitchen)" in problem_pddl

    # goal
    assert "(robot_at r2d2 bedroom)" in problem_pddl
    assert "(person_at paco kitchen)" in problem_pddl

    # -------- CLEAR --------
    assert problem_expert.clear_knowledge()

    assert len(problem_expert.get_predicates()) == 0
    assert len(problem_expert.get_functions()) == 0
    assert len(problem_expert.get_instances()) == 0

