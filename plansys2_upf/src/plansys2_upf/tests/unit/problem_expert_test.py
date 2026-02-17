import pathlib
import pytest

from ament_index_python.packages import get_package_share_directory

from plansys2_msgs.msg import Node as NodeMsg, Tree

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
    assert problem_expert.add_instance(("Paco", "person"))
    assert problem_expert.add_instance(("Paco", "person"))   # duplicado permitido
    assert not problem_expert.add_instance(("Paco", "room"))
    assert not problem_expert.add_instance(("Paco", "SCIENTIFIC"))

    assert problem_expert.add_instance(("r2d2", "robot"))
    assert problem_expert.add_instance(("ur5e", "Robot"))  # case-insensitive

    instances = problem_expert.get_instances()
    assert len(instances) == 3

    assert instances[0].name == "Paco"
    assert instances[0].type == "person"
    assert instances[1].name == "r2d2"
    assert instances[1].type == "robot"
    assert instances[2].name == "ur5e"
    assert instances[2].type == "robot"

    # ---- REMOVE INSTANCE ----
    assert problem_expert.remove_instance(("Paco", "person"))

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
    assert problem_expert.add_instance(("bedroom", "room"))
    assert problem_expert.add_instance(("kitchen", "room_with_teleporter"))

    # ---- FUNCTION 1 ----
    f1 = NodeMsg()
    f1.node_type = NodeMsg.FUNCTION
    f1.name = "room_distance"
    f1.parameters.append(NodeMsg.Param(name="bedroom", type="room"))
    f1.parameters.append(NodeMsg.Param(name="kitchen", type="room_with_teleporter"))
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
    f2.parameters.append(NodeMsg.Param(name="kitchen", type="room_with_teleporter"))
    f2.parameters.append(NodeMsg.Param(name="bedroom", type="room"))
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
    f3.parameters.append(NodeMsg.Param(name="bedroom", type="room"))
    f3.parameters.append(NodeMsg.Param(name="kitchen", type="room_with_teleporter"))
    f3.value = 2.34

    assert not problem_expert.add_function(f3)
    assert not problem_expert.remove_function(f3)

    assert problem_expert.remove_instance(("kitchen", "room_with_teleporter"))

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
    p1.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
    p1.parameters.append(NodeMsg.Param(name="bedroom", type="room"))

    p2 = NodeMsg()
    p2.node_type = NodeMsg.PREDICATE
    p2.name = "robot_at"
    p2.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
    p2.parameters.append(NodeMsg.Param(name="kitchen", type="room"))

    p3 = NodeMsg()
    p3.node_type = NodeMsg.PREDICATE
    p3.name = "person_at"
    p3.parameters.append(NodeMsg.Param(name="paco", type="person"))
    p3.parameters.append(NodeMsg.Param(name="bedroom", type="room"))

    p4 = NodeMsg()
    p4.node_type = NodeMsg.PREDICATE
    p4.name = "person_at"
    p4.parameters.append(NodeMsg.Param(name="paco", type="person"))
    p4.parameters.append(NodeMsg.Param(name="kitchen", type="room"))

    p5 = NodeMsg()
    p5.node_type = NodeMsg.PREDICATE
    p5.name = "person_at"
    p5.parameters.append(NodeMsg.Param(name="paco", type="person"))
    p5.parameters.append(NodeMsg.Param(name="kitchen", type="room"))
    p5.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
    p5.parameters.append(NodeMsg.Param(name="bedroom", type="room"))

    p6 = NodeMsg()
    p6.node_type = NodeMsg.PREDICATE
    p6.name = "person_at"
    p6.parameters.append(NodeMsg.Param(name="kitchen", type="room"))
    p6.parameters.append(NodeMsg.Param(name="paco", type="person"))

    # ---- ADD INSTANCES ----
    assert problem_expert.add_instance(("paco", "person"))
    assert problem_expert.add_instance(("r2d2", "robot"))
    assert problem_expert.add_instance(("bedroom", "room"))
    assert problem_expert.add_instance(("kitchen", "room"))

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

    assert problem_expert.add_instance(("bathroom", "room_with_teleporter"))

    p7 = NodeMsg()
    p7.node_type = NodeMsg.PREDICATE
    p7.name = "is_teleporter_enabled"
    p7.parameters.append(NodeMsg.Param(name="bathroom", type="room_with_teleporter"))

    assert problem_expert.add_predicate(p7)

    p8 = NodeMsg()
    p8.node_type = NodeMsg.PREDICATE
    p8.name = "is_teleporter_destination"
    p8.parameters.append(NodeMsg.Param(name="bathroom", type="room_with_teleporter"))

    assert problem_expert.add_predicate(p8)

    assert problem_expert.remove_instance(("bathroom", "room_with_teleporter"))

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
    f1.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
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
    f2.parameters.append(NodeMsg.Param(name="wp1", type="waypoint"))
    f2.parameters.append(NodeMsg.Param(name="wp2", type="waypoint"))
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
    f3.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
    f3.parameters.append(NodeMsg.Param(name="wp1", type="waypoint"))

    # ---------- function_4 (inválida) ----------
    f4 = NodeMsg()
    f4.node_type = NodeMsg.FUNCTION
    f4.name = "distance"
    f4.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
    f4.parameters.append(NodeMsg.Param(name="wp1", type="waypoint"))

    # ---------- add instances ----------
    assert problem_expert.add_instance(("r2d2", "robot"))
    assert problem_expert.add_instance(("wp1", "waypoint"))
    assert problem_expert.add_instance(("wp2", "waypoint"))

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

# def test_problem_expert_addget_goals():

#     pkgpath = get_package_share_directory("plansys2_problem_expert")
#     domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

#     domain_expert = DomainUPFReader()
#     domain_expert.load_pddl(str(domain_path))

#     problem_expert = ProblemUPFExpert(domain_expert)

#     assert problem_expert.add_instance(("paco", "person"))
#     assert problem_expert.add_instance(("r2d2", "robot"))
#     assert problem_expert.add_instance(("bedroom", "room"))
#     assert problem_expert.add_instance(("kitchen", "room"))

#     goal = Tree()
#     pddl.fromString(goal, "(and (robot_at r2d2 bedroom)(person_at paco kitchen))")
#     assert pddl.toString(goal) == "(and (robot_at r2d2 bedroom)(person_at paco kitchen))"

#     goal2 = Tree()
#     pddl.fromString(goal2, "(and (robot_at r2d2 bedroom)(not(person_at paco kitchen)))")
#     assert (
#         pddl.toString(goal2)
#         == "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))"
#     )

#     assert problem_expert.set_goal(goal)
#     assert problem_expert.set_goal(goal2)

#     assert (
#         pddl.toString(problem_expert.get_goal())
#         == "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))"
#     )

#     goal3 = problem_expert.get_goal()
#     assert (
#         pddl.toString(goal3)
#         == "(and (robot_at r2d2 bedroom)(not (person_at paco kitchen)))"
#     )

#     assert problem_expert.clear_goal()
#     assert problem_expert.clear_goal()

#     assert pddl.toString(problem_expert.get_goal()) == ""

#     goal4 = Tree()
#     pddl.fromString(
#         goal4,
#         "(and (or (robot_at r2d2 bedroom) (robot_at r2d2 kitchen)) "
#         "(not (person_at paco kitchen)))",
#     )

#     assert problem_expert.set_goal(goal4)

def test_problem_expert_empty_goals():

    pkgpath = get_package_share_directory("plansys2_problem_expert")
    domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

    domain_expert = DomainUPFReader()
    domain_expert.load_pddl(str(domain_path))

    problem_expert = ProblemUPFExpert(domain_expert)

    goal = Tree()
    assert not problem_expert.is_valid_goal(goal)

# def test_problem_expert_get_problem():

#     pkgpath = get_package_share_directory("plansys2_problem_expert")
#     domain_path = pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"

#     domain_expert = DomainUPFReader()
#     domain_expert.load_pddl(str(domain_path))

#     problem_expert = ProblemUPFExpert(domain_expert)

#     p1 = NodeMsg()
#     p1.node_type = NodeMsg.PREDICATE
#     p1.name = "robot_at"
#     p1.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
#     p1.parameters.append(NodeMsg.Param(name="bedroom", type="room"))

#     p2 = NodeMsg()
#     p2.node_type = NodeMsg.PREDICATE
#     p2.name = "robot_at"
#     p2.parameters.append(NodeMsg.Param(name="r2d2", type="robot"))
#     p2.parameters.append(NodeMsg.Param(name="kitchen", type="room"))

#     p3 = NodeMsg()
#     p3.node_type = NodeMsg.PREDICATE
#     p3.name = "person_at"
#     p3.parameters.append(NodeMsg.Param(name="paco", type="person"))
#     p3.parameters.append(NodeMsg.Param(name="bedroom", type="room"))

#     p4 = NodeMsg()
#     p4.node_type = NodeMsg.PREDICATE
#     p4.name = "person_at"
#     p4.parameters.append(NodeMsg.Param(name="paco", type="person"))
#     p4.parameters.append(NodeMsg.Param(name="kitchen", type="room"))

#     assert problem_expert.add_instance(("paco", "person"))
#     assert problem_expert.add_instance(("r2d2", "robot"))
#     assert problem_expert.add_instance(("bedroom", "room"))
#     assert problem_expert.add_instance(("kitchen", "room"))

#     assert problem_expert.add_predicate(p1)
#     assert problem_expert.add_predicate(p2)
#     assert problem_expert.add_predicate(p3)
#     assert problem_expert.add_predicate(p4)

#     goal = Tree()
#     pddl.fromString(goal, "(and (robot_at r2d2 bedroom)(person_at paco kitchen))")
#     assert problem_expert.set_goal(goal)

#     expected = (
#         "( define ( problem problem_1 )\n"
#         "( :domain simple )\n"
#         "( :objects\n"
#         "\tpaco - person\n"
#         "\tr2d2 - robot\n"
#         "\tbedroom kitchen - room\n"
#         ")\n"
#         "( :init\n"
#         "\t( robot_at r2d2 bedroom )\n"
#         "\t( robot_at r2d2 kitchen )\n"
#         "\t( person_at paco bedroom )\n"
#         "\t( person_at paco kitchen )\n"
#         ")\n"
#         "( :goal\n"
#         "\t( and\n"
#         "\t\t( robot_at r2d2 bedroom )\n"
#         "\t\t( person_at paco kitchen )\n"
#         "\t))\n"
#         ")\n"
#     )

#     assert problem_expert.get_problem() == expected

#     assert problem_expert.clear_knowledge()
#     assert len(problem_expert.get_predicates()) == 0
#     assert len(problem_expert.get_functions()) == 0
#     assert len(problem_expert.get_instances()) == 0

# def test_problem_expert_add_problem_with_constants():

#     pkgpath = get_package_share_directory("plansys2_problem_expert")

#     # Cargar dominio con constantes
#     with open(pathlib.Path(pkgpath) / "pddl/domain_simple_constants.pddl") as f:
#         domain_str = f.read()

#     domain_expert = DomainUPFReader()
#     domain_expert.load_pddl(str(pathlib.Path(pkgpath) / "pddl/domain_simple_constants.pddl"))

#     problem_expert = ProblemUPFExpert(domain_expert)

#     # Cargar primer problema
#     with open(pathlib.Path(pkgpath) / "pddl/problem_simple_constants_1.pddl") as f:
#         problem_1_str = f.read()

#     assert problem_expert.add_problem(problem_1_str)

#     # Tipos válidos
#     assert problem_expert.is_valid_type("robot")
#     assert problem_expert.is_valid_type("person")
#     assert problem_expert.is_valid_type("room")
#     assert problem_expert.is_valid_type("teleporter_room")
#     assert problem_expert.is_valid_type("message")

#     assert len(problem_expert.get_instances()) == 7
#     assert len(problem_expert.get_predicates()) == 2
#     assert len(problem_expert.get_functions()) == 0

#     # Existencia de instancias
#     assert problem_expert.exist_instance("leia")
#     assert problem_expert.exist_instance("lema")
#     assert problem_expert.exist_instance("jack")
#     assert problem_expert.exist_instance("john")
#     assert problem_expert.exist_instance("kitchen")
#     assert problem_expert.exist_instance("bedroom")
#     assert problem_expert.exist_instance("m1")

#     assert not problem_expert.exist_instance("r2d2")
#     assert not problem_expert.exist_instance("hallway")
#     assert not problem_expert.exist_instance("m2")

#     # Predicados existentes
#     assert problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(robot_at leia kitchen)")
#     )
#     assert problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(person_at jack bedroom)")
#     )

#     # Goal
#     assert (
#         pddl.toString(problem_expert.get_goal())
#         == "(and (robot_talk leia m1 jack))"
#     )

#     expected_problem = (
#         "( define ( problem problem_1 )\n"
#         "( :domain plansys2 )\n"
#         "( :objects\n"
#         "\tm1 - message\n"
#         "\tkitchen bedroom - room\n"
#         ")\n"
#         "( :init\n"
#         "\t( robot_at leia kitchen )\n"
#         "\t( person_at jack bedroom )\n"
#         ")\n"
#         "( :goal\n"
#         "\t( and\n"
#         "\t\t( robot_talk leia m1 jack )\n"
#         "\t))\n"
#         ")\n"
#     )

#     assert problem_expert.get_problem() == expected_problem

#     # Clear knowledge
#     assert problem_expert.clear_knowledge()
#     assert len(problem_expert.get_predicates()) == 0
#     assert len(problem_expert.get_functions()) == 0
#     assert len(problem_expert.get_instances()) == 0

#     expected_empty = (
#         "( define ( problem problem_1 )\n"
#         "( :domain plansys2 )\n"
#         "( :objects\n)\n"
#         "( :init\n)\n"
#         "( :goal\n\t( and\n\t))\n"
#         ")\n"
#     )

#     assert problem_expert.get_problem() == expected_empty

#     # Segundo problema
#     with open(pathlib.Path(pkgpath) / "pddl/problem_simple_constants_2.pddl") as f:
#         problem_2_str = f.read()

#     assert problem_expert.add_problem(problem_2_str)

#     assert problem_1_str != problem_2_str
#     assert problem_expert.get_problem() != problem_2_str
#     assert problem_expert.get_problem() == problem_1_str

# def test_problem_expert_is_goal_satisfied():

#     pkgpath = get_package_share_directory("plansys2_problem_expert")

#     domain_expert = DomainUPFReader()
#     domain_expert.load_pddl(str(pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"))

#     problem_expert = ProblemUPFExpert(domain_expert)

#     assert problem_expert.add_instance(("leia", "robot"))
#     assert problem_expert.add_instance(("Jack", "person"))
#     assert problem_expert.add_instance(("kitchen", "room"))
#     assert problem_expert.add_instance(("bedroom", "room"))
#     assert problem_expert.add_instance(("m1", "message"))

#     assert problem_expert.add_predicate(
#         pddl.fromStringPredicate("(robot_at leia kitchen)")
#     )
#     assert problem_expert.add_predicate(
#         pddl.fromStringPredicate("(person_at Jack bedroom)")
#     )

#     expression = "(and (robot_talk leia m1 Jack))"
#     goal = Tree()
#     pddl.fromString(goal, expression)

#     assert pddl.toString(goal) == "(and (robot_talk leia m1 Jack))"

#     assert problem_expert.set_goal(goal)
#     assert not problem_expert.is_goal_satisfied(goal)

#     assert problem_expert.add_predicate(
#         pddl.fromStringPredicate("(robot_talk leia m1 Jack)")
#     )

#     assert problem_expert.is_goal_satisfied(goal)

# def test_problem_expert_exist_predicate():

#     pkgpath = get_package_share_directory("plansys2_problem_expert")

#     domain_expert = DomainUPFReader()
#     domain_expert.load_pddl(str(pathlib.Path(pkgpath) / "pddl/domain_simple_derived.pddl"))

#     problem_expert = ProblemUPFExpert(domain_expert)

#     with open(pathlib.Path(pkgpath) / "pddl/problem_simple_1.pddl") as f:
#         assert problem_expert.add_problem(f.read())

#     assert problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(robot_at leia kitchen)")
#     )
#     assert problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(inferred-robot_at leia kitchen)")
#     )
#     assert problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(person_at jack bedroom)")
#     )
#     assert problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(inferred-person_at jack bedroom)")
#     )

#     assert not problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(inferred-person_at jack kitchen)")
#     )
#     assert not problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(inferred-robot_at leia bedroom)")
#     )

#     problem_expert.remove_predicate(
#         pddl.fromStringPredicate("(robot_at leia kitchen)")
#     )
#     problem_expert.remove_predicate(
#         pddl.fromStringPredicate("(person_at jack bedroom)")
#     )

#     assert not problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(inferred-person_at jack bedroom)")
#     )
#     assert not problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(inferred-robot_at leia kitchen)")
#     )
#     assert not problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(person_at jack bedroom)")
#     )
#     assert not problem_expert.exist_predicate(
#         pddl.fromStringPredicate("(robot_at leia kitchen)")
#     )

# def test_problem_expert_get_predicate_with_derived():

#     pkgpath = get_package_share_directory("plansys2_problem_expert")

#     domain_expert = DomainUPFReader()
#     domain_expert.load_pddl(str(pathlib.Path(pkgpath) / "pddl/domain_simple_derived.pddl"))

#     problem_expert = ProblemUPFExpert(domain_expert)

#     with open(pathlib.Path(pkgpath) / "pddl/problem_simple_1.pddl") as f:
#         assert problem_expert.add_problem(f.read())

#     predicates = problem_expert.get_predicates()
#     assert len(predicates) == 4

#     predicate_names = [pddl.toString(p) for p in predicates]

#     assert "(inferred-robot_at leia kitchen)" in predicate_names
#     assert "(person_at jack bedroom)" in predicate_names
#     assert "(inferred-person_at jack bedroom)" in predicate_names
#     assert "(inferred-person_at jack kitchen)" not in predicate_names
#     assert "(inferred-robot_at leia bedroom)" not in predicate_names

#     problem_expert.remove_predicate(
#         pddl.fromStringPredicate("(robot_at leia kitchen)")
#     )
#     problem_expert.remove_predicate(
#         pddl.fromStringPredicate("(person_at jack bedroom)")
#     )

#     predicates = problem_expert.get_predicates()
#     predicate_names2 = [pddl.toString(p) for p in predicates]

#     assert "(inferred-person_at jack bedroom)" not in predicate_names2
#     assert "(inferred-robot_at leia kitchen)" not in predicate_names2
