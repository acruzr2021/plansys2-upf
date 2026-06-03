import rclpy
import threading
import time
import pathlib
import pytest

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from ament_index_python.packages import get_package_share_directory

# services planner
from plansys2_msgs.srv import GetPlan, GetProblem, GetDomain


# problem services
from plansys2_msgs.srv import (
    AffectParam,
    AffectNode,
    AddProblemGoal
)

from rclpy.parameter import Parameter

from plansys2_upf.domain_expert_node_upf.DomainExpertNode import DomainUPFExpertNode
from plansys2_upf.problem_expert_node_upf.ProblemExpertNode import ProblemUPFExpertNode
from plansys2_upf.planner_upf.PlannerUPFNode import PlannerUPFNode

@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


# ============================================================
# UTILS
# ============================================================

def spin_executor(executor, stop_flag):
    while not stop_flag.is_set():
        executor.spin_once(timeout_sec=0.1)


def change_state(node_name, test_node, transition_id):
    client = test_node.create_client(
        ChangeState, f"/{node_name}/change_state"
    )
    client.wait_for_service()

    req = ChangeState.Request()
    req.transition.id = transition_id

    future = client.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=2.0)
    return future.result().success


# ============================================================
# TEST 1: PLAN SIMPLE (equivalente a generate_plan_good)
# ============================================================
def test_generate_plan_simple(rclpy_init):

    test_node = Node("test_planner_node")

    domain_node = DomainUPFExpertNode()
    problem_node = ProblemUPFExpertNode()
    planner_node = PlannerUPFNode()

    pkgpath = get_package_share_directory("plansys2_planner")
    domain_file = str(pathlib.Path(pkgpath) / "pddl/domain_simple.pddl")

    domain_node.set_parameters([
        rclpy.parameter.Parameter("model_file",
                                  rclpy.parameter.Parameter.Type.STRING,
                                  domain_file)
    ])

    problem_node.set_parameters([
        rclpy.parameter.Parameter("model_file",
                                  rclpy.parameter.Parameter.Type.STRING,
                                  domain_file)
    ])

    planner_node.set_parameters([
        Parameter("plan_solver_plugins", Parameter.Type.STRING_ARRAY, ["UPF"]),
        Parameter(
            "UPF.plugin",
            Parameter.Type.STRING,
            "plansys2_upf.upf_planner_solver.upf_plan_solver.UPFPlanSolver"
        ),
        Parameter(
            "UPF.preferred_planner",
            Parameter.Type.STRING,
            "default"
        )
    ])

    executor = MultiThreadedExecutor()
    executor.add_node(domain_node)
    executor.add_node(problem_node)
    executor.add_node(planner_node)
    executor.add_node(test_node)

    stop_flag = threading.Event()
    thread = threading.Thread(target=spin_executor, args=(executor, stop_flag))
    thread.start()

    assert change_state("domain_expert", test_node, Transition.TRANSITION_CONFIGURE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_CONFIGURE)
    assert change_state("planner", test_node, Transition.TRANSITION_CONFIGURE)

    assert change_state("domain_expert", test_node, Transition.TRANSITION_ACTIVATE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_ACTIVATE)
    assert change_state("planner", test_node, Transition.TRANSITION_ACTIVATE)

    time.sleep(0.5)

    add_instance = test_node.create_client(AffectParam, "problem_expert/add_problem_instance")
    add_predicate = test_node.create_client(AffectNode, "problem_expert/add_problem_predicate")
    set_goal = test_node.create_client(AddProblemGoal, "problem_expert/add_problem_goal")
    get_plan = test_node.create_client(GetPlan, "planner/get_plan")

    for c in [add_instance, add_predicate, set_goal, get_plan]:
        c.wait_for_service()

    # =========================
    # HELPERS LIMPIOS
    # =========================

    from plansys2_msgs.msg import Node as PSNode, Param, Tree
    import itertools

    # generador de IDs
    id_counter = itertools.count(1)

    def next_id():
        return next(id_counter)

    def make_predicate(name, params):
        node = PSNode()
        node.node_type = 5  # PREDICATE
        node.node_id = next_id()

        node.name = name
        node.parameters = [Param(name=p[0], type=p[1]) for p in params]

        return node

    def make_and(children_nodes):
        node = PSNode()
        node.node_type = 2  # AND
        node.node_id = next_id()

        node.children = [c.node_id for c in children_nodes]

        return node

    def add_inst(name, type_):
        req = AffectParam.Request()
        req.param.name = name
        req.param.type = type_
        future = add_instance.call_async(req)
        rclpy.spin_until_future_complete(test_node, future)
        return future.result().success

    def add_pred(name, params):
        req = AffectNode.Request()
        req.node = make_predicate(name, params)
        future = add_predicate.call_async(req)
        rclpy.spin_until_future_complete(test_node, future)
        return future.result().success

    def set_goal_fn(predicates):

        req = AddProblemGoal.Request()

        tree = Tree()

        for name, params in predicates:
            node = PSNode()
            node.node_type = PSNode.PREDICATE
            node.name = name

            node.parameters = [
                Param(name=p[0], type=p[1]) for p in params
            ]

            tree.nodes.append(node)

        req.tree = tree

        future = set_goal.call_async(req)
        rclpy.spin_until_future_complete(test_node, future)

        return future.result().success

    # =========================
    # SETUP PROBLEM
    # =========================

    assert add_inst("leia", "robot")
    assert add_inst("francisco", "person")
    assert add_inst("message1", "message")

    assert add_inst("bedroom", "room")
    assert add_inst("kitchen", "room")
    assert add_inst("corridor", "room")

    assert add_pred("robot_at", [("leia", "robot"), ("kitchen", "room")])
    assert add_pred("person_at", [("francisco", "person"), ("bedroom", "room")])

    print("antes del goal")
    assert set_goal_fn([
        ("robot_talk", [
            ("leia", "robot"),
            ("message1", "message"),
            ("francisco", "person"),
        ])
    ])
    print("despues del goal")

    # =========================
    # GET PLAN
    # =========================

    get_problem_client = test_node.create_client(GetProblem, "problem_expert/get_problem")
    get_domain_client = test_node.create_client(GetDomain, "domain_expert/get_domain")

    # esperar servicios
    get_problem_client.wait_for_service()
    get_domain_client.wait_for_service()

    # pedir problem
    future = get_problem_client.call_async(GetProblem.Request())
    rclpy.spin_until_future_complete(test_node, future)
    problem_str = future.result().problem

    # pedir domain
    future = get_domain_client.call_async(GetDomain.Request())
    rclpy.spin_until_future_complete(test_node, future)
    domain_str = future.result().domain

    req = GetPlan.Request()
    req.domain = domain_str
    req.problem = problem_str

    future = get_plan.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=10.0)

    result = future.result()

    assert result is not None
    assert result.success
    assert len(result.plan.items) > 0

    # cleanup
    domain_node.destroy_node()
    problem_node.destroy_node()
    planner_node.destroy_node()
    executor.shutdown()
    stop_flag.set()
    thread.join()


# ============================================================
# TEST 2: DOMAIN CONSTANTS
# ============================================================

def test_generate_plan_with_problem_file(rclpy_init):

    test_node = Node("test_planner_constants")

    domain_node = DomainUPFExpertNode()
    problem_node = ProblemUPFExpertNode()
    planner_node = PlannerUPFNode()

    pkgpath = get_package_share_directory("plansys2_planner")

    domain_file = str(pathlib.Path(pkgpath) / "pddl/domain_simple_constants.pddl")
    problem_file = str(pathlib.Path(pkgpath) / "pddl/problem_simple_constants_1.pddl")

    domain_node.set_parameters([
        rclpy.parameter.Parameter("model_file",
                                  rclpy.parameter.Parameter.Type.STRING,
                                  domain_file)
    ])

    problem_node.set_parameters([
        rclpy.parameter.Parameter("model_file",
                                  rclpy.parameter.Parameter.Type.STRING,
                                  domain_file)
    ])

    planner_node.set_parameters([
        Parameter("plan_solver_plugins", Parameter.Type.STRING_ARRAY, ["UPF"]),
        Parameter(
            "UPF.plugin",
            Parameter.Type.STRING,
            "plansys2_upf.upf_planner_solver.upf_plan_solver.UPFPlanSolver"
        ),
        Parameter(
            "UPF.preferred_planner",
            Parameter.Type.STRING,
            "default"
        )
    ])

    executor = MultiThreadedExecutor()
    executor.add_node(domain_node)
    executor.add_node(problem_node)
    executor.add_node(planner_node)
    executor.add_node(test_node)

    stop_flag = threading.Event()
    thread = threading.Thread(target=spin_executor, args=(executor, stop_flag))
    thread.start()

    # lifecycle
    assert change_state("domain_expert", test_node, Transition.TRANSITION_CONFIGURE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_CONFIGURE)
    assert change_state("planner", test_node, Transition.TRANSITION_CONFIGURE)

    assert change_state("domain_expert", test_node, Transition.TRANSITION_ACTIVATE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_ACTIVATE)
    assert change_state("planner", test_node, Transition.TRANSITION_ACTIVATE)

    time.sleep(0.5)

    with open(problem_file, "r") as f:
        problem_str = f.read()

    from plansys2_msgs.srv import AddProblem

    add_problem = test_node.create_client(AddProblem, "problem_expert/add_problem")
    add_problem.wait_for_service()

    req = AddProblem.Request()
    req.problem = problem_str

    future = add_problem.call_async(req)
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().success

    # get plan
    get_plan = test_node.create_client(GetPlan, "planner/get_plan")
    get_plan.wait_for_service()

    # =========================
    # GET DOMAIN + PROBLEM
    # =========================

    get_problem_client = test_node.create_client(GetProblem, "problem_expert/get_problem")
    get_domain_client = test_node.create_client(GetDomain, "domain_expert/get_domain")

    get_problem_client.wait_for_service()
    get_domain_client.wait_for_service()

    # pedir problem (ya cargado con AddProblem)
    future = get_problem_client.call_async(GetProblem.Request())
    rclpy.spin_until_future_complete(test_node, future)
    problem_str = future.result().problem

    # pedir domain
    future = get_domain_client.call_async(GetDomain.Request())
    rclpy.spin_until_future_complete(test_node, future)
    domain_str = future.result().domain

    req = GetPlan.Request()
    req.domain = domain_str
    req.problem = problem_str

    future = get_plan.call_async(req)
    rclpy.spin_until_future_complete(test_node, future, timeout_sec=10.0)
    
    result = future.result()

    assert result is not None
    assert result.success
    assert len(result.plan.items) > 0
    domain_node.destroy_node()
    problem_node.destroy_node()
    planner_node.destroy_node()

    stop_flag.set()
    thread.join()