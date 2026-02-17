import threading
import time
import pathlib

import pytest

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

from ament_index_python.packages import get_package_share_directory

from plansys2_msgs.msg import Node as NodeMsg, Knowledge, Tree, Param

from plansys2_msgs.srv import (
    AffectParam,
    AffectNode,
    AddProblemGoal,
    IsProblemGoalSatisfied,
    ClearProblemKnowledge,
    GetProblemInstances,
    GetStates
)

from plansys2_upf.domain_expert_node_upf.DomainExpertNode import DomainUPFExpertNode
from plansys2_upf.problem_expert_node_upf.ProblemExpertNode import ProblemUPFExpertNode


@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def spin_executor(executor, stop_flag):
    while not stop_flag.is_set():
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception:
            break


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
# TEST 1: INSTANCIAS, PREDICADOS Y CLEAR KNOWLEDGE
# ============================================================

def test_problem_expert_addget_instances(rclpy_init):

    test_node = Node("test_problem_expert_node")
    test_node_2 = Node("test_problem_expert_node_2")

    domain_node = DomainUPFExpertNode()
    problem_node = ProblemUPFExpertNode()

    pkgpath = get_package_share_directory("plansys2_problem_expert")

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

    executor = SingleThreadedExecutor()
    executor.add_node(domain_node)
    executor.add_node(problem_node)
    executor.add_node(test_node_2)

    stop_flag = threading.Event()
    spin_thread = threading.Thread(
        target=spin_executor,
        args=(executor, stop_flag),
        daemon=True,
    )
    spin_thread.start()

    # -------- Lifecycle --------
    assert change_state("domain_expert", test_node, Transition.TRANSITION_CONFIGURE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_CONFIGURE)
    assert change_state("domain_expert", test_node, Transition.TRANSITION_ACTIVATE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_ACTIVATE)

    time.sleep(0.3)

    # -------- Knowledge subscriber --------
    from rclpy.qos import QoSProfile, DurabilityPolicy

    qos = QoSProfile(depth=100)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    last_knowledge_msg = None
    knowledge_msg_counter = 0

    def knowledge_callback(msg):
        nonlocal last_knowledge_msg, knowledge_msg_counter
        last_knowledge_msg = msg
        knowledge_msg_counter += 1

    sub = test_node_2.create_subscription(
        Knowledge,
        "problem_expert/knowledge",
        knowledge_callback,
        qos,
    )

    time.sleep(0.2)

    # -------- Clients --------
    add_instance = test_node.create_client(AffectParam, "problem_expert/add_problem_instance")
    remove_instance = test_node.create_client(AffectParam, "problem_expert/remove_problem_instance")
    get_instances = test_node.create_client(GetProblemInstances, "problem_expert/get_problem_instances")
    add_predicate = test_node.create_client(AffectNode, "problem_expert/add_problem_predicate")
    get_predicates = test_node.create_client(GetStates, "problem_expert/get_problem_predicates")
    set_goal = test_node.create_client(AddProblemGoal, "problem_expert/add_problem_goal")
    clear_knowledge = test_node.create_client(ClearProblemKnowledge, "problem_expert/clear_problem_knowledge")

    for c in [add_instance, remove_instance, get_instances,
              add_predicate, get_predicates,
              set_goal, clear_knowledge]:
        c.wait_for_service()

    # ----------------------------------------------------
    # ADD INSTANCES
    # ----------------------------------------------------

    def add_inst(name, type_):
        req = AffectParam.Request()
        req.param.name = name
        req.param.type = type_
        future = add_instance.call_async(req)
        rclpy.spin_until_future_complete(test_node, future)
        return future.result().success

    assert add_inst("Paco", "person")
    assert add_inst("Paco", "person")
    assert not add_inst("Paco", "SCIENTIFIC")
    assert add_inst("bedroom", "room")
    assert add_inst("kitchen", "room")

    time.sleep(0.5)

    assert knowledge_msg_counter == 4
    assert last_knowledge_msg.instances == ["Paco", "bedroom", "kitchen"]
    assert last_knowledge_msg.predicates == []
    assert last_knowledge_msg.goal == ""

    # Check getInstances
    req = GetProblemInstances.Request()
    future = get_instances.call_async(req)
    rclpy.spin_until_future_complete(test_node, future)
    instances = future.result().instances

    assert len(instances) == 3
    assert instances[0].name == "Paco"
    assert instances[0].type == "person"
    assert instances[1].name == "bedroom"
    assert instances[1].type == "room"
    assert instances[2].name == "kitchen"
    assert instances[2].type == "room"

    # ----------------------------------------------------
    # ADD r2d2
    # ----------------------------------------------------

    assert add_inst("r2d2", "robot")

    time.sleep(0.5)

    assert knowledge_msg_counter == 5
    assert len(last_knowledge_msg.instances) == 4
    assert last_knowledge_msg.instances[3] == "r2d2"

    # ----------------------------------------------------
    # REMOVE PACO
    # ----------------------------------------------------

    req = AffectParam.Request()
    req.param.name = "Paco"
    req.param.type = "person"
    future = remove_instance.call_async(req)
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().success

    time.sleep(0.5)

    assert knowledge_msg_counter == 6
    assert last_knowledge_msg.instances == ["bedroom", "kitchen", "r2d2"]

    # ----------------------------------------------------
    # ADD PREDICATES
    # ----------------------------------------------------

    def add_pred(name, p1, t1, p2, t2):
        node = NodeMsg()
        node.node_type = NodeMsg.PREDICATE
        node.name = name

        param1 = Param()
        param1.name = p1
        param1.type = t1
        node.parameters.append(param1)

        param2 = Param()
        param2.name = p2
        param2.type = t2
        node.parameters.append(param2)

        req = AffectNode.Request()
        req.node = node
        future = add_predicate.call_async(req)
        rclpy.spin_until_future_complete(test_node, future)
        return future.result().success

    assert add_pred("robot_at", "r2d2", "robot", "bedroom", "room")
    assert add_pred("robot_at", "r2d2", "robot", "kitchen", "room")

    time.sleep(0.5)

    assert knowledge_msg_counter == 8
    assert len(last_knowledge_msg.predicates) == 2
    assert last_knowledge_msg.predicates[0] == "(robot_at r2d2 bedroom)"
    assert last_knowledge_msg.predicates[1] == "(robot_at r2d2 kitchen)"

    # ----------------------------------------------------
    # SET GOAL
    # ----------------------------------------------------

    goal_tree = Tree()
    goal_node = NodeMsg()
    goal_node.node_type = NodeMsg.PREDICATE
    goal_node.name = "robot_at"

    p1 = Param()
    p1.name = "r2d2"
    p1.type = "robot"

    p2 = Param()
    p2.name = "kitchen"
    p2.type = "room"

    goal_node.parameters = [p1, p2]
    goal_tree.nodes.append(goal_node)

    req = AddProblemGoal.Request()
    req.tree = goal_tree
    future = set_goal.call_async(req)
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().success

    time.sleep(0.5)

    assert knowledge_msg_counter == 9
    assert last_knowledge_msg.goal != ""

    # ----------------------------------------------------
    # CLEAR KNOWLEDGE
    # ----------------------------------------------------

    future = clear_knowledge.call_async(ClearProblemKnowledge.Request())
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().success

    time.sleep(0.5)

    # Verify cleared
    future = get_instances.call_async(GetProblemInstances.Request())
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().instances == []

    future = get_predicates.call_async(GetStates.Request())
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().states == []

    stop_flag.set()
    spin_thread.join()
    test_node.destroy_node()
    test_node_2.destroy_node()
    domain_node.destroy_node()
    problem_node.destroy_node()


# ============================================================
# TEST 2: GOAL SATISFIED
# ============================================================

def test_problem_expert_addget_goal_is_satisfied(rclpy_init):

    test_node = Node("test_problem_expert_node")
    test_node_2 = Node("test_problem_expert_node_2")

    domain_node = DomainUPFExpertNode()
    problem_node = ProblemUPFExpertNode()

    pkgpath = get_package_share_directory("plansys2_problem_expert")

    domain_node.set_parameters([
        rclpy.parameter.Parameter(
            "model_file",
            rclpy.parameter.Parameter.Type.STRING,
            str(pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"),
        )
    ])

    problem_node.set_parameters([
        rclpy.parameter.Parameter(
            "model_file",
            rclpy.parameter.Parameter.Type.STRING,
            str(pathlib.Path(pkgpath) / "pddl/domain_simple.pddl"),
        )
    ])

    executor = SingleThreadedExecutor()
    executor.add_node(domain_node)
    executor.add_node(problem_node)
    executor.add_node(test_node_2)

    stop_flag = threading.Event()
    spin_thread = threading.Thread(
        target=spin_executor,
        args=(executor, stop_flag),
        daemon=True,
    )
    spin_thread.start()

    # -------- Lifecycle --------
    assert change_state("domain_expert", test_node, Transition.TRANSITION_CONFIGURE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_CONFIGURE)

    assert change_state("domain_expert", test_node, Transition.TRANSITION_ACTIVATE)
    assert change_state("problem_expert", test_node, Transition.TRANSITION_ACTIVATE)

    time.sleep(0.3)

    # -------- Clients --------
    add_instance = test_node.create_client(
        AffectParam, "problem_expert/add_problem_instance"
    )
    add_instance.wait_for_service()

    add_predicate = test_node.create_client(
        AffectNode, "problem_expert/add_problem_predicate"
    )
    add_predicate.wait_for_service()

    add_goal = test_node.create_client(
        AddProblemGoal, "problem_expert/add_problem_goal"
    )
    add_goal.wait_for_service()

    is_goal_satisfied = test_node.create_client(
        IsProblemGoalSatisfied, "problem_expert/is_problem_goal_satisfied"
    )
    is_goal_satisfied.wait_for_service()

    print('antes add instances')

    # -------- Add instances --------
    def call_add_instance(name, type_):
        req = AffectParam.Request()
        req.param.name = name
        req.param.type = type_
        future = add_instance.call_async(req)
        rclpy.spin_until_future_complete(test_node, future)
        return future.result().success

    assert call_add_instance("Paco", "person")
    assert call_add_instance("bedroom", "room")
    assert call_add_instance("kitchen", "room")

    time.sleep(0.2)

    print('despues add instances y antes add predicate')

    # -------- Add predicate: (person_at Paco bedroom) --------
    pred = NodeMsg()
    pred.node_type = NodeMsg.PREDICATE
    pred.name = "person_at"

    p1 = Param()
    p1.name = "Paco"
    p1.type = "person"

    p2 = Param()
    p2.name = "bedroom"
    p2.type = "room"

    pred.parameters = [p1, p2]

    req = AffectNode.Request()
    req.node = pred

    future = add_predicate.call_async(req)
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().success

    time.sleep(0.2)

    print('despues add predicate y antes add goal')

    # -------- Set goal: (and (person_at Paco kitchen)) --------
    goal_tree = Tree()

    # Nodo AND raíz
    root = NodeMsg()
    root.node_type = NodeMsg.AND
    root.name = ""
    root.node_id = 0
    root.children = [1]

    goal_tree.nodes.append(root)
  
    # Nodo predicate

    p1 = Param()
    p1.name = "Paco"
    p1.type = "person"
    p1.sub_types = []

    p2 = Param()
    p2.name = "kitchen"
    p2.type = "room"
    p2.sub_types = []

    pred = NodeMsg()
    pred.node_type = NodeMsg.PREDICATE
    pred.node_id = 1
    pred.name = "person_at"
    pred.parameters = [p1, p2]

    goal_tree.nodes = [root, pred]

    # Ahora enviarlo
    goal_req = AddProblemGoal.Request()
    goal_req.tree = goal_tree

    future = add_goal.call_async(goal_req)
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().success

    time.sleep(0.2)

    print('despues add goal y antes check satisfied')

    # -------- Check NOT satisfied --------
    goal_check_req = IsProblemGoalSatisfied.Request()
    goal_check_req.tree = goal_tree

    future = is_goal_satisfied.call_async(goal_check_req)
    rclpy.spin_until_future_complete(test_node, future)

    print(future.result().satisfied, future)

    assert future.result().satisfied is False



    # -------- Add missing predicate: (person_at Paco kitchen) --------

    print('despues check satisfied y antes add predicate 2 (falta predicate)')

    pred2 = NodeMsg()
    pred2.node_type = NodeMsg.PREDICATE
    pred2.name = "person_at"

    p3 = Param()
    p3.name = "Paco"
    p3.type = "person"

    p4 = Param()
    p4.name = "kitchen"
    p4.type = "room"

    pred2.parameters = [p3, p4]

    req2 = AffectNode.Request()
    req2.node = pred2

    future = add_predicate.call_async(req2)
    rclpy.spin_until_future_complete(test_node, future)
    assert future.result().success

    time.sleep(0.2)

    print('despues add predicate 2 (falta predicate) y antes check satisfied 2')

    # -------- Now should be satisfied --------
    goal_check_req = IsProblemGoalSatisfied.Request()
    goal_check_req.tree = goal_tree

    future = is_goal_satisfied.call_async(goal_check_req)
    rclpy.spin_until_future_complete(test_node, future)

    assert future.result().satisfied is True

    print('despues check satisfied 2 y antes de limpiar')

    # -------- Cleanup --------
    stop_flag.set()
    spin_thread.join()

    test_node.destroy_node()
    test_node_2.destroy_node()
    domain_node.destroy_node()
    problem_node.destroy_node()
