import threading
import time
import pathlib

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from lifecycle_msgs.msg import Transition, State
from lifecycle_msgs.srv import ChangeState, GetState

from ament_index_python.packages import get_package_share_directory

from plansys2_upf.domain_expert_node_upf.DomainExpertNode import (
    DomainUPFExpertNode as DomainExpertNode
)

# ---------------- FIXTURE ----------------

@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()

# ================== HELPERS ==================

def spin_executor(executor, stop_flag):
    while not stop_flag.is_set():
        try:
            executor.spin_once(timeout_sec=0.1)
        except rclpy.executors.ExternalShutdownException:
            break
        except Exception:
            pass


def call_change_state(helper_node: Node, transition_id: int, timeout_sec: float = 2.0):
    client = helper_node.create_client(ChangeState, "/domain_expert/change_state")
    assert client.wait_for_service(timeout_sec=timeout_sec)

    req = ChangeState.Request()
    req.transition.id = transition_id

    future = client.call_async(req)
    rclpy.spin_until_future_complete(helper_node, future)

    return future.result().success


def get_current_state(helper_node: Node) -> int:
    client = helper_node.create_client(GetState, "/domain_expert/get_state")
    assert client.wait_for_service(timeout_sec=2.0)

    req = GetState.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(helper_node, future)

    return future.result().current_state.id


def wait_for_primary_state(helper_node: Node, expected_states, timeout_sec: float = 5.0):
    start = time.time()
    while time.time() - start < timeout_sec:
        state = get_current_state(helper_node)
        if state in expected_states:
            return state
        time.sleep(0.05)
    raise TimeoutError(f"Timeout esperando a estado primario. Último estado: {state}")


# ================== TESTS ==================

def test_domain_expert_lifecycle(rclpy_init):

    helper_node = Node("helper")
    domain_node = DomainExpertNode()

    pkgpath = get_package_share_directory("plansys2_domain_expert")
    domain_node.set_parameters([
        rclpy.parameter.Parameter(
            "model_file",
            rclpy.parameter.Parameter.Type.STRING,
            str(pathlib.Path(pkgpath) / "pddl/domain_simple.pddl")
        )
    ])

    executor = SingleThreadedExecutor()
    executor.add_node(domain_node)
    executor.add_node(helper_node)

    stop_flag = threading.Event()
    thread = threading.Thread(
        target=spin_executor,
        args=(executor, stop_flag),
        daemon=True,
    )
    thread.start()

    # ---- CONFIGURE (VÍA SERVICIO, como C++) ----
    success = call_change_state(helper_node, Transition.TRANSITION_CONFIGURE)
    assert success

    state = wait_for_primary_state(
        helper_node,
        expected_states=(State.PRIMARY_STATE_INACTIVE,)
    )
    assert state == State.PRIMARY_STATE_INACTIVE

    # ---- ACTIVATE ----
    success = call_change_state(helper_node, Transition.TRANSITION_ACTIVATE)
    assert success

    state = wait_for_primary_state(
        helper_node,
        expected_states=(State.PRIMARY_STATE_ACTIVE,)
    )
    assert state == State.PRIMARY_STATE_ACTIVE
    domain_node.destroy_node()
    helper_node.destroy_node()
    stop_flag.set()
    thread.join()





def test_domain_expert_lifecycle_error(rclpy_init):

    helper_node = Node("helper_error")
    domain_node = DomainExpertNode()

    pkgpath = get_package_share_directory("plansys2_domain_expert")
    domain_node.set_parameters([
        rclpy.parameter.Parameter(
            "model_file",
            rclpy.parameter.Parameter.Type.STRING,
            str(pathlib.Path(pkgpath) / "pddl/domain_2_error.pddl")
        )
    ])

    executor = SingleThreadedExecutor()
    executor.add_node(domain_node)
    executor.add_node(helper_node)

    stop_flag = threading.Event()
    thread = threading.Thread(
        target=spin_executor,
        args=(executor, stop_flag),
        daemon=True,
    )
    thread.start()

    success = call_change_state(helper_node, Transition.TRANSITION_CONFIGURE)

    # En caso de error, debe volver a UNCONFIGURED
    state = wait_for_primary_state(
        helper_node,
        expected_states=(State.PRIMARY_STATE_UNCONFIGURED,)
    )
    assert state == State.PRIMARY_STATE_UNCONFIGURED
    domain_node.destroy_node()
    helper_node.destroy_node()
    stop_flag.set()
    thread.join()




