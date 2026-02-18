from ament_index_python.packages import get_package_share_directory
from plansys2_upf.domain_expert_node_upf.DomainUPFReader import DomainUPFReader


def test_exist_domain():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)

    # dominio charging
    charging_path = f"{pkgpath}/pddl/domain_charging.pddl"
    assert reader.load_pddl(charging_path)
    assert reader.get_name() == "charging"

    # extender dominio
    simple_path = f"{pkgpath}/pddl/domain_simple.pddl"
    assert reader.load_pddl(simple_path)
    assert reader.get_name() == "plansys2"

def test_get_domain():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")

    domain_str = str(reader.get_domain())

    # Comprobaciones semánticas equivalentes
    assert "plansys2" in domain_str
    assert "types" in domain_str.lower()
    assert "fluents" in domain_str.lower()

def test_get_domain_factory():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/factory.pddl")

    domain_str = str(reader.get_domain())

    assert "factory" in domain_str.lower()
    assert "types" in domain_str.lower()

def test_get_name():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/factory.pddl")

    name = reader.get_name()
    assert name == "factory"

def test_get_domain_constants():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple_constants.pddl")

    domain_str = str(reader.get_domain())

    # checks semánticos equivalentes
    assert "plansys2" in domain_str
    assert "constants" in domain_str.lower() or "objects" in domain_str.lower()

def test_get_types():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")

    types = reader.get_types()

    # orden estable para comparar
    types = sorted(types)

    expected = sorted([
        "object",
        "person",
        "message",
        "robot",
        "room",
        "teleporter_room"
    ])

    assert types == expected

def test_get_constants():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple_constants.pddl")

    consts_robot = sorted(reader.get_constants("robot"))
    consts_person = sorted(reader.get_constants("person"))

    assert consts_robot == ["leia", "lema"]
    assert consts_person == ["jack", "john"]

def test_get_predicates():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")

    predicates = reader.get_predicates()
    predicate_names = [p.name for p in predicates]

    expected = [
        "person_at",
        "robot_at",
        "robot_near_person",
        "robot_talk"
    ]

    assert len(predicate_names) == len(expected)
    assert sorted(predicate_names) == expected

def test_get_predicate_params():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")

    # robot_talk
    pred = reader.get_predicate("robot_talk")
    assert pred is not None
    assert pred.name == "robot_talk"
    assert len(pred.parameters) == 3

    assert pred.parameters[0].name == "?robot0"
    assert pred.parameters[0].type == "robot"
    assert pred.parameters[1].name == "?message1"
    assert pred.parameters[1].type == "message"
    assert pred.parameters[2].name == "?person2"
    assert pred.parameters[2].type == "person"

    # case-insensitive
    pred_upper = reader.get_predicate("ROBOT_TALK")
    assert pred_upper is not None

    # person_at
    pred2 = reader.get_predicate("person_at")
    assert pred2 is not None
    assert len(pred2.parameters) == 2
    assert pred2.parameters[0].name == "?person0"
    assert pred2.parameters[0].type == "person"
    assert pred2.parameters[1].name == "?room1"
    assert pred2.parameters[1].type == "room"

def test_get_functions():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_charging.pddl")

    functions = reader.get_functions()
    names = [f.name for f in functions]

    expected = ["speed", "max_range", "state_of_charge", "distance"]

    assert len(names) == len(expected)
    assert names == expected

def test_get_function_params():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_charging.pddl")

    # speed
    func = reader.get_function("speed")
    assert func is not None
    assert func.name == "speed"
    assert len(func.parameters) == 1
    assert func.parameters[0].name == "?robot0"
    assert func.parameters[0].type == "robot"

    # case-insensitive
    func_upper = reader.get_function("SPEED")
    assert func_upper is not None

    # distance
    func2 = reader.get_function("distance")
    assert func2 is not None
    assert len(func2.parameters) == 2
    assert func2.parameters[0].name == "?waypoint0"
    assert func2.parameters[0].type == "waypoint"
    assert func2.parameters[1].name == "?waypoint1"
    assert func2.parameters[1].type == "waypoint"

def test_get_actions():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")

    actions = reader.get_actions()
    assert len(actions) == 1
    assert actions[0] == "move_person"

    durative_actions = reader.get_durative_actions()
    assert len(durative_actions) == 3
    assert durative_actions == ["move", "talk", "approach"]

def test_get_action_params():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    loaded = reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")
    assert loaded, "No se pudo cargar el PDDL"

    # ------------------------------------------------------------------
    # 1) Acción inexistente
    # ------------------------------------------------------------------
    assert reader.get_action("noexist") is None

    # ------------------------------------------------------------------
    # 2) Acción durativa "move"
    # ------------------------------------------------------------------
    move = reader.get_durative_action("move")
    assert move is not None, "get_durative_action('move') devolvió None"
    assert move.name == "move"

    # ---- Parámetros (estructura y tipos) ----
    assert len(move.parameters) == 3

    # Nombres
    assert move.parameters[0].name == "?0"
    assert move.parameters[1].name == "?1"
    assert move.parameters[2].name == "?2"

    # Tipos (según TU dominio UPF: robot, waypoint, waypoint)
    assert move.parameters[0].type == "robot"
    assert move.parameters[1].type == "room"
    assert move.parameters[2].type == "room"

    # ------------------------------------------------------------------
    # 3) Preconditions (at_start_requirements)
    # ------------------------------------------------------------------
    at_start = move.at_start_requirements
    assert at_start is not None

    # Comprobaciones “semánticas” (no string exacto)
    at_start_str = str(at_start)
    assert "robot_at" in at_start_str
    assert "?0" in at_start_str
    assert "?1" in at_start_str

    # ------------------------------------------------------------------
    # 4) over_all_requirements y at_end_requirements
    # ------------------------------------------------------------------
    # En UPF pueden existir árboles vacíos o triviales,
    # así que solo comprobamos que son objetos válidos
    assert move.over_all_requirements is not None
    assert move.at_end_requirements is not None

    # (Opcional) aseguramos que son Trees de ROS
    from plansys2_msgs.msg import Tree
    assert isinstance(move.over_all_requirements, Tree)
    assert isinstance(move.at_end_requirements, Tree)

    # ------------------------------------------------------------------
    # 5) Efectos (at_start_effects y at_end_effects)
    # ------------------------------------------------------------------
    # Comprobaciones de contenido, no de formato exacto
    at_start_eff = move.at_start_effects
    at_end_eff = move.at_end_effects

    assert at_start_eff is not None
    assert at_end_eff is not None

    at_start_eff_str = str(at_start_eff)
    at_end_eff_str = str(at_end_eff)

    # En start effects debería haber una negación de robot_at(?0, ?1)
    assert "robot_at" in at_start_eff_str
    # Comprobar que hay un nodo NOT (node_type == 3) en los efectos de inicio
    has_not = any(node.node_type == 3 for node in move.at_start_effects.nodes)
    assert has_not


    # En end effects debería aparecer robot_at(?0, ?2)
    assert "robot_at" in at_end_eff_str
    assert "?2" in at_end_eff_str


def test_multidomain_get_types():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")

    reader.extend_domain(
        open(f"{pkgpath}/pddl/domain_simple_ext.pddl").read()
    )

    types = reader.get_types()
    expected_types = [
        "object", "person", "message", "robot",
        "room", "teleporter_room", "pickable_object"
    ]
    assert sorted(types) == sorted(expected_types)

    predicates = reader.get_predicates()
    pred_names = [p.name for p in predicates]
    expected_preds = [
        "object_at_robot", "object_at_room", "person_at",
        "robot_at", "robot_near_person", "robot_talk"
    ]
    assert sorted(pred_names) == sorted(expected_preds)

    actions = reader.get_actions()
    assert actions == ["move_person"]

    dactions = reader.get_durative_actions()
    expected_dactions = [
        "move", "talk", "approach", "pick_object", "place_object"
    ]
    assert dactions == expected_dactions

def test_sub_types():
    pkgpath = get_package_share_directory("plansys2_domain_expert")

    reader = DomainUPFReader(None)
    reader.load_pddl(f"{pkgpath}/pddl/domain_simple.pddl")

    # Durative action parameter subtypes
    move = reader.get_durative_action("move")
    assert move is not None
    assert len(move.parameters) == 3
    assert move.parameters[1].type == "room"
    assert move.parameters[1].sub_types == ["teleporter_room"]

    # Predicate parameter subtypes
    pred = reader.get_predicate("robot_at")
    assert pred is not None
    assert len(pred.parameters) == 2
    assert pred.parameters[1].type == "room"
    assert pred.parameters[1].sub_types == ["teleporter_room"]

    # Instant action parameter subtypes
    action = reader.get_action("move_person")
    assert action is not None
    assert len(action.parameters) == 3
    assert action.parameters[1].type == "room"
    assert action.parameters[1].sub_types == ["teleporter_room"]

    # Function parameter subtypes
    func = reader.get_function("teleportation_time")
    assert func is not None
    assert len(func.parameters) == 2

    assert func.parameters[0].type == "teleporter_room"
    assert func.parameters[0].sub_types == []

    assert func.parameters[1].type == "room"
    assert func.parameters[1].sub_types == ["teleporter_room"]


