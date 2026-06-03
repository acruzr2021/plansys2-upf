# Integration of Unified Planning Framework in PlanSys2

[![Stars](https://img.shields.io/github/stars/acruzr2021/plansys2-upf?style=flat-square)](https://github.com/acruzr2021/plansys2-upf/stargazers)
[![Forks](https://img.shields.io/github/forks/acruzr2021/plansys2-upf?style=flat-square)](https://github.com/acruzr2021/plansys2-upf/network/members)

A ROS 2 Python package that replaces the PlanSys2 C++ nodes (Domain Expert, Problem Expert and Planner) with an implementation backed by the [Unified Planning Framework (UPF)](https://github.com/aiplan4eu/unified-planning). It exposes the same ROS 2 services and topics as the original PlanSys2 nodes, so it can be used as a drop-in replacement without modifying any other component of the system.

## Overview

The package provides three ROS 2 lifecycle nodes:

| Node | Executable | Responsibility |
|------|-----------|----------------|
| `domain_expert` | `domain_upf_node` | Loads and exposes the PDDL domain (types, predicates, functions, actions) |
| `problem_expert` | `problem_upf_node` | Manages the planning problem at runtime (instances, predicates, functions, goal) |
| `planner` | `planner_upf_node` | Receives planning requests and returns a plan using UPF solvers |

## Installation

### Prerequisites

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) — tested on Jazzy Jalisco (Ubuntu 24.04)
- [PlanSys2](https://github.com/PlanSys2/ros2_planning_system) — built from source in the same workspace
- [pixi](https://pixi.sh) — used to manage the Python environment

### 1. Install pixi

If you do not have pixi installed yet:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

Then restart your terminal or run `source ~/.bashrc` for the `pixi` command to be available.

### 2. Set up your ROS 2 workspace

Create a workspace if you do not have one already and clone both PlanSys2 and this repository into it:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PlanSys2/ros2_planning_system.git
git clone https://github.com/acruzr2021/plansys2-upf.git
```

### 3. Install ROS 2 dependencies

From the workspace root, use `rosdep` to install any missing ROS 2 dependencies declared in the `package.xml` files:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Set up the Python environment with pixi

The Python dependencies (`unified-planning`, `pyperplan`, OpenJDK) are managed with pixi. Go into the package directory and run:

```bash
cd ~/ros2_ws/src/plansys2-upf
pixi install
```

This creates an isolated environment under `.pixi/` with Python 3.12 and all required packages. You only need to run this once (or again if `pixi.toml` changes).

### 5. Build the workspace

The build must run inside the pixi environment so that colcon finds `unified-planning` in the active Python. Activate the environment first, then build:

```bash
cd ~/ros2_ws/src/plansys2-upf
pixi shell
```

You are now inside the pixi environment. From here, source ROS 2 and build:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select plansys2_upf
```

If you also need to build PlanSys2 from source, omit `--packages-select` to build the full workspace:

```bash
colcon build
```

### 6. Source the workspace

After a successful build, source the install space to make the package available:

```bash
source ~/ros2_ws/install/setup.bash
```

> **Tip:** add both `source /opt/ros/jazzy/setup.bash` and `source ~/ros2_ws/install/setup.bash` to your `~/.bashrc` so you do not need to run them every time. You will still need to activate the pixi environment (`pixi shell`) in any terminal where you run or build this package.

## Usage

### Launch all nodes

```bash
ros2 launch plansys2_upf launch.py model_file:=<path/to/domain.pddl>
```

With an optional initial problem file:

```bash
ros2 launch plansys2_upf launch.py \
  model_file:=<path/to/domain.pddl> \
  problem_file:=<path/to/problem.pddl>
```

Multiple domain files can be merged by separating their paths with `:`:

```bash
ros2 launch plansys2_upf launch.py \
  model_file:=<domain1.pddl>:<domain2.pddl>
```

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `model_file` | — | Path(s) to the PDDL domain file(s), separated by `:` |
| `problem_file` | `""` | Path to an optional initial PDDL problem file |
| `namespace` | `""` | ROS 2 namespace for all nodes |

## Configuration

The planner is configured via `config/planner.yaml`:

```yaml
planner:
  ros__parameters:
    plan_solver_timeout: 30.0
    plan_solver_plugins: ["UPF"]
    UPF:
      plugin: "plansys2_upf.upf_planner_solver.upf_plan_solver.UPFPlanSolver"
      preferred_planner: "default"
```

| Parameter | Description |
|-----------|-------------|
| `plan_solver_timeout` | Maximum planning time in seconds |
| `plan_solver_plugins` | List of solver plugin identifiers to load |
| `UPF.plugin` | Full Python class path of the solver |
| `UPF.preferred_planner` | Planner to use: `"default"` (auto-select best) or a specific name such as `"popf"`, `"tamer"`, `"fast-downward"` |

When `preferred_planner` is set to `"default"`, the solver tries all planners available in the UPF environment that support the problem type and returns the plan with the shortest makespan.

## Running the tests

Activate the pixi environment and source both ROS 2 and the workspace before running the tests:

```bash
cd ~/ros2_ws/src/plansys2-upf
pixi shell
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
colcon test --packages-select plansys2_upf
colcon test-result --verbose
```

## Implemented features

- [x] **Domain Expert**
  - [x] `domain_expert/get_domain_name`
  - [x] `domain_expert/get_domain_types`
  - [x] `domain_expert/get_domain_actions`
  - [x] `domain_expert/get_domain_action_details`
  - [x] `domain_expert/get_domain_durative_actions`
  - [x] `domain_expert/get_domain_durative_action_details`
  - [x] `domain_expert/get_domain_predicates`
  - [x] `domain_expert/get_domain_predicate_details`
  - [x] `domain_expert/get_domain_functions`
  - [x] `domain_expert/get_domain_function_details`
  - [x] `domain_expert/get_domain`
  - [ ] `domain_expert/get_domain_derived_predicates`
  - [ ] `domain_expert/get_domain_derived_predicate_details`
  - [x] Tests

- [x] **Problem Expert**
  - [x] `problem_expert/add_problem`
  - [x] `problem_expert/add_problem_goal`
  - [x] `problem_expert/add_problem_instance`
  - [x] `problem_expert/add_problem_predicate`
  - [x] `problem_expert/add_problem_function`
  - [x] `problem_expert/get_problem_goal`
  - [x] `problem_expert/get_problem_instance`
  - [x] `problem_expert/get_problem_instances`
  - [x] `problem_expert/get_problem_predicate`
  - [x] `problem_expert/get_problem_predicates`
  - [x] `problem_expert/get_problem_function`
  - [x] `problem_expert/get_problem_functions`
  - [x] `problem_expert/get_problem`
  - [x] `problem_expert/is_problem_goal_satisfied`
  - [x] `problem_expert/remove_problem_goal`
  - [x] `problem_expert/clear_problem_knowledge`
  - [x] `problem_expert/remove_problem_instance`
  - [x] `problem_expert/remove_problem_predicate`
  - [x] `problem_expert/remove_problem_function`
  - [x] `problem_expert/exist_problem_predicate`
  - [x] `problem_expert/exist_problem_function`
  - [x] `problem_expert/update_problem_function`
  - [x] `problem_expert/knowledge` (publisher)
  - [x] Tests

- [x] **Planner**
  - [x] `planner/get_plan`
  - [x] `planner/validate_domain`
  - [x] Plugin-based solver architecture
  - [x] POPF engine adapter
  - [x] OPTIC engine adapter
  - [x] Tests
