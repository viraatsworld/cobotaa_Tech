# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Pick-to-bin's MDP logic, without the simulator.

The terms read the scene through :mod:`...pick_to_bin.mdp.state`; these tests
replace those reads with scripted values, so what is checked is the
bookkeeping: angles, the arm action's limits, progress shaping, one-off
bonuses, the gripper rule's state machine and the terminations.
"""

from __future__ import annotations

import importlib
import math
from types import SimpleNamespace

import pytest
import torch

from isaaclab.managers import RewardTermCfg, TerminationTermCfg

from techtory_cobotta_isaaclab.robot.robot_cfg import GRIPPER_CLOSED, GRIPPER_OPEN
from techtory_cobotta_isaaclab.robot.top_down import (
    rate_limit,
    smooth_step,
    step_target,
    top_down_quat,
    wrap_half_turn,
    x_axis_yaw,
)

# By module path: `from ...mdp import actions` would hand back Isaac Lab's own
# mdp.actions, because the task's mdp package forwards unknown names to it.
_MDP = "techtory_cobotta_isaaclab.tasks.pick_to_bin.mdp"
actions = importlib.import_module(f"{_MDP}.actions")
rewards = importlib.import_module(f"{_MDP}.rewards")
state = importlib.import_module(f"{_MDP}.state")
terminations = importlib.import_module(f"{_MDP}.terminations")

pytestmark = pytest.mark.unit

DT = 0.04  # policy step of the task: 0.01 s x decimation 4


def _env(n: int = 1, **extra) -> SimpleNamespace:
    return SimpleNamespace(num_envs=n, device="cpu", step_dt=DT, physics_dt=0.01, extras={}, **extra)


def _feed(monkeypatch: pytest.MonkeyPatch, **values) -> dict[str, list]:
    """Make ``state.<name>(env)`` return the next scripted value each time it is called."""
    queues = {name: list(seq) for name, seq in values.items()}
    for name, queue in queues.items():
        monkeypatch.setattr(state, name, lambda env, _q=queue: torch.as_tensor([_q.pop(0)]))
    return queues


##
# Angles and the arm action's limits.
##


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(0.0, 0.0), (math.pi, 0.0), (-math.pi, 0.0), (0.3, 0.3), (math.pi - 0.3, -0.3), (2 * math.pi + 0.2, 0.2)],
)
def test_yaw_error_is_modulo_a_half_turn(angle: float, expected: float) -> None:
    """The jaw and the handle look the same turned by 180 deg: 0 and pi are both a perfect grasp."""
    assert wrap_half_turn(torch.tensor([angle])).item() == pytest.approx(expected, abs=1e-6)


def test_round_objects_have_no_yaw_to_get_right() -> None:
    """The soda can has no grasp axis: any jaw yaw grasps it, without reading the scene."""
    env = _env(2, cfg=SimpleNamespace(grasp_object="soda_can"))
    assert state.grasp_yaw_error(env).tolist() == [0.0, 0.0]
    assert state.object_yaw(env).tolist() == [0.0, 0.0]


@pytest.mark.parametrize("yaw", [-3.0, -1.2, 0.0, 0.7, 2.9])
def test_top_down_orientation(yaw: float) -> None:
    quat = top_down_quat(torch.tensor([yaw]))
    assert x_axis_yaw(quat).item() == pytest.approx(yaw, abs=1e-5)
    # the approach axis (TCP +z) points straight down
    from isaaclab.utils.math import quat_apply

    approach = quat_apply(quat, torch.tensor([[0.0, 0.0, 1.0]]))
    assert approach.tolist()[0] == pytest.approx([0.0, 0.0, -1.0], abs=1e-6)


def test_target_leads_the_tcp_by_a_bounded_amount() -> None:
    tcp = torch.zeros(1, 3)
    target = torch.tensor([[0.025, 0.0, 0.0]])
    delta = torch.tensor([[0.005, 0.0, 0.0, 0.0]])
    for _ in range(10):  # the arm is blocked: the TCP does not move
        target, _ = step_target(target, torch.zeros(1), tcp, torch.zeros(1), delta, 0.03, 0.1)
    assert target[0, 0].item() == pytest.approx(0.03)


def test_yaw_target_wraps_and_is_bounded() -> None:
    _, yaw = step_target(
        torch.zeros(1, 3), torch.tensor([3.1]), torch.zeros(1, 3), torch.tensor([3.1]),
        torch.tensor([[0.0, 0.0, 0.0, 0.5]]), 0.03, 0.1,
    )  # fmt: skip
    assert yaw.item() == pytest.approx(3.2 - 2 * math.pi, abs=1e-6)


def test_workspace_bounds_the_target() -> None:
    workspace = (torch.tensor([-0.2, -0.7, 0.005]), torch.tensor([0.8, 0.27, 0.45]))
    tcp = torch.tensor([[0.79, 0.26, 0.01]])
    delta = torch.tensor([[0.005, 0.005, -0.005, 0.0]])
    target, _ = step_target(tcp, torch.zeros(1), tcp, torch.zeros(1), delta, 0.03, 0.1, workspace)
    assert target.tolist()[0] == pytest.approx([0.795, 0.265, 0.005])  # z already at the floor
    target, _ = step_target(target, torch.zeros(1), tcp, torch.zeros(1), delta * 4, 0.03, 0.1, workspace)
    assert target.tolist()[0] == pytest.approx([0.8, 0.27, 0.005])  # x, y stopped at the walls


def test_joint_targets_never_outrun_the_velocity_limit() -> None:
    """Whatever the IK asks for -- a 1 rad jump near a singularity, say -- the target moves at most v * dt."""
    max_step = torch.tensor([0.0040, 0.0033, 0.0040, 0.0050, 0.0050, 0.0060])
    current = torch.zeros(6)
    out = rate_limit(current, torch.tensor([1.0, -1.0, 0.002, -0.002, 0.5, -0.5]), max_step)
    assert out.tolist() == pytest.approx([0.0040, -0.0033, 0.002, -0.002, 0.0050, -0.0060])


def _drive(goal: float, ticks: int = 400, v: float = 0.4, a: float = 1.0, dt: float = 0.01):
    """Move a joint target from 0 towards ``goal`` with :func:`smooth_step`; return positions and steps."""
    max_step, max_change = torch.tensor([v * dt]), torch.tensor([a * dt * dt])
    pos, step = torch.zeros(1), torch.zeros(1)
    positions, steps = [], []
    for _ in range(ticks):
        step = smooth_step(step, torch.tensor([goal]) - pos, max_step, max_change)
        pos = pos + step
        positions.append(pos.item())
        steps.append(step.item())
    return positions, steps


def test_joint_targets_respect_speed_and_acceleration() -> None:
    v, a, dt = 0.4, 1.0, 0.01
    _, steps = _drive(1.0)
    assert max(abs(s) for s in steps) <= v * dt + 1e-9
    changes = [abs(b - c) for b, c in zip([0.0, *steps], steps, strict=False)]
    assert max(changes) <= a * dt * dt + 1e-9
    # From rest it takes v / a = 0.4 s to reach full speed, not one tick
    assert steps[0] == pytest.approx(a * dt * dt)
    assert steps[39] == pytest.approx(v * dt, rel=1e-3)


@pytest.mark.parametrize("goal", [1.0, -0.3, 0.02])
def test_joint_targets_arrive_without_overshoot(goal: float) -> None:
    positions, steps = _drive(goal)
    # Never past the goal by more than rounding: one tick's acceleration, 1e-4 rad (0.1 mm at 1 m);
    # the naive sqrt(2 a e) bound overshot 2e-3 rad.
    assert max(abs(p) for p in positions) <= abs(goal) + 1.0 * 0.01 * 0.01
    assert positions[-1] == pytest.approx(goal, abs=1e-5)
    assert abs(steps[-1]) < 1e-6  # and stopped


##
# Rewards.
##


def _reward(term_cls, env, **params):
    return term_cls(RewardTermCfg(func=term_cls, params=params), env)


def test_progress_telescopes_and_starts_at_zero(monkeypatch: pytest.MonkeyPatch) -> None:
    distances = [0.5, 0.4, 0.45, 0.2]
    _feed(
        monkeypatch,
        grasp_distance=distances,
        grasp_yaw_error=[0.0] * 4,
        object_grasped=[False] * 4,
    )
    env = _env()
    term = _reward(rewards.ApproachProgress, env, yaw_weight=0.1)
    paid = [term(env, yaw_weight=0.1).item() * DT for _ in distances]
    assert paid == pytest.approx([0.0, 0.1, -0.05, 0.25])
    assert sum(paid) == pytest.approx(distances[0] - distances[-1])


def test_progress_restarts_after_reset(monkeypatch: pytest.MonkeyPatch) -> None:
    _feed(monkeypatch, grasp_distance=[0.5, 0.4, 0.9], grasp_yaw_error=[0.0] * 3, object_grasped=[False] * 3)
    env = _env()
    term = _reward(rewards.ApproachProgress, env)
    term(env), term(env)
    term.reset([0])
    assert term(env).item() == 0.0  # a reset is not a jump in distance


def test_approach_counts_yaw(monkeypatch: pytest.MonkeyPatch) -> None:
    """Turning the jaw onto the handle pays, as moving closer does: 0.1 m per rad."""
    _feed(monkeypatch, grasp_distance=[0.1, 0.1], grasp_yaw_error=[1.0, 0.5], object_grasped=[False] * 2)
    env = _env()
    term = _reward(rewards.ApproachProgress, env, yaw_weight=0.1)
    term(env, yaw_weight=0.1)
    assert term(env, yaw_weight=0.1).item() * DT == pytest.approx(0.05)


def test_transport_pays_only_while_carried(monkeypatch: pytest.MonkeyPatch) -> None:
    grasped = [False, True, True, False, True]
    _feed(monkeypatch, bin_target_distance=[0.8, 0.7, 0.6, 0.4, 0.3], object_grasped=grasped)
    env = _env()
    term = _reward(rewards.TransportProgress, env)
    paid = [term(env).item() * DT for _ in grasped]
    # step 1: grasped now but not before; step 3: dropped; step 4: re-grasped this step
    assert paid == pytest.approx([0.0, 0.0, 0.1, 0.0, 0.0])


def test_milestone_pays_once_per_episode(monkeypatch: pytest.MonkeyPatch) -> None:
    _feed(monkeypatch, object_picked=[False, True, True, False, True, True])
    env = _env()
    term = _reward(rewards.PickedObject, env)
    paid = [term(env).item() for _ in range(4)]
    assert paid == [0.0, 1.0 / DT, 0.0, 0.0]  # grasp -> drop -> re-grasp cannot farm it
    term.reset([0])
    assert [term(env).item() for _ in range(2)] == [1.0 / DT, 0.0]


def test_weights_are_the_reward() -> None:
    """The reward manager multiplies by dt; the terms divide by it, so +5 means +5."""
    weight = 5.0
    assert (1.0 / DT) * weight * DT == pytest.approx(weight)
    assert rewards.per_step(_env(3)).tolist() == pytest.approx([1.0 / DT] * 3)


def test_lost_in_transport_comes_from_the_rule(monkeypatch: pytest.MonkeyPatch) -> None:
    rule = SimpleNamespace(lost_this_step=torch.tensor([True, True, False]))
    monkeypatch.setattr(state, "gripper_rule", lambda env: rule)
    monkeypatch.setattr(state, "object_near_bin", lambda env: torch.tensor([False, True, False]))
    # lost far from the bin: penalised; at the bin: that is the release; not lost: nothing
    assert (rewards.lost_in_transport(_env(3)) * DT).tolist() == pytest.approx([1.0, 0.0, 0.0])


##
# Terminations.
##


def _termination(term_cls, env):
    return term_cls(TerminationTermCfg(func=term_cls), env)


def _scene(monkeypatch: pytest.MonkeyPatch, rule, *, in_bin: bool, speed: float = 0.0, lowest: float = 1.0):
    monkeypatch.setattr(state, "gripper_rule", lambda env: rule)
    monkeypatch.setattr(state, "object_in_bin", lambda env: torch.tensor([in_bin]))
    monkeypatch.setattr(state, "object_speed", lambda env: torch.tensor([speed]))
    monkeypatch.setattr(state, "object_corners", lambda env: torch.full((1, 8, 3), lowest))


def test_holding_still_outside_the_bin_is_not_a_drop(monkeypatch: pytest.MonkeyPatch) -> None:
    """A pause mid-carry or a squeeze on the table: still and outside the bin, but still in the jaw."""
    rule = SimpleNamespace(closed=torch.tensor([True]), picked_ever=torch.tensor([True]))
    _scene(monkeypatch, rule, in_bin=False)
    env = _env()
    term = _termination(terminations.DroppedOutsideBin, env)
    assert not any(term(env, hold_steps=5).item() for _ in range(20))
    rule.closed = torch.tensor([False])  # let go
    fired = [term(env, hold_steps=5).item() for _ in range(5)]
    assert fired == [False, False, False, False, True]


def test_falling_off_the_table_ends_the_episode(monkeypatch: pytest.MonkeyPatch) -> None:
    rule = SimpleNamespace(closed=torch.tensor([False]), picked_ever=torch.tensor([False]))
    _scene(monkeypatch, rule, in_bin=False, speed=1.0, lowest=0.5)
    env = _env()
    assert _termination(terminations.DroppedOutsideBin, env)(env).item()


def test_success_needs_the_object_to_settle_and_is_logged(monkeypatch: pytest.MonkeyPatch) -> None:
    rule = SimpleNamespace(closed=torch.tensor([False]), picked_ever=torch.tensor([True]))
    _scene(monkeypatch, rule, in_bin=True)
    env = _env()
    term = _termination(terminations.ObjectResting, env)
    fired = [term(env, hold_steps=3).item() for _ in range(3)]
    assert fired == [False, False, True]
    term.reset([0])
    assert env.extras["log"]["Metrics/success_rate"] == 1.0


##
# The gripper rule.
##


class _Robot:
    def __init__(self) -> None:
        self.target: float | None = None

    def find_joints(self, names):
        return [0], list(names)

    def set_joint_position_target_index(self, target: torch.Tensor, joint_ids) -> None:
        self.target = round(target[0, 0].item(), 6)  # float32 -> the robot_cfg constants


def _rule(monkeypatch: pytest.MonkeyPatch, **scene):
    """A gripper rule on a fake robot; ``scene`` sets what each state read returns (mutable later)."""
    robot = _Robot()
    env = _env(scene={"robot": robot})
    rule = actions.ProximityGripperAction(actions.ProximityGripperActionCfg(), env)
    defaults = dict(
        grasp_distance=0.1, at_grasp_pose=False, object_grasped=False, jaw_closed_on_nothing=False,
        object_lift=0.0, object_over_bin=False,
    )  # fmt: skip
    values = {**defaults, **scene}
    for name in defaults:
        monkeypatch.setattr(state, name, lambda env, _n=name: torch.tensor([values[_n]]))
    return rule, robot, values


def test_rule_closes_at_the_handle_and_reopens_after_a_miss(monkeypatch: pytest.MonkeyPatch) -> None:
    rule, robot, scene = _rule(monkeypatch)
    rule.apply_actions()
    assert robot.target == GRIPPER_OPEN
    scene.update(at_grasp_pose=True, grasp_distance=0.005)
    rule.apply_actions()
    assert robot.target == GRIPPER_CLOSED and rule.closed_ever.item()
    scene.update(jaw_closed_on_nothing=True)
    rule.apply_actions()
    assert robot.target == GRIPPER_OPEN  # a miss
    scene.update(jaw_closed_on_nothing=False)
    rule.apply_actions()
    assert robot.target == GRIPPER_OPEN  # not re-armed: the TCP has not backed off
    scene.update(grasp_distance=0.05, at_grasp_pose=False)
    rule.apply_actions()
    scene.update(grasp_distance=0.005, at_grasp_pose=True)
    rule.apply_actions()
    assert robot.target == GRIPPER_CLOSED  # backed off 5 cm, came back: closes again


def test_rule_gives_up_on_a_jaw_stalled_on_something_else(monkeypatch: pytest.MonkeyPatch) -> None:
    rule, robot, _ = _rule(monkeypatch, at_grasp_pose=True, grasp_distance=0.005)
    steps = round(rule.cfg.close_timeout / 0.01)  # physics steps of 0.01 s
    for _ in range(steps - 5):
        rule.apply_actions()
    assert robot.target == GRIPPER_CLOSED  # still giving the jaw time to close
    for _ in range(10):
        rule.apply_actions()
    assert robot.target == GRIPPER_OPEN


def test_rule_releases_over_the_bin_for_good_and_reports_losses(monkeypatch: pytest.MonkeyPatch) -> None:
    rule, robot, scene = _rule(monkeypatch, at_grasp_pose=True, grasp_distance=0.005)
    rule.apply_actions()
    scene.update(object_grasped=True, object_lift=0.05, at_grasp_pose=False)
    rule.apply_actions()
    assert rule.picked_ever.item()

    # slips out on the way: the jaw closes on nothing -> reported once for this policy step
    rule.process_actions(torch.zeros(1, 0))
    scene.update(object_grasped=False, jaw_closed_on_nothing=True)
    rule.apply_actions()
    assert rule.lost_this_step.item()
    rule.process_actions(torch.zeros(1, 0))
    assert not rule.lost_this_step.item()

    # picked up again and carried over the bin: released, and stays open
    scene.update(jaw_closed_on_nothing=False, grasp_distance=0.05)
    rule.apply_actions()
    scene.update(grasp_distance=0.005, at_grasp_pose=True)
    rule.apply_actions()
    scene.update(object_grasped=True, object_over_bin=True)
    rule.apply_actions()
    assert rule.released.item() and robot.target == GRIPPER_OPEN
    rule.apply_actions()
    assert robot.target == GRIPPER_OPEN
    rule.reset([0])
    assert not (rule.released.item() or rule.picked_ever.item() or rule.closed_ever.item())
