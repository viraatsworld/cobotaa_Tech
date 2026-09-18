"""Wrist force/torque sensor for the cvrb0609 + OnRobot RG6.

Reads the 6-axis reaction wrench at the arm-flange -> gripper interface straight
from PhysX and hands it back as plain numpy vectors. Optionally mirrors it onto a
ROS 2 ``geometry_msgs/msg/WrenchStamped`` topic.

Where it sits in the articulation
--------------------------------
The gripper hangs off ``cobotta_pro_J6`` through the fixed
``onrobot_rg6/gripper_joint``. PhysX keeps a row for that fixed joint in its
link-incoming-joint-force table, so the RG6 ``base_link`` is the first solid
frame below the tool flange -- exactly where a real wrist FT sensor bolts in.
``get_measured_joint_forces()`` returns, per link, the total 6D force/torque its
incoming joint carries to balance everything distal to it; the ``base_link`` row
is therefore the flange <-> gripper wrench.

What the numbers mean
--------------------
This is the *raw* PhysX constraint wrench: noise-free, no bias drift, no
bandwidth limit. At rest the ``base_link`` row reads the tool weight
(~9.81 N in +Z for the 1.0 kg authored gripper) with near-zero torque, because
in the home pose the gripper CoM sits essentially on the J6 axis. Push or pull
on the fingers and all six axes respond.

Sign: the raw reading is the constraint force the wrist joint exerts *on the
tool* (it points +Z, holding the tool up against gravity). Pipelines that expect
"force the environment applies to the tool" want the opposite -- pass
``negate=True`` for that convention.

Frame: the wrench is expressed in the child link's joint frame, which is not
guaranteed to match the URDF/TF orientation of ``onrobot_rg6/base_link``. The Z
magnitude alone will not catch a rotated frame -- verify X/Y against a known
push before trusting them downstream.

See ``plan-ft-implement.md`` (sections 0, 1, 4) for the full rationale and the
baseline numbers this is regression-checked against.
"""

import numpy as np


class WristFTSensor:
    """Samples the flange <-> gripper reaction wrench from the articulation.

    Args:
        robot: the ``isaacsim.core.api.robots.Robot`` (SingleArticulation) wrapper
            for the arm+gripper. Must be added to the scene and ``world.reset()``
            already called so its physics handles exist.
        link_name: link whose incoming joint carries the wrench. ``base_link`` is
            the RG6 base = the wrist FT frame.
        negate: flip the sign convention (see module docstring).
    """

    def __init__(self, robot, link_name: str = "base_link", negate: bool = False):
        self._robot = robot
        self._link_name = link_name
        self._sign = -1.0 if negate else 1.0
        self._row = None          # resolved lazily, once physics is live
        self._warned = False
        self._last_error = None   # why the last read came back empty

        # ROS 2 (optional, wired by try_enable_ros2)
        self._ros_node = None
        self._ros_pub = None
        self._ros_msg = None
        self._frame_id = link_name

    # ------------------------------------------------------------------ reading

    def _resolve_row(self) -> bool:
        """Map link_name -> row index in the measured-joint-forces table.

        The table is link-indexed (one row per link's incoming joint, the fixed
        gripper joint included), so the link's body index is the row. Resolve by
        name -- the index depends on link ordering and must never be hardcoded.
        """
        view = self._robot._articulation_view
        try:
            idx = view.get_body_index(self._link_name)
        except (KeyError, TypeError):
            idx = None

        if idx is None:
            # Fall back to a suffix match against the full link list, so a
            # namespaced name ("onrobot_rg6/base_link") still resolves.
            # body_names is None until the view is initialized -- that is a
            # "not yet", not a "never", so keep retrying on later steps.
            names = list(getattr(view, "body_names", None) or [])
            matches = [i for i, n in enumerate(names)
                       if n == self._link_name or n.endswith("/" + self._link_name)]
            if len(matches) == 1:
                idx = matches[0]
            else:
                if not names:
                    self._last_error = "articulation view not initialized (no body names yet)"
                elif len(matches) > 1:
                    self._last_error = (f"link '{self._link_name}' is ambiguous, matches "
                                        f"{[names[i] for i in matches]}")
                else:
                    self._last_error = (f"link '{self._link_name}' not in the articulation; "
                                        f"links are {names}")
                if not self._warned and names:
                    print(f"[WristFTSensor] {self._last_error}; no wrench will be published")
                    self._warned = True
                return False        # never fall through to int(None)

        self._last_error = None
        self._row = int(idx)
        print(f"[WristFTSensor] wrist wrench frame = link '{self._link_name}' "
              f"(row {self._row}), sign {'-1 (env->tool)' if self._sign < 0 else '+1 (raw)'}")
        return True

    def read(self):
        """Return ``(force, torque)`` as numpy (3,) arrays, or ``(None, None)``.

        ``(None, None)`` means physics is not up yet -- call again next step.
        """
        if not self._robot.handles_initialized:
            self._last_error = "articulation handles not initialized (physics not live yet)"
            return None, None
        if self._row is None and not self._resolve_row():
            return None, None

        try:
            wrench = self._robot.get_measured_joint_forces(joint_indices=[self._row])
        except Exception as exc:                # handles vanished mid-reset, etc.
            self._last_error = f"get_measured_joint_forces raised {exc.__class__.__name__}: {exc}"
            return None, None
        if wrench is None:
            # Isaac logs the detail as a carb warning and hands back None: either
            # the articulation view is not initialized or the physics simulation
            # view does not exist yet.
            self._last_error = "get_measured_joint_forces returned None (physics sim view not ready)"
            return None, None

        self._last_error = None
        wrench = np.asarray(wrench).reshape(-1)  # (6,) -> fx fy fz tx ty tz
        return self._sign * wrench[:3].copy(), self._sign * wrench[3:].copy()

    def status(self) -> str:
        """Why the last ``read()`` produced nothing. Empty string when healthy."""
        return self._last_error or ""

    # ------------------------------------------------------------------- ROS 2

    def try_enable_ros2(self, topic: str = "/wrist_ft", frame_id: str | None = None) -> bool:
        """Best-effort: stand up a WrenchStamped publisher. Returns success.

        Kept optional and non-fatal -- if rclpy/geometry_msgs are not importable
        in this interpreter the sensor still reads, it just publishes nothing.
        """
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import WrenchStamped
        except Exception as exc:
            print(f"[WristFTSensor] ROS 2 publish disabled ({exc.__class__.__name__}: {exc})")
            return False

        if frame_id is not None:
            self._frame_id = frame_id

        if not rclpy.ok():
            rclpy.init(args=None)
        self._ros_node = Node("wrist_ft_sensor")
        self._ros_pub = self._ros_node.create_publisher(WrenchStamped, topic, 10)
        self._ros_msg = WrenchStamped()
        self._ros_msg.header.frame_id = self._frame_id
        print(f"[WristFTSensor] publishing geometry_msgs/WrenchStamped on '{topic}' "
              f"(frame_id '{self._frame_id}')")
        return True

    def publish(self, force, torque, sim_time: float | None = None) -> None:
        """Publish one WrenchStamped. No-op if ROS 2 was not enabled.

        Pass ``sim_time`` (e.g. ``world.current_time``) to stamp with simulation
        time so the wrench lines up with ``/clock`` and the joint states; without
        it the stamp is wall-clock node time.
        """
        if self._ros_pub is None:
            return
        m = self._ros_msg
        if sim_time is None:
            m.header.stamp = self._ros_node.get_clock().now().to_msg()
        else:
            m.header.stamp.sec = int(sim_time)
            m.header.stamp.nanosec = int((sim_time - int(sim_time)) * 1e9)
        m.wrench.force.x, m.wrench.force.y, m.wrench.force.z = map(float, force)
        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z = map(float, torque)
        self._ros_pub.publish(m)

    def shutdown(self) -> None:
        if self._ros_node is not None:
            self._ros_node.destroy_node()
            self._ros_node = None
