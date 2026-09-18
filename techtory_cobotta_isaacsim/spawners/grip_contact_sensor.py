"""Grip-load sensor -- the extra wrench the grasped object puts on the wrist.

Why a dynamics observer and not a contact read
----------------------------------------------
``WristFTSensor`` reads ``get_measured_joint_forces()`` at the flange->gripper
fixed joint. That row carries only the articulation's *internal* reaction
(gravity on the gripper links + finger drives); a grasped payload reaches the
gripper through pad<->object contact, which PhysX does not fold into that table.
Pick up a cube and the wrist wrench does not move.

Reading the contact directly does not work on this gripper either (verified):
``get_net_contact_forces()`` on an *articulation link* returns zero, and the RG6
pad colliders (``PhysxMeshMergeCollisionAPI``) emit no contact reports at all.

So this sensor does not touch contacts. It applies Newton's second law to the
grasped **object**:

    F_ext_on_object = m * a
    F_ext_on_object = F_from_gripper + m * g        (g = (0,0,-9.81))
    => F_from_gripper = m * (a - g)

The reaction -- the force the object applies back on the gripper, i.e. the load
the wrist feels -- is ``m * (g - a)``. Held still: ``m*g`` pulling the tool down
(the payload weight). Accelerated during a move: the ``m*a`` term adds the
inertial load. Squeeze force is internal to the jaw and cancels through the
object, so it correctly does not appear.

Which object to track
---------------------
``auto_discover=True`` (default): every ``scan_every`` reads the sensor looks
for dynamic rigid bodies whose origin is within ``discover_radius`` of the jaw
centre (midpoint of the two inner-finger links). A body that stays in the jaw
for ``discover_persistence`` scans is tracked; one that leaves for that many
scans is dropped. No prim path to configure -- spawn a cube in the GUI, grasp
it, and it is picked up.

``object_paths`` still works and is always tracked (never dropped by discovery);
use it to force-track something, or set ``auto_discover=False`` to use only it.

Output
------
``geometry_msgs/WrenchStamped`` at the ``base_link`` (wrist) frame -- the same
frame as ``/wrist_ft``, so the two add:

    wrench at wrist  ~=  /wrist_ft      (gravity + drive, articulation-internal)
                      +  /grip_contact  (payload weight + inertia)

``negate=False`` (default) gives "force the environment applies to the tool" --
the opposite default sense to ``WristFTSensor.negate``.

Stage hygiene (why views are cached, never rebuilt per scan)
------------------------------------------------------------
Constructing an ``XFormPrim``/``RigidPrim`` is *not* a read-only operation:

* with the default ``reset_xform_properties=True`` it calls ``ClearXformOpOrder``,
  rewrites ``xformOp:translate/orient/scale`` and then teleports the prim with
  ``set_world_poses`` -- USD authoring on a live rigid body, which makes PhysX
  resync that actor;
* it stamps a fresh ``isaac_sim:view_index:<hash(view)>`` attribute into fabric,
  one per view instance;
* ``RigidPrim.__init__`` creates a PhysX rigid-body view off the *current*
  simulation view and calls ``get_linear_velocities()`` on it.

A PhysX resync invalidates ``SimulationManager._physics_sim_view`` in place, and
nothing on the Python side is notified (``PHYSICS_READY`` only fires on play, and
``is_physics_handle_valid()`` only checks for ``None``). Every view built before
that point keeps a dangling handle and the next read logs

    [omni.physx.tensors.plugin] Simulation view object is invalidated
    and cannot be used again to call getVelocities

So: one view per prim, built once and cached, always with
``reset_xform_properties=False``; discovery probes are pose-only ``XFormPrim``s
(no rigid-body view, no velocity read). ``_rebind_if_stale`` additionally
re-initializes the views if the simulation view object is ever swapped out.

Requirements / limits
---------------------
* Uses the object's authored mass (``RigidPrim.get_masses``). If that is wrong,
  the load is wrong by the same factor.
* Always attributes ``m*(g-a)`` to the gripper -- it cannot tell that a surface
  is sharing the load. An untracked object left sitting in the open jaw is
  picked up too; close the jaw or move it out.
* Ignores rotational inertia of the payload (no ``I*alpha`` term).
* Gravity is taken as ``(0, 0, -9.81)`` -- change ``GRAVITY`` if the scene
  differs.
"""

import numpy as np

GRAVITY = np.array([0.0, 0.0, -9.81])
A_MAX = 20.0        # m/s^2, per-axis clamp on the estimated payload acceleration
WARMUP_READS = 5    # skip the inertial term for the first few reads of a view


def _to_np(a):
    try:
        return np.asarray(a, dtype=float)
    except Exception:
        return np.asarray(a.detach().cpu().numpy(), dtype=float)


def _quat_rotate_inverse(q, v):
    """World-frame ``v`` expressed in the local frame of quaternion ``q`` (w,x,y,z)."""
    w, x, y, z = (float(c) for c in q)
    u = np.array([x, y, z])
    uv = np.cross(u, v)
    return v * (w * w - u.dot(u)) - 2.0 * w * uv + 2.0 * u * (u.dot(v))


class GripContactSensor:
    """Extra wrench on the wrist from the grasped object, via a payload observer.

    Args:
        world: the ``isaacsim.core.api.World`` (for the physics ``dt``).
        object_paths: prim path(s) to always track (may be empty when
            ``auto_discover`` is on). One ``RigidPrim`` view per entry.
        wrist_prim_path: link whose frame the wrench is reported in. Keep equal
            to ``WristFTSensor``'s frame so the two wrenches add.
        auto_discover: track whatever dynamic rigid body sits in the jaw.
        jaw_prim_paths: the two links whose midpoint is the jaw centre.
        discover_radius: max distance (m) from the jaw centre to count as "in
            the jaw".
        discover_root / exclude_prefixes: subtree scanned for rigid bodies, and
            path prefixes ignored (the robot itself, the cell, ...).
        scan_every / discover_persistence: scan cadence (reads) and how many
            consecutive scans a body must be in / out of the jaw before it is
            added / dropped.
        negate: flip the sign. Default False = "force the environment applies to
            the tool" (opposite default to ``WristFTSensor.negate``).
        ema: acceleration smoothing in [0, 1]. 0 = weight only, no inertial term.
    """

    def __init__(self, world,
                 object_paths=None,
                 wrist_prim_path: str = "/World/Cobotta/onrobot_rg6/base_link",
                 auto_discover: bool = True,
                 jaw_prim_paths=("/World/Cobotta/onrobot_rg6/left_inner_finger",
                                 "/World/Cobotta/onrobot_rg6/right_inner_finger"),
                 discover_radius: float = 0.10,
                 discover_root: str = "/World",
                 exclude_prefixes=("/World/Cobotta",),
                 scan_every: int = 15,
                 discover_persistence: int = 3,
                 negate: bool = False,
                 ema: float = 0.25):
        self._world = world
        if isinstance(object_paths, str):
            object_paths = [object_paths]
        self._explicit = [p for p in (object_paths or []) if p]
        self._wrist_path = wrist_prim_path
        self._auto = bool(auto_discover)
        self._jaw_paths = list(jaw_prim_paths)
        self._radius = float(discover_radius)
        self._discover_root = discover_root
        self._exclude = tuple(exclude_prefixes)
        self._scan_every = max(1, int(scan_every))
        self._persist = max(1, int(discover_persistence))
        self._sign = -1.0 if negate else 1.0
        self._ema = float(np.clip(ema, 0.0, 1.0))

        self._obj_views = {}        # {path: RigidPrim}  (explicit + discovered)
        self._probe_views = {}      # {path: XFormPrim}  pose-only, discovery scans
        self._wrist_view = None
        self._jaw_views = []        # XFormPrim per jaw link
        self._v_prev = {}
        self._a_filt = {}
        self._reads = {}
        self._in_streak = {}        # {path: consecutive scans seen in the jaw}
        self._out_streak = {}       # {path: consecutive scans seen out}
        self._read_count = 0
        self._RigidPrim = None
        self._XFormPrim = None
        self._SimulationManager = None
        self._sim_view = None       # the SimulationView our views were built against
        self._stage = None
        self._disabled = False
        self._warned = False
        self._last_error = None     # why the last read came back empty

        # ROS 2 (optional)
        self._ros_node = None
        self._ros_pub = None
        self._ros_msg = None
        self._frame_id = wrist_prim_path.rsplit("/", 1)[-1]

    # ------------------------------------------------------------------ build

    def prepare(self) -> bool:
        """Import the API and build the fixed views. Retryable / idempotent."""
        if self._disabled:
            return False
        if not self._auto and not self._explicit:
            print("[GripContactSensor] no object_paths and auto_discover off; disabled")
            self._disabled = True
            return False

        if self._RigidPrim is None:
            try:
                from isaacsim.core.prims import RigidPrim, XFormPrim
                from isaacsim.core.simulation_manager import SimulationManager
                from isaacsim.core.utils.stage import get_current_stage
            except Exception as exc:             # pragma: no cover - env dependent
                print(f"[GripContactSensor] isaacsim core APIs unavailable "
                      f"({exc.__class__.__name__}: {exc}); disabled")
                self._disabled = True
                return False
            self._RigidPrim = RigidPrim
            self._XFormPrim = XFormPrim
            self._SimulationManager = SimulationManager
            self._stage = get_current_stage()
            self._sim_view = SimulationManager.get_physics_sim_view()

        if self._wrist_view is None:
            try:
                self._wrist_view = self._xform_view(self._wrist_path, "grip_ld_wrist")
            except Exception:
                self._wrist_view = None
                return False

        if self._auto and not self._jaw_views:
            try:
                self._jaw_views = [self._xform_view(p, f"grip_ld_jaw{i}")
                                   for i, p in enumerate(self._jaw_paths)]
            except Exception:
                self._jaw_views = []

        for path in self._explicit:
            self._add_view(path)

        if self._obj_views or self._auto:
            print(f"[GripContactSensor] payload observer -> wrench at link "
                  f"'{self._frame_id}', sign "
                  f"{'+1 (env->tool)' if self._sign > 0 else '-1 (tool->env)'}, "
                  f"ema {self._ema}, "
                  + ("auto-discover in the jaw" if self._auto else "explicit paths only")
                  + (f", explicit {self._explicit}" if self._explicit else ""))
        return True

    def _xform_view(self, path, name):
        """Pose-only view. ``reset_xform_properties=False`` is load-bearing: the
        default rewrites the prim's xform op stack and teleports it (see the
        "Stage hygiene" note in the module docstring)."""
        return self._XFormPrim(path, name=name, reset_xform_properties=False)

    def _probe_view(self, path):
        """Cached pose-only view used by discovery. Built once per prim path.

        Never a ``RigidPrim``: that would build a PhysX rigid-body view (and read
        velocities off it) for every candidate on every scan.
        """
        view = self._probe_views.get(path)
        if view is None:
            view = self._xform_view(path, f"grip_ld_probe_{abs(hash(path)) % 100000}")
            self._probe_views[path] = view
        return view

    def _add_view(self, path) -> bool:
        if path in self._obj_views:
            return True
        try:
            from isaacsim.core.utils.prims import is_prim_path_valid
        except Exception:
            is_prim_path_valid = lambda p: True  # noqa: E731
        if not is_prim_path_valid(path):
            return False
        try:
            view = self._RigidPrim(path, name=f"grip_ld_obj_{abs(hash(path)) % 100000}",
                                   reset_xform_properties=False)
            # __init__ already builds the physics handle when physics is live;
            # only initialize() if it did not, so we never create a second view.
            if not view.is_physics_handle_valid():
                view.initialize()
            self._obj_views[path] = view
            print(f"[GripContactSensor] tracking '{path}'")
            return True
        except Exception as exc:
            if not self._warned:
                print(f"[GripContactSensor] view for '{path}' not ready "
                      f"({exc.__class__.__name__}: {exc}); will retry")
                self._warned = True
            return False

    def _drop_view(self, path):
        self._obj_views.pop(path, None)
        for d in (self._v_prev, self._a_filt, self._reads, self._in_streak, self._out_streak):
            d.pop(path, None)
        print(f"[GripContactSensor] released '{path}' (left the jaw)")

    # -------------------------------------------------------- view lifecycle

    def _rebind_if_stale(self) -> bool:
        """Re-init the object views if the simulation view was swapped out.

        ``World.reset()`` (and any timeline stop/play) invalidates the old
        ``SimulationView`` and builds a new one. Views created against the old
        one recover via the ``PHYSICS_READY`` callback they register themselves,
        but only for that path -- comparing the object identity here also covers
        a swap that arrives without the event.

        Returns False if physics is not up, in which case there is nothing to
        read this step.
        """
        if self._SimulationManager is None:
            return True
        sim_view = self._SimulationManager.get_physics_sim_view()
        if sim_view is None:
            return False
        if sim_view is self._sim_view:
            return True

        self._sim_view = sim_view
        for path, view in list(self._obj_views.items()):
            try:
                if not view.is_physics_handle_valid():
                    view.initialize()
            except Exception:
                self._obj_views.pop(path, None)
        # velocity history spans the discontinuity; drop it so the inertial term
        # does not spike on the first read after the reset.
        self._v_prev.clear()
        self._a_filt.clear()
        self._reads.clear()
        return True

    # --------------------------------------------------------------- discovery

    def _jaw_centre(self):
        pts = []
        for v in self._jaw_views:
            try:
                p, _ = self._pose(v)
                pts.append(p)
            except Exception:
                return None
        return None if not pts else np.mean(pts, axis=0)

    def _candidate_paths(self):
        from pxr import Usd, UsdPhysics
        root = self._stage.GetPrimAtPath(self._discover_root)
        if not root or not root.IsValid():
            return []
        out = []
        for prim in Usd.PrimRange(root):
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            p = prim.GetPath().pathString
            if any(p == e or p.startswith(e + "/") for e in self._exclude):
                continue
            rb = UsdPhysics.RigidBodyAPI(prim)
            en = rb.GetRigidBodyEnabledAttr()
            if en and en.HasAuthoredValue() and not en.Get():
                continue
            kin = rb.GetKinematicEnabledAttr()
            if kin and kin.HasAuthoredValue() and kin.Get():
                continue
            out.append(p)
        return out

    def _scan(self):
        centre = self._jaw_centre()
        if centre is None:
            return
        try:
            candidates = set(self._candidate_paths())
        except Exception:
            return

        for path in candidates:
            if path in self._explicit:
                continue
            try:
                view = self._obj_views.get(path) or self._probe_view(path)
                pos, _ = self._pose(view)
            except Exception:
                continue
            inside = np.linalg.norm(np.asarray(pos) - centre) <= self._radius
            if inside:
                self._in_streak[path] = self._in_streak.get(path, 0) + 1
                self._out_streak[path] = 0
                if path not in self._obj_views and self._in_streak[path] >= self._persist:
                    self._add_view(path)
            elif path in self._obj_views:
                self._out_streak[path] = self._out_streak.get(path, 0) + 1
                self._in_streak[path] = 0
                if self._out_streak[path] >= self._persist:
                    self._drop_view(path)

        # a tracked path that vanished from the stage entirely
        for path in list(self._obj_views):
            if path not in self._explicit and path not in candidates:
                self._drop_view(path)

        # keep the probe cache from growing over deleted prims
        for path in list(self._probe_views):
            if path not in candidates:
                self._probe_views.pop(path, None)

    # ------------------------------------------------------------------ read

    def read(self):
        """Return ``(force, torque)`` numpy (3,) arrays in the wrist frame.

        ``(None, None)`` while physics is not up or nothing is tracked. Near
        zero once live means the tracked object is in free fall or resting on a
        surface (no net support from the gripper).
        """
        if self._disabled:
            self._last_error = "disabled"
            return None, None
        if self._wrist_view is None or (self._auto and not self._jaw_views) \
                or (not self._auto and len(self._obj_views) < len(self._explicit)):
            if not self.prepare():
                self._last_error = "views not built yet (wrist/jaw prims not resolvable)"
                return None, None
        if not self._rebind_if_stale():
            self._last_error = "physics simulation view not created yet"
            return None, None

        self._read_count += 1
        if self._auto and self._read_count % self._scan_every == 1:
            try:
                self._scan()
            except Exception:
                pass
        if not self._obj_views:
            # Expected whenever the jaw is empty: with no payload there is no
            # extra load on the wrist, so there is nothing to report.
            self._last_error = "no object in the jaw (nothing tracked)"
            return None, None

        dt = self._world.get_physics_dt() or (1.0 / 60.0)
        try:
            p_wrist, q_wrist = self._pose(self._wrist_view)
            f_world = np.zeros(3)
            t_world = np.zeros(3)
            got = False
            for path, view in self._obj_views.items():
                if not view.is_physics_handle_valid():
                    continue        # handle not up yet; _rebind_if_stale retries
                v = _to_np(view.get_linear_velocities(clone=True)).reshape(-1, 3)[0]
                m = float(_to_np(view.get_masses(clone=True)).reshape(-1)[0])
                p_obj, _ = self._pose(view)

                n = self._reads.get(path, 0)
                self._reads[path] = n + 1
                v_prev = self._v_prev.get(path)
                self._v_prev[path] = v
                if v_prev is None or n < WARMUP_READS:
                    a = np.zeros(3)
                else:
                    a = np.clip((v - v_prev) / dt, -A_MAX, A_MAX)
                a_f = self._ema * a + (1.0 - self._ema) * self._a_filt.get(path, np.zeros(3))
                self._a_filt[path] = a_f

                f_obj = m * (GRAVITY - a_f)          # object -> gripper (env -> tool)
                f_world += f_obj
                t_world += np.cross(p_obj - p_wrist, f_obj)
                got = True
        except Exception as exc:                    # handles vanished mid-reset
            self._last_error = f"read raised {exc.__class__.__name__}: {exc}"
            return None, None
        if not got:
            self._last_error = ("tracked objects have no valid physics handle "
                                f"({list(self._obj_views)})")
            return None, None

        self._last_error = None
        f = _quat_rotate_inverse(q_wrist, f_world)
        t = _quat_rotate_inverse(q_wrist, t_world)
        return self._sign * f, self._sign * t

    def status(self) -> str:
        """Why the last ``read()`` produced nothing. Empty string when healthy."""
        return self._last_error or ""

    def _pose(self, view):
        for kw in (dict(clone=True, usd=False), dict(usd=False), dict(clone=True), {}):
            try:
                pos, quat = view.get_world_poses(**kw)
                return _to_np(pos).reshape(-1, 3)[0], _to_np(quat).reshape(-1, 4)[0]
            except TypeError:
                continue
        pos, quat = view.get_world_poses()
        return _to_np(pos).reshape(-1, 3)[0], _to_np(quat).reshape(-1, 4)[0]

    @property
    def tracked(self):
        """Prim paths currently contributing to the wrench."""
        return list(self._obj_views)

    # ------------------------------------------------------------------- ROS 2

    def try_enable_ros2(self, topic: str = "/grip_contact", frame_id: str | None = None) -> bool:
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import WrenchStamped
        except Exception as exc:
            print(f"[GripContactSensor] ROS 2 publish disabled ({exc.__class__.__name__}: {exc})")
            return False

        if frame_id is not None:
            self._frame_id = frame_id
        if not rclpy.ok():
            rclpy.init(args=None)
        self._ros_node = Node("grip_contact_sensor")
        self._ros_pub = self._ros_node.create_publisher(WrenchStamped, topic, 10)
        self._ros_msg = WrenchStamped()
        self._ros_msg.header.frame_id = self._frame_id
        print(f"[GripContactSensor] publishing geometry_msgs/WrenchStamped on '{topic}' "
              f"(frame_id '{self._frame_id}')")
        return True

    def publish(self, force, torque, sim_time: float | None = None) -> None:
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
