# Copyright (c) 2026, Fraunhofer IPA.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Spawners for the two scene assets the stock configs cannot express.

* :class:`StaticMeshFileCfg` -- an STL (the cell, the base plate) as a static
  body. On PhysX it is an **exact triangle-mesh collider**: the cell is a hollow
  shell that fills about 18 % of its own convex hull, and as a convex hull it is
  a solid 2 m block with the robot buried inside it (the Isaac Lab form of
  ``make_cell_collisions_static`` in ``techtory_cobotta_isaacsim``). On Newton
  (MuJoCo Warp) the cell is **visual-only**, because MuJoCo collides every mesh
  as its convex hull -- the same call ``techtory_cobotta_mujoco`` made.
* :class:`GraspableUsdFileCfg` -- the hammer, made grippable: one rigid body with
  a set mass, a high-friction material and ``convexDecomposition`` collision.
  The Isaac Lab form of ``configure_graspable_object``; its docstring explains
  why each of the three is needed. On Newton the collider is a single
  ``convexHull`` instead: Newton decomposes with CoACD using builder-wide
  settings no per-shape attribute reaches, and on the hammer that yields a
  near-degenerate piece MuJoCo refuses to compile ("mesh surface area is too
  small").

Both work **without Kit**: each builds a small USD layer once with ``pxr``,
caches it under the system temp dir, and hands it to Isaac Lab's own
``spawn_from_usd``, which handles the per-environment cloning. Nothing edits
the live stage, and nothing needs ``omni.*`` -- Isaac Lab's ``MeshConverter``
does (``omni.kit.commands``), which is why it is not used here.

``pxr`` is imported inside the functions: this module is imported while task
configs are resolved, before the simulation exists.
"""

from __future__ import annotations

import hashlib
import os
import tempfile
from collections.abc import Callable
from pathlib import Path
from typing import TYPE_CHECKING

import isaaclab.sim as sim_utils
from isaaclab.utils.configclass import configclass

if TYPE_CHECKING:
    from pxr import Usd

__all__ = [
    "GraspableUsdFileCfg",
    "StaticMeshFileCfg",
    "newton_physics_active",
    "spawn_graspable_usd",
    "spawn_static_mesh",
]

CACHE_DIR = Path(tempfile.gettempdir()) / "IsaacLab" / "techtory_cobotta"
# Bump when the generated layers change shape, so stale caches are not reused.
_CACHE_VERSION = "3"


def _cache_path(stem: str, suffix: str, *parts: object) -> Path:
    digest = hashlib.sha1("|".join(map(str, (_CACHE_VERSION, *parts))).encode()).hexdigest()[:12]
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    return CACHE_DIR / f"{stem}_{digest}{suffix}"


def _file_signature(path: str) -> tuple[str, int, int]:
    stat = os.stat(path)
    return (os.path.realpath(path), stat.st_size, int(stat.st_mtime))


def newton_physics_active() -> bool:
    """Whether the running simulation uses a Newton physics backend.

    Read from the resolved ``SimulationCfg.physics``, so it follows
    ``physics=newton_mjwarp`` / ``--physics newton_mjwarp`` without the scene
    needing a preset of its own.
    """
    try:
        from isaaclab.sim import SimulationContext

        sim = SimulationContext.instance()
    except Exception:
        return False
    physics = getattr(getattr(sim, "cfg", None), "physics", None)
    return type(physics).__name__.startswith("Newton")


##
# Static meshes.
##


def _read_stl(path: str):
    """Triangle vertices of an STL as an ``(3 * n, 3)`` float32 array."""
    import numpy as np

    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
        if len(data) == 84 + 50 * count:
            record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
            triangles = np.frombuffer(data, dtype=record, count=count, offset=84)
            return np.ascontiguousarray(triangles["vertices"].reshape(-1, 3))
    vertices = [
        [float(v) for v in line.split()[1:4]]
        for line in data.decode("ascii", errors="ignore").splitlines()
        if line.strip().startswith("vertex")
    ]
    return np.asarray(vertices, dtype=np.float32)


def _build_mesh_layer(cfg: StaticMeshFileCfg, collide: bool) -> str:
    """Write the STL as a one-mesh USD layer (cached) and return its path."""
    out = _cache_path(Path(cfg.mesh_path).stem, ".usdc", *_file_signature(cfg.mesh_path), collide, cfg.color)
    if out.exists():
        return str(out)

    import numpy as np
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

    vertices = _read_stl(cfg.mesh_path)
    triangles = len(vertices) // 3

    tmp = out.with_suffix(".tmp.usdc")
    stage = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/mesh")
    stage.SetDefaultPrim(root.GetPrim())

    mesh = UsdGeom.Mesh.Define(stage, "/mesh/geometry")
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(vertices))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(triangles, 3, dtype=np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(3 * triangles, dtype=np.int32)))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*cfg.color)])
    lo, hi = vertices.min(axis=0), vertices.max(axis=0)
    mesh.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])

    if collide:
        # No RigidBodyAPI anywhere: a collider without a rigid body is static,
        # which is also what permits an exact triangle mesh on PhysX.
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set("none")

    stage.GetRootLayer().Save()
    del stage
    os.replace(tmp, out)
    return str(out)


def spawn_static_mesh(
    prim_path: str,
    cfg: StaticMeshFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn ``cfg.mesh_path`` as a static mesh, colliding per ``cfg.collide``."""
    collide = cfg.collide if cfg.collide is not None else not newton_physics_active()
    usd_cfg = sim_utils.UsdFileCfg(usd_path=_build_mesh_layer(cfg, collide))
    return sim_utils.spawn_from_usd(prim_path, usd_cfg, translation, orientation, **kwargs)


@configclass
class StaticMeshFileCfg(sim_utils.UsdFileCfg):
    """An STL spawned as a static body, exact-mesh collider or visual-only."""

    func: Callable = spawn_static_mesh

    mesh_path: str = ""
    """STL to spawn. Converted to USD once and cached under the system temp dir."""

    collide: bool | None = None
    """``True``: exact triangle-mesh collider. ``False``: visual only.
    ``None`` (default): exact on PhysX, visual-only on Newton, whose MuJoCo
    contact pipeline would collide the mesh as its convex hull."""

    color: tuple[float, float, float] = (0.55, 0.57, 0.6)
    """Display colour (``primvars:displayColor``); works without Kit."""

    usd_path: str = ""
    """Unused; the spawner generates the USD."""


##
# Graspable objects.
##


def _build_graspable_layer(cfg: GraspableUsdFileCfg, approximation: str) -> str:
    """Write a wrapper layer that references the object prim, with grasp overrides.

    ``approximation`` is the ``physics:approximation`` for every collision mesh:
    ``convexDecomposition`` on PhysX, ``convexHull`` on Newton.
    """
    out = _cache_path(
        Path(cfg.usd_path).stem,
        ".usda",
        *_file_signature(cfg.usd_path),
        approximation,
        cfg.source_prim,
        cfg.object_mass,
        cfg.static_friction,
        cfg.dynamic_friction,
        cfg.max_convex_hulls,
    )
    if out.exists():
        return str(out)

    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    tmp = out.with_suffix(".tmp.usda")
    stage = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # The spawn root is a clean, identity-transform rigid body; the source
    # prim is referenced one level BELOW it. The hammer file is authored in
    # millimetres and carries its unit fix-up as xform ops on the object prim
    # itself (scale 0.1, rotateX:unitsResolve, scale:unitsResolve 0.001).
    # Isaac Lab rewrites the spawn root's xform ops into its standard
    # translate/orient/scale when it places the object, so those ops must not
    # live on the root -- on a child they compose exactly as authored.
    root = UsdGeom.Xform.Define(stage, "/object").GetPrim()
    stage.SetDefaultPrim(root)
    geometry = stage.DefinePrim("/object/geometry", "Xform")
    # Reference the object prim itself, not the file's default prim: the
    # hammer file is a saved Kit stage, and its default prim /World also carries
    # a distant light and render settings that would otherwise be copied into
    # every environment.
    geometry.GetReferences().AddReference(cfg.usd_path, Sdf.Path(cfg.source_prim))

    # ONE rigid body, on the root, for RigidObject to bind to. The source prim
    # arrives with its own RigidBodyAPI; a nested second body would split the
    # object in two.
    for prim in Usd.PrimRange(geometry):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    UsdPhysics.RigidBodyAPI.Apply(root)
    # Without MassAPI, mass and inertia come from the collision hull at
    # default density -- not the object you can see.
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr().Set(float(cfg.object_mass))

    material = UsdShade.Material.Define(stage, "/object/PhysicsMaterial")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(float(cfg.static_friction))
    physics_material.CreateDynamicFrictionAttr().Set(float(cfg.dynamic_friction))
    physics_material.CreateRestitutionAttr().Set(0.0)

    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "Mesh":
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        # A hammer's convex hull is a solid wedge from head to handle that
        # fills the notch the pads are meant to close on. Decomposition keeps
        # head and handle as separate hulls -- on PhysX, whose decomposition
        # these physx* attributes tune. Newton ignores them (see module
        # docstring) and gets a single hull.
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set(approximation)
        if approximation == "convexDecomposition":
            prim.AddAppliedSchema("PhysxConvexDecompositionCollisionAPI")
            prim.CreateAttribute("physxConvexDecompositionCollision:maxConvexHulls", Sdf.ValueTypeNames.Int).Set(
                int(cfg.max_convex_hulls)
            )
            prim.CreateAttribute(
                "physxConvexDecompositionCollision:errorPercentage", Sdf.ValueTypeNames.Float
            ).Set(2.0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")

    stage.GetRootLayer().Save()
    del stage
    os.replace(tmp, out)
    return str(out)


def spawn_graspable_usd(
    prim_path: str,
    cfg: GraspableUsdFileCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> Usd.Prim:
    """Spawn a USD object made grippable (see :func:`_build_graspable_layer`)."""
    if not os.path.isfile(cfg.usd_path):
        raise FileNotFoundError(
            f"{cfg.usd_path} not found. It ships with techtory_cobotta_isaacsim; keep that "
            "package beside this one or point TECHTORY_HAMMER_USD at the file."
        )
    approximation = "convexHull" if newton_physics_active() else "convexDecomposition"
    usd_cfg = sim_utils.UsdFileCfg(usd_path=_build_graspable_layer(cfg, approximation))
    return sim_utils.spawn_from_usd(prim_path, usd_cfg, translation, orientation, **kwargs)


@configclass
class GraspableUsdFileCfg(sim_utils.UsdFileCfg):
    """A USD object made grippable: rigid, massed, high friction, decomposed."""

    func: Callable = spawn_graspable_usd

    source_prim: str = "/World/hammer"
    """Prim inside ``usd_path`` to reference (the object, not the whole saved stage)."""

    object_mass: float = 0.3
    """kg -- techtory_cobotta_isaacsim's configure_graspable_object default."""

    static_friction: float = 1.2
    dynamic_friction: float = 1.1
    """PhysX averages the two materials in a contact, so the object needs as
    much friction as the pads, not just the pads."""

    max_convex_hulls: int = 32
