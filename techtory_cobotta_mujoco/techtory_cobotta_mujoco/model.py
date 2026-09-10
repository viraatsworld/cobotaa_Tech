"""Convert the installed Techtory Cobotta workcell description into a MuJoCo scene.

The URDF supplies link transforms, joint limits and meshes. What is added here is everything
a simulator needs and a description does not carry: position servos, a ground plane, equality
constraints standing in for the RG6's URDF mimics, and a wrist force/torque sensor pair at the
arm-flange -> gripper interface. Generated assets live in a work directory, never in the
installed description packages.

Adapted from rox_fr3_demo_mujoco/model.py. The differences are structural, not cosmetic: this
cell is welded to `world` (no floating base, no free joint, no odometry), and the actuator
gains come from config rather than from the URDF -- see the note on DEFAULT_GAINS.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


# The RG6 base link is the first solid frame below the tool flange -- exactly where a real
# wrist FT sensor bolts in, and the same frame techtory_cobotta_isaacsim samples via PhysX in
# spawners/ft_sensor.py. MuJoCo's force/torque sensors report the interaction wrench between
# the site's body and its parent body, so a site here measures the J6 <-> gripper interface.
FT_BODY = 'base_link'
FT_SITE = 'tcp_ft_frame'
# Must match `mujoco_sensor_name` in cobotta_mujoco.ros2_control.xacro. The hardware interface
# appends the default _force/_torque suffixes to find the two MJCF sensors.
FT_SENSOR = 'tcp_fts'

ARM_JOINT_PREFIX = 'cobotta_pro_joint_'
FINGER_JOINT = 'finger_joint'

# denso_robot_descriptions authors mass=1 kg / identity-inertia links and effort="1" joints for
# the CVRB0609. Deriving forcerange from the URDF the way rox_fr3_demo_mujoco does for the FR3
# would cap every actuator at 1 Nm and the arm would fold on the first physics step, so gains
# come from config/actuators.yaml instead -- these are only the fallback.
DEFAULT_GAINS = {
    'arm': {'kp': 4000.0, 'kv': 200.0, 'forcerange': 300.0},
    'finger': {'kp': 200.0, 'kv': 10.0, 'forcerange': 20.0},
}
DEFAULT_MIMIC_SOLREF = '0.002 1'

# MuJoCo refuses meshes above this face count. The Techtory cell shell ships at ~467k faces,
# so it has to be decimated on the way in; the target leaves headroom under the hard cap.
MAX_MESH_FACES = 200_000
DECIMATED_FACES = 150_000

# Links whose collision geometry is dropped when the scene is built.
#
# MuJoCo collides meshes as convex hulls. `cell_link` is the workcell shell: a 2.2 m cube that
# encloses the robot and fills only 18% of its own hull, so as a collider it becomes a solid
# 7.8 m^3 block with the arm buried inside it, permanently in contact. It stays as a visual.
# The shelf is authored from boxes and the base plate is a 95%-convex slab, so both collide
# faithfully and are left alone.
VISUAL_ONLY_LINKS = frozenset({'cell_link'})

# The SRDF's disable_collisions list is the authority on which link pairs are allowed to
# overlap. MoveIt already plans against it, so reusing it keeps the simulator's contact set
# and the planner's collision matrix in agreement instead of hand-maintaining a second list.
DEFAULT_SRDF = ('techtory_cobotta_moveit', 'config/techtory_demo_description.srdf')


def _mesh_path(uri: str) -> Path:
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory

        package, relative = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / relative
    return Path(uri.removeprefix('file://')).resolve()


def _prepare_meshes(robot: ET.Element, assets: Path) -> None:
    """Flatten COLLADA scene transforms and use unique mesh names for MuJoCo.

    The cell contributes .STL, the RG6 .stl and the Denso arm .dae, so the COLLADA branch is
    not optional here.
    """
    converted: dict[str, Path] = {}
    for mesh in robot.iter('mesh'):
        uri = mesh.attrib['filename']
        if uri not in converted:
            source = _mesh_path(uri)
            digest = hashlib.sha256(str(source).encode()).hexdigest()[:12]
            destination = assets / f'{source.stem}_{digest}.stl'
            if source.suffix.lower() == '.stl' and _stl_face_count(source) <= MAX_MESH_FACES:
                shutil.copyfile(source, destination)
            else:
                import trimesh

                geometry = trimesh.load(source, force='scene').to_mesh()
                if len(geometry.faces) > MAX_MESH_FACES:
                    geometry = geometry.simplify_quadric_decimation(
                        face_count=DECIMATED_FACES)
                # Binary STL avoids external material/texture dependencies and preserves the
                # URDF material colors on the imported geoms.
                geometry.export(destination, file_type='stl')
            converted[uri] = destination
        mesh.set('filename', str(converted[uri]))


def _stl_face_count(source: Path) -> int:
    """Face count of a binary STL, or 0 for an ASCII one (which trimesh must re-export)."""
    with source.open('rb') as handle:
        header = handle.read(84)
    if len(header) < 84 or header[:5].lower() == b'solid':
        return 0
    return int.from_bytes(header[80:84], 'little')


def _strip_visual_only_collisions(robot: ET.Element) -> None:
    """Remove collision geometry from links listed in VISUAL_ONLY_LINKS."""
    for link in robot.findall('link'):
        if link.get('name', '') not in VISUAL_ONLY_LINKS:
            continue
        for collision in link.findall('collision'):
            link.remove(collision)


def _deduplicate_links(robot: ET.Element) -> None:
    """Drop repeated <link> definitions, keeping the first.

    The expanded workcell declares `world` twice -- once by techtory_cobotta_workcell.urdf.xacro
    itself and once via techtory_cell.xacro. The MuJoCo URDF importer rejects the duplicate.
    """
    seen: set[str] = set()
    for link in robot.findall('link'):
        name = link.get('name', '')
        if name in seen:
            robot.remove(link)
            continue
        seen.add(name)


def _prepare_urdf(robot: ET.Element, assets: Path) -> None:
    # Keeping only these three tags also discards the RG6's legacy <transmission> blocks and
    # its libgazebo_ros2_control / mimic-joint <gazebo> plugins, which MuJoCo cannot use.
    for element in list(robot):
        if element.tag not in {'link', 'joint', 'material'}:
            robot.remove(element)
    _deduplicate_links(robot)
    _strip_visual_only_collisions(robot)
    extension = ET.SubElement(robot, 'mujoco')
    # fusestatic="false" is load-bearing: it stops MuJoCo folding fixed-jointed links into
    # their parent, which would erase the RG6 base_link body and with it the FT sensor's
    # parent interface.
    ET.SubElement(extension, 'compiler', {
        'discardvisual': 'false', 'strippath': 'false',
        'fusestatic': 'false', 'balanceinertia': 'true',
    })
    for link in robot.findall('link'):
        name = link.get('name', '')
        for index, collision in enumerate(link.findall('collision')):
            collision.set('name', f'{name}_collision_{index}')
        for index, visual in enumerate(link.findall('visual')):
            visual.set('name', f'{name}_visual_{index}')
    _prepare_meshes(robot, assets)


def _load_gains(config_file: str | None) -> tuple[dict, str]:
    """Read actuator gains from YAML, falling back to the module defaults."""
    gains = {group: dict(values) for group, values in DEFAULT_GAINS.items()}
    solref = DEFAULT_MIMIC_SOLREF
    overrides: dict[str, dict] = {}
    if config_file:
        import yaml

        loaded = yaml.safe_load(Path(config_file).read_text(encoding='utf-8')) or {}
        for group, values in (loaded.get('defaults') or {}).items():
            gains.setdefault(group, {}).update(values or {})
        overrides = loaded.get('joints') or {}
        solref = ((loaded.get('mimic') or {}).get('solref')) or solref
    gains['_overrides'] = overrides
    return gains, solref


def _disable_visual_contacts(scene: ET.Element) -> int:
    """Stop <visual> geoms taking part in collision detection.

    MuJoCo's URDF importer turns both <visual> and <collision> into ordinary geoms, and an
    ordinary geom collides. Left alone, every link's visual mesh immediately contacts its own
    collision mesh -- overlapping by construction -- and the resulting contact forces blow the
    solver up within a second. Setting `group` alone is not enough: that only hides them in
    the viewer, it changes no physics.
    """
    disabled = 0
    for geom in scene.iter('geom'):
        if '_visual_' in geom.get('name', ''):
            geom.set('contype', '0')
            geom.set('conaffinity', '0')
            disabled += 1
    return disabled


def _srdf_disabled_pairs(srdf_path: str | None) -> list[tuple[str, str]]:
    """Read allowed-overlap link pairs from a MoveIt SRDF."""
    if srdf_path is None:
        package, relative = DEFAULT_SRDF
        from ament_index_python.packages import get_package_share_directory

        srdf_path = str(Path(get_package_share_directory(package)) / relative)
    root = ET.parse(srdf_path).getroot()
    pairs = []
    for entry in root.findall('disable_collisions'):
        first, second = entry.get('link1'), entry.get('link2')
        if first and second:
            pairs.append((first, second))
    return pairs


def _exclude_self_collisions(scene: ET.Element, pairs) -> int:
    """Emit <contact><exclude> for each SRDF pair that exists as a body in the model.

    The RG6 is the reason this is not optional: its four-bar linkage is a closed loop held by
    equality constraints, so the knuckles and fingers overlap permanently by design and the
    contact forces between them diverge.
    """
    bodies = {body.get('name') for body in scene.iter('body')}
    contact = ET.SubElement(scene, 'contact')
    written = 0
    for first, second in pairs:
        if first in bodies and second in bodies:
            ET.SubElement(contact, 'exclude', {'body1': first, 'body2': second})
            written += 1
    if not written:
        scene.remove(contact)
    return written


def _add_actuators(scene: ET.Element, robot: ET.Element, gains: dict, solref: str) -> None:
    """Emit one position servo per controlled joint, plus equalities for the RG6 mimics.

    MuJoCo actuator names must equal their URDF joint name -- the hardware interface maps
    tendon-transmission actuators to joints by name.
    """
    actuators = ET.SubElement(scene, 'actuator')
    equality = ET.SubElement(scene, 'equality')
    overrides = gains['_overrides']
    limits: dict[str, float] = {}
    for source in robot.findall('joint'):
        name = source.attrib['name']
        if source.get('type') == 'fixed':
            continue
        mimic = source.find('mimic')
        if mimic is not None:
            # The RG6's four-bar linkage is a tree in URDF; MuJoCo reproduces the closure with
            # a polynomial joint coupling. polycoef is (offset, multiplier, 0, 0, 0).
            ET.SubElement(equality, 'joint', {
                'joint1': name, 'joint2': mimic.attrib['joint'],
                'polycoef': f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
                'solref': solref,
            })
            continue
        if name.startswith(ARM_JOINT_PREFIX):
            group = 'arm'
        elif name == FINGER_JOINT:
            group = 'finger'
        else:
            # Anything else in the cell is passive scenery; leave it unactuated.
            continue
        tuning = dict(gains[group])
        tuning.update(overrides.get(name) or {})
        force = float(tuning['forcerange'])
        limits[name] = force
        attributes = {
            'name': name, 'joint': name,
            'kp': str(tuning['kp']), 'kv': str(tuning['kv']),
            'forcerange': f'-{force} {force}',
        }
        limit = source.find('limit')
        if limit is not None and limit.get('lower') and limit.get('upper'):
            attributes['ctrlrange'] = f"{limit.attrib['lower']} {limit.attrib['upper']}"
        ET.SubElement(actuators, 'position', attributes)
    if not len(equality):
        scene.remove(equality)
    _raise_joint_force_limits(scene, limits)


def _raise_joint_force_limits(scene: ET.Element, limits: dict[str, float]) -> None:
    """Widen each controlled joint's own actuator-force clamp to match its servo.

    MuJoCo's URDF importer turns <limit effort="..."> into a *joint-level* `actuatorfrcrange`,
    which is applied on top of the actuator's `forcerange` -- the tighter of the two wins. The
    Denso description authors effort="1" as a placeholder, so every arm joint arrives clamped
    at +/-1 Nm: the servo saturates instantly, the arm falls under its own weight and the
    solver diverges a second later. Setting the actuator's forcerange alone does nothing.
    """
    for joint in scene.iter('joint'):
        force = limits.get(joint.get('name', ''))
        if force is not None:
            joint.set('actuatorfrcrange', f'-{force} {force}')


def _add_force_torque_sensor(scene: ET.Element) -> None:
    """Bolt a 6-axis FT sensor onto the flange <-> gripper interface.

    A `force`/`torque` pair on a site inside the gripper's base body reads the wrench that
    body's incoming (fixed) joint carries -- the same quantity a real wrist FT sensor measures
    and the same one the Isaac Sim side reports on /wrist_ft.

    The hardware interface negates MuJoCo's raw sensordata when it reads these, so the six
    state interfaces already carry the ROS-conventional reaction wrench.
    """
    body = None
    for candidate in scene.iter('body'):
        if candidate.get('name') == FT_BODY:
            body = candidate
            break
    if body is None:
        names = sorted(b.get('name', '') for b in scene.iter('body'))
        raise ValueError(
            f"Cannot mount the force/torque sensor: no body named '{FT_BODY}' in the compiled "
            'model. The gripper base link is where the wrist sensor belongs; the description '
            f'must have been re-prefixed. Bodies present: {names}'
        )
    ET.SubElement(body, 'site', {
        'name': FT_SITE, 'pos': '0 0 0', 'size': '0.005', 'group': '3',
    })
    sensors = ET.SubElement(scene, 'sensor')
    ET.SubElement(sensors, 'force', {'name': f'{FT_SENSOR}_force', 'site': FT_SITE})
    ET.SubElement(sensors, 'torque', {'name': f'{FT_SENSOR}_torque', 'site': FT_SITE})


def generate_scene(
    urdf_xml: str,
    output_dir: str,
    actuator_config: str | None = None,
    srdf: str | None = None,
) -> str:
    """Write a validated MJCF scene and its assets; return the absolute scene filename.

    Requires Python ``mujoco``, ``trimesh``, ``pycollada`` and a sourced ROS environment to
    resolve ``package://`` mesh URIs. The base is welded to ``world`` -- there is no free
    joint. Every controlled actuator carries its URDF joint name.

    ``srdf`` supplies the allowed-overlap link pairs; it defaults to the MoveIt config's.
    """
    import mujoco

    directory = Path(output_dir).resolve()
    assets = directory / 'assets'
    assets.mkdir(parents=True, exist_ok=True)
    gains, solref = _load_gains(actuator_config)

    robot = ET.fromstring(urdf_xml)
    _prepare_urdf(robot, assets)

    imported = mujoco.MjModel.from_xml_string(ET.tostring(robot, encoding='unicode'))
    scene_path = directory / 'scene.xml'
    mujoco.mj_saveLastXML(str(scene_path), imported)

    scene = ET.parse(scene_path).getroot()
    ET.SubElement(scene, 'option', {
        'timestep': '0.002', 'integrator': 'implicitfast',
        'cone': 'elliptic', 'iterations': '80',
    })
    world = scene.find('worldbody')
    for geom in scene.iter('geom'):
        if '_collision_' in geom.get('name', ''):
            # MuJoCo hides group 3 by default; overlapping collision and visual meshes
            # otherwise flicker in the viewer. Physics is unchanged.
            geom.set('group', '3')
            geom.set('friction', '1.0 0.005 0.0001')
            geom.set('condim', '4')
    ET.SubElement(world, 'geom', {
        'name': 'ground', 'type': 'plane', 'size': '20 20 0.1',
        'rgba': '0.25 0.28 0.30 1', 'friction': '1.0 0.005 0.0001',
        'condim': '4',
    })
    ET.SubElement(world, 'light', {'pos': '0 0 5', 'dir': '0 0 -1', 'directional': 'true'})
    # The cell is ~2 m tall and the arm works around z=1; frame the camera on that.
    ET.SubElement(scene, 'statistic', {'center': '0 0 1.0', 'extent': '2.5'})

    _disable_visual_contacts(scene)
    _exclude_self_collisions(scene, _srdf_disabled_pairs(srdf))
    _add_force_torque_sensor(scene)
    _add_actuators(scene, robot, gains, solref)

    ET.indent(scene)
    ET.ElementTree(scene).write(scene_path, encoding='unicode')
    # Fail during launch setup with a useful compiler error, before controllers are spawned
    # against a simulator that cannot start.
    mujoco.MjModel.from_xml_path(str(scene_path))
    return str(scene_path)
