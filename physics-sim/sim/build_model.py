"""Build the MuJoCo model (bunny.xml) from the BOM and the part-mass table.

Steps:
  1. Read the BOM export and part_masses.csv, and check they agree.
  2. Lump every part into the rigid body it rides on (torso, thighs, shins,
     front legs), computing each body's total mass, center of mass and inertia.
  3. Write an MJCF file using those lumped values plus the geometry in config.py.

Run:  python sim/build_model.py   (writes bunny_<design>.xml for every design in config.DESIGNS)
"""

import csv
import math
from dataclasses import dataclass

import numpy as np

import dataclasses

from config import BOM_CSV, DESIGNS, PART_MASSES_CSV, BunnyDesign, model_path
from legs import peak_spring_torque, winder_gear_ratio

SIDES = {"left": +1, "right": -1}


# ---------------------------------------------------------------------------
# 1. Read the spreadsheets
# ---------------------------------------------------------------------------

def read_bom_ids(bom_path):
    """Return {bom_id: item_name} for every item row in the BOM export."""
    with open(bom_path, newline="") as f:
        rows = list(csv.reader(f))

    header_index = next(i for i, row in enumerate(rows) if row and row[0] == "ID")
    header = rows[header_index]
    items = {}
    for row in rows[header_index + 1:]:
        if not row or not row[0].strip():
            continue
        record = dict(zip(header, row))
        items[record["ID"]] = record["Item"]
    return items


def read_part_masses(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def check_tables_agree(bom_items, mass_rows):
    """Warn about BOM items with no mass entry, and mass rows not in the BOM."""
    ids_with_mass = {row["bom_id"] for row in mass_rows}
    missing = [i for i in bom_items if i not in ids_with_mass]
    unknown = sorted(ids_with_mass - set(bom_items))
    proposed = sorted(i for i in unknown
                      if all(r["mass_source"].startswith("proposed") for r in mass_rows if r["bom_id"] == i))
    for bom_id in missing:
        print(f"WARNING: {bom_id} ({bom_items[bom_id]}) has no row in part_masses.csv")
    for bom_id in unknown:
        if bom_id not in proposed:
            print(f"WARNING: part_masses.csv mentions {bom_id}, which is not in the BOM")
    if proposed:
        print(f"Note: {len(proposed)} proposed parts are not in the BOM yet: {', '.join(proposed)}")


# ---------------------------------------------------------------------------
# 2. Lump parts into rigid bodies
# ---------------------------------------------------------------------------

@dataclass
class PointPart:
    mass: float  # kg
    position: np.ndarray  # m, in the body's frame
    box_size: np.ndarray  # m, full edge lengths (zeros = point mass)


@dataclass
class LumpedBody:
    mass: float
    center_of_mass: np.ndarray
    inertia: np.ndarray  # 3x3, about the center of mass


def box_inertia(mass, size):
    sx, sy, sz = size
    return mass / 12.0 * np.diag([sy**2 + sz**2, sx**2 + sz**2, sx**2 + sy**2])


def lump(parts):
    total_mass = sum(p.mass for p in parts)
    com = sum(p.mass * p.position for p in parts) / total_mass

    inertia = np.zeros((3, 3))
    for p in parts:
        offset = p.position - com
        # Parallel-axis theorem: own inertia + mass * (distance from the COM)
        parallel_axis = p.mass * (offset @ offset * np.eye(3) - np.outer(offset, offset))
        inertia += box_inertia(p.mass, p.box_size) + parallel_axis
    return LumpedBody(total_mass, com, inertia)


def row_applies(row, design):
    """A row's "when" column limits it to one drive or power option, e.g. "drive=latch"."""
    condition = row.get("when", "").strip()
    if not condition:
        return True
    setting, value = condition.split("=")
    return getattr(design, setting) == value


def parts_by_body(mass_rows, design):
    """Turn mass-table rows into PointParts grouped by MuJoCo body name.

    "rear_thigh_*" means one copy of the part on each side (left and right).
    mirror_y = yes means the quantity is split half at +y and half at -y.
    """
    bodies = {}

    def add(body_name, part):
        bodies.setdefault(body_name, []).append(part)

    for row in mass_rows:
        if row["sim_body"] == "off_robot" or not row_applies(row, design):
            continue
        qty = float(row["qty"])
        mass = qty * float(row["mass_each_g"]) / 1000.0
        position = np.array([float(row[k]) for k in ("x_mm", "y_mm", "z_mm")]) / 1000.0
        size = np.array([float(row[k]) for k in ("size_x_mm", "size_y_mm", "size_z_mm")]) / 1000.0

        if row["sim_body"].endswith("_*"):
            base = row["sim_body"][:-1]
            for side in SIDES:
                add(base + side, PointPart(mass, position, size))
        elif row["mirror_y"].strip().lower() == "yes":
            mirrored = position * np.array([1, -1, 1])
            add(row["sim_body"], PointPart(mass / 2, position, size))
            add(row["sim_body"], PointPart(mass / 2, mirrored, size))
        else:
            add(row["sim_body"], PointPart(mass, position, size))

    return {name: lump(parts) for name, parts in bodies.items()}


# ---------------------------------------------------------------------------
# 3. Geometry helpers
# ---------------------------------------------------------------------------

def rear_leg_length(design, hip_angle):
    """Hip-to-foot distance along the push axis for a given hip angle."""
    # Thigh and shin are equal-length with the knee coupled at -2x the hip,
    # so the foot stays on the push axis at this distance.
    return 2 * design.thigh_length * math.cos(hip_angle)


def crouched_torso_height(design):
    """Height of the torso origin when crouched on all four feet."""
    crouch = math.radians(design.crouch_angle_deg)
    tilt = math.radians(design.rear_leg_tilt_deg)
    hip_height = rear_leg_length(design, crouch) * math.cos(tilt) + design.foot_radius
    return hip_height - design.rear_hip_pos[2]


def front_leg_length(design):
    """Front leg length that keeps the torso level when crouched."""
    shoulder_height = crouched_torso_height(design) + design.front_shoulder_pos[2]
    return shoulder_height - design.foot_radius


# ---------------------------------------------------------------------------
# 4. Write the MJCF
# ---------------------------------------------------------------------------

def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


def inertial_xml(body):
    i = body.inertia
    full_inertia = [i[0, 0], i[1, 1], i[2, 2], i[0, 1], i[0, 2], i[1, 2]]
    return (f'<inertial pos="{fmt(body.center_of_mass)}" mass="{body.mass:.6g}" '
            f'fullinertia="{fmt(full_inertia)}"/>')


def mirror(position, side):
    x, y, z = position
    return (x, SIDES[side] * y, z)


def rear_leg_xml(design, bodies, side):
    hip_min, hip_max = (math.radians(a) for a in design.hip_range_deg)
    tilt = math.radians(design.rear_leg_tilt_deg)
    spring_rest = math.radians(design.spring_assist_rest_angle_deg)
    L1, L2 = design.thigh_length, design.shin_length
    return f"""
      <body name="rear_thigh_{side}" pos="{fmt(mirror(design.rear_hip_pos, side))}" euler="0 {tilt:.6g} 0">
        <joint name="hip_{side}" type="hinge" axis="0 1 0" range="{hip_min:.6g} {hip_max:.6g}"
               armature="{design.servo_armature}" damping="0.01"
               stiffness="{design.spring_assist_stiffness}" springref="{spring_rest:.6g}"/>
        {inertial_xml(bodies[f"rear_thigh_{side}"])}
        <geom type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.008" rgba="0.85 0.85 0.85 1"/>
        <body name="rear_shin_{side}" pos="0 0 {-L1}">
          <joint name="knee_{side}" type="hinge" axis="0 1 0" range="-3.1 0.3" damping="0.01"/>
          {inertial_xml(bodies[f"rear_shin_{side}"])}
          <geom type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.007" rgba="0.85 0.85 0.85 1"/>
          <geom name="rear_foot_{side}" type="sphere" pos="0 0 {-L2}" size="{design.foot_radius}" rgba="0.2 0.2 0.2 1"/>
        </body>
      </body>"""


def front_stance_angle(design):
    """Front hip angle that puts a servo-driven front foot on the floor when crouched."""
    full_length = design.front_thigh_length + design.front_shin_length
    return math.acos(front_leg_length(design) / full_length)


def front_leg_xml(design, bodies, side):
    length = front_leg_length(design)
    strut = f"""<body name="front_leg_{side}" pos="{fmt(mirror(design.front_shoulder_pos, side))}">
        <joint name="front_spring_{side}" type="slide" axis="0 0 1" range="0 {design.front_leg_travel}"
               stiffness="{design.front_spring_stiffness}" springref="0" damping="{design.front_spring_damping}"/>
        {inertial_xml(bodies[f"front_leg_{side}"])}"""
    if design.front_drive != "servo":
        return f"""
      {strut}
        <geom type="capsule" fromto="0 0 0 0 0 {-length:.6g}" size="0.007" rgba="0.85 0.85 0.85 1"/>
        <geom name="front_foot_{side}" type="sphere" pos="0 0 {-length:.6g}" size="{design.foot_radius}" rgba="0.2 0.2 0.2 1"/>
      </body>"""

    # Servo-driven: the same coupled-knee thigh + shin as the rear legs, hanging
    # under the sprung strut, with the foot moving straight up and down.
    hip_min, hip_max = (math.radians(a) for a in design.front_hip_range_deg)
    L1, L2 = design.front_thigh_length, design.front_shin_length
    return f"""
      {strut}
        <body name="front_thigh_{side}">
          <joint name="front_hip_{side}" type="hinge" axis="0 1 0" range="{hip_min:.6g} {hip_max:.6g}"
                 armature="{design.servo_armature}" damping="0.01"/>
          {inertial_xml(bodies[f"front_thigh_{side}"])}
          <geom type="capsule" fromto="0 0 0 0 0 {-L1}" size="0.007" rgba="0.85 0.85 0.85 1"/>
          <body name="front_shin_{side}" pos="0 0 {-L1}">
            <joint name="front_knee_{side}" type="hinge" axis="0 1 0" range="-3.1 0.3" damping="0.01"/>
            {inertial_xml(bodies[f"front_shin_{side}"])}
            <geom type="capsule" fromto="0 0 0 0 0 {-L2}" size="0.006" rgba="0.85 0.85 0.85 1"/>
            <geom name="front_foot_{side}" type="sphere" pos="0 0 {-L2}" size="{design.foot_radius}" rgba="0.2 0.2 0.2 1"/>
          </body>
        </body>
      </body>"""


def tether_xml(design):
    if design.tether_length is None:
        return "", ""
    anchor_site = f'<site name="tether_anchor" pos="{fmt(design.tether_anchor)}" size="0.02" rgba="1 0.5 0 1"/>'
    tendon = f"""
  <tendon>
    <!-- solreflimit makes the end stop slightly soft, like a real cable with some give -->
    <spatial name="tether" limited="true" range="0 {design.tether_length}" solreflimit="0.03 1"
             width="0.003" rgba="1 0.5 0 1">
      <site site="tether_anchor"/>
      <site site="tail"/>
    </spatial>
  </tendon>"""
    return anchor_site, tendon


def latch_xml(design):
    """Extra equality constraints and actuators for the wind-and-latch drive."""
    if design.drive != "latch":
        return "", ""
    crouch = math.radians(design.crouch_angle_deg)
    spring_limit = 1.5 * peak_spring_torque(design)
    pawls = "\n".join(
        f'    <joint name="latch_{side}" joint1="hip_{side}" polycoef="{crouch:.6g} 0 0 0 0" solref="0.002 1"/>'
        for side in SIDES)
    springs = "\n".join(
        f'    <motor name="spring_{side}" joint="hip_{side}" ctrlrange="{-spring_limit:.6g} {spring_limit:.6g}"/>'
        for side in SIDES)
    return (f"""
    <!-- Ratchet pawls: lock each hip where it caught. legs.py switches them on and off. -->
{pawls}""", f"""
    <!-- Latch-drive springs. legs.py computes the torque from the spring stretch. -->
{springs}""")


def front_drive_xml(design):
    """Knee linkages, servo actuators and IMU sensors for servo-driven front legs."""
    if design.front_drive != "servo":
        return "", "", ""
    knees = "\n".join(
        f'    <joint joint1="front_knee_{side}" joint2="front_hip_{side}" polycoef="0 -2 0 0 0" solref="0.002 1"/>'
        for side in SIDES)
    servos = "\n".join(
        f'    <motor name="front_servo_{side}" joint="front_hip_{side}" '
        f'ctrlrange="{-design.servo_stall_torque:.6g} {design.servo_stall_torque:.6g}"/>'
        for side in SIDES)
    imu = """
  <!-- MPU-6050 (HAL2-008). front_legs.py adds its noise, bias and range limits. -->
  <sensor>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_accel" site="imu"/>
  </sensor>"""
    return f"""
    <!-- Front-leg linkages, like the rear: knee angle = -2 x hip angle. -->
{knees}""", f"""
    <!-- Front-leg ST3025s, direct drive. The torque-speed limits are applied in servo.py. -->
{servos}""", imu


def build_mjcf(design: BunnyDesign, bodies):
    crouch = math.radians(design.crouch_angle_deg)
    start_height = crouched_torso_height(design) + 0.002
    # Keyframe qpos: torso position + quaternion, then each joint in tree order:
    # left hip, left knee, right hip, right knee, then the front legs.
    # Servo-driven front legs add their hip and knee after each front slide.
    if design.front_drive == "servo":
        stance = front_stance_angle(design)
        front_qpos = [0, stance, -2 * stance] * 2
    else:
        front_qpos = [0, 0]
    crouch_qpos = [0, 0, start_height, 1, 0, 0, 0,
                   crouch, -2 * crouch, crouch, -2 * crouch, *front_qpos]
    if design.drive == "latch":
        hip_torque_limit = design.servo_stall_torque * winder_gear_ratio(design)
    else:
        hip_torque_limit = design.servo_stall_torque / design.linkage_ratio
    anchor_site, tendon = tether_xml(design)
    pawl_constraints, spring_actuators = latch_xml(design)
    front_knees, front_servos, imu_sensors = front_drive_xml(design)

    return f"""<mujoco model="halo_bunny">
  <!-- Generated by build_model.py from the BOM and part_masses.csv. Do not edit by hand. -->
  <compiler angle="radian" autolimits="true"/>
  <!-- cone="elliptic": MuJoCo's default pyramidal friction grips differently along the
       floor's x/y axes than along diagonals, which made turning depend on world heading. -->
  <option timestep="{design.timestep}" integrator="implicitfast" cone="elliptic"/>

  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.25 0.27 0.3" rgb2="0.3 0.32 0.35" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="20 20"/>
  </asset>

  <default>
    <!-- Robot geoms collide with the floor but not with each other.
         Their friction is low so the floor's friction value always wins. -->
    <geom contype="2" conaffinity="1" condim="3" friction="0.1 0.005 0.0001"
          solref="0.004 1" solimp="0.9 0.95 0.001"/>
  </default>

  <worldbody>
    <light pos="0 0 3" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" size="10 10 0.1" contype="1" conaffinity="1"
          friction="{design.foot_floor_friction} 0.005 0.0001" material="grid"/>
    {anchor_site}

    <body name="torso" pos="0 0 {start_height:.6g}">
      <freejoint name="root"/>
      {inertial_xml(bodies["torso"])}
      <geom name="torso_shell" type="ellipsoid" size="{fmt(design.torso_half_size)}" rgba="0.95 0.93 0.9 1"/>
      <geom name="head" type="sphere" pos="{fmt(design.head_pos)}" size="{design.head_radius}" rgba="0.95 0.93 0.9 1"/>
      <geom name="ear_left" type="capsule" fromto="{fmt((0.11, 0.02, 0.08))} {fmt((0.08, 0.035, 0.17))}" size="0.012"
            contype="0" conaffinity="0" rgba="0.95 0.8 0.8 1"/>
      <geom name="ear_right" type="capsule" fromto="{fmt((0.11, -0.02, 0.08))} {fmt((0.08, -0.035, 0.17))}" size="0.012"
            contype="0" conaffinity="0" rgba="0.95 0.8 0.8 1"/>
      <site name="imu" pos="0 0 0.02" size="0.005"/>
      <site name="tail" pos="{fmt(design.tail_pos)}" size="0.01"/>
      {rear_leg_xml(design, bodies, "left")}
      {rear_leg_xml(design, bodies, "right")}
      {front_leg_xml(design, bodies, "left")}
      {front_leg_xml(design, bodies, "right")}
    </body>
  </worldbody>
{tendon}
  <equality>
    <!-- The hind-leg linkage: knee angle = -2 x hip angle keeps each foot on its push axis. -->
    <joint joint1="knee_left" joint2="hip_left" polycoef="0 -2 0 0 0" solref="0.002 1"/>
    <joint joint1="knee_right" joint2="hip_right" polycoef="0 -2 0 0 0" solref="0.002 1"/>{pawl_constraints}{front_knees}
  </equality>

  <actuator>
    <!-- Hip torque from each ST3025 (through the winder gears for the latch drive).
         The torque-speed limits are applied in servo.py. -->
    <motor name="servo_left" joint="hip_left" ctrlrange="{-hip_torque_limit:.6g} {hip_torque_limit:.6g}"/>
    <motor name="servo_right" joint="hip_right" ctrlrange="{-hip_torque_limit:.6g} {hip_torque_limit:.6g}"/>{spring_actuators}{front_servos}
  </actuator>{imu_sensors}

  <keyframe>
    <key name="crouch" qpos="{fmt(crouch_qpos)}"/>
  </keyframe>
</mujoco>
"""


def build(design: BunnyDesign, path, verbose=True):
    bom_items = read_bom_ids(BOM_CSV)
    mass_rows = read_part_masses(PART_MASSES_CSV)
    if verbose:
        check_tables_agree(bom_items, mass_rows)

    bodies = parts_by_body(mass_rows, design)
    path.write_text(build_mjcf(design, bodies))

    if verbose:
        total = sum(b.mass for b in bodies.values())
        print(f"Wrote {path.name}. Robot mass from BOM: {total * 1000:.0f} g")
        for name, body in bodies.items():
            com_mm = ", ".join(f"{v * 1000:.0f}" for v in body.center_of_mass)
            print(f"  {name:18s} {body.mass * 1000:6.0f} g   COM ({com_mm}) mm")
    return path


def build_named(design_name, tether_length=None, verbose=False):
    """Build one of the designs in config.DESIGNS, optionally with a tether."""
    design = DESIGNS[design_name]
    if tether_length is not None:
        design = dataclasses.replace(design, tether_length=tether_length)
    path = model_path(design_name, with_tether=tether_length is not None)
    return build(design, path, verbose)


if __name__ == "__main__":
    for i, name in enumerate(DESIGNS):
        build_named(name, verbose=(i == 0))
        if i > 0:
            print(f"Wrote {model_path(name).name}")
