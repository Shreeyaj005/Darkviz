#!/usr/bin/env python3
import math

# Geometry shared with the demo script (heights above ground, metres)
WHEEL_RADIUS = 0.08
WHEEL_WIDTH = 0.05
WHEEL_OFFSET = 0.24
AXLE_DROP = 0.036
BASE_LINK_Z = WHEEL_RADIUS + AXLE_DROP            # 0.116

PLATE_R = 0.22
BASE_T, MID_T, TOP_T = 0.012, 0.010, 0.010
MID_Z, TOP_Z = 0.101, 0.231
TOP_SURFACE_Z = TOP_Z + TOP_T / 2.0               # 0.236
RAIL_H = 0.012

TRAY_JOINT_Z = TOP_SURFACE_Z + RAIL_H + 0.006     # 0.254
TRAY_TRAVEL = 0.30
TRAY_FLOOR_TOP = BASE_LINK_Z + TRAY_JOINT_Z + 0.012   # 0.382

STATION_TOP = BASE_LINK_Z + MID_Z + MID_T / 2.0   # 0.222 (middle plate top)
SLAB_T = 0.02
SLAB_HALF = 0.08
HOLE_HALF = 0.0275
CYL_R = 0.02
CYL_LEN = 0.20
CYL_TOP = TRAY_FLOOR_TOP - 0.006
CYL_DROP = 0.03
CYL_TRAVEL = 0.04
APPLE_R = 0.035
APPLE_MASS = 0.15
POST_FROM_ROBOT = 0.33        # cylinder x in robot frame when the tray is fully out

# Omni roller layout: two rows, each WHEEL_WIDTH/2 wide (=2*ROLLER_R), flush with the
# wheel faces; the ring radius is chosen so the roller END corners sit on the
# WHEEL_RADIUS circle (sqrt((ring+R)^2 + half_len^2) = WHEEL_RADIUS).
ROLLERS_PER_ROW = 16
ROLLER_R = WHEEL_WIDTH / 4.0                       # 0.0125
ROLLER_HALF_LEN = 0.010
ROLLER_RING_R = math.sqrt(WHEEL_RADIUS ** 2 - ROLLER_HALF_LEN ** 2) - ROLLER_R   # ~0.0670

WHEELS = [("front", 90.0), ("rear_left", 210.0), ("rear_right", -30.0)]

# --------------------------------------------------------------------------
# colours
# --------------------------------------------------------------------------
PLATE_WHITE = (0.88, 0.88, 0.85, 1)
SILVER = (0.70, 0.70, 0.75, 1)
MOTOR_GRAY = (0.35, 0.37, 0.42, 1)
ROLLER = (0.12, 0.12, 0.13, 1)
STEPPER = (0.10, 0.10, 0.12, 1)
DARK = (0.20, 0.20, 0.20, 1)
ACRYLIC = (0.75, 0.88, 1.0, 0.30)
PCB = (0.10, 0.45, 0.20, 1)
LIDAR = (0.08, 0.08, 0.10, 1)
ORANGE = (0.90, 0.55, 0.10, 1)


# helpers
def rpy_quat(r, p, y):
    """URDF-style rpy (R = Rz(y) Ry(p) Rx(r)) -> MuJoCo quaternion (w x y z)."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def f(vals):
    return " ".join(f"{v:.6g}" for v in vals)


def geom(kind, size, pos=(0, 0, 0), quat=None, rgba=SILVER, collide=False):
    """One <geom>. Visual-only unless collide=True."""
    q = f' quat="{f(quat)}"' if quat is not None else ""
    if collide:
        phys = ' contype="1" conaffinity="1" friction="1 0.005 0.0001"'
    else:
        phys = ' contype="0" conaffinity="0"'
    return (f'<geom type="{kind}" size="{f(size)}" pos="{f(pos)}"{q} '
            f'rgba="{f(rgba)}"{phys}/>')


def box(half, pos, rgba, quat=None, collide=False):
    return geom("box", half, pos, quat, rgba, collide)


def cyl(radius, half_len, pos, rgba, quat=None, collide=False):
    return geom("cylinder", (radius, half_len), pos, quat, rgba, collide)


def hex_plate(r, t, z, rgba):
    """Regular hexagon (vertices at 30/90/150... deg) from 3 rotated boxes."""
    out = []
    for yaw in (30, 90, 150):
        out.append(box((r / 2.0, math.sqrt(3) * r / 2.0, t / 2.0), (0, 0, z), rgba,
                       quat=rpy_quat(0, 0, math.radians(yaw))))
    return out


def box_inertia(m, x, y, z):
    return (m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y))


# robot pieces
def robot_static_geoms():
    g = []
    # plates
    g += hex_plate(PLATE_R, BASE_T, 0.0, PLATE_WHITE)
    g += hex_plate(PLATE_R, MID_T, MID_Z, PLATE_WHITE)
    g += hex_plate(PLATE_R, TOP_T, TOP_Z, PLATE_WHITE)
    # 3 aluminum standoffs
    for ang in (30, 150, 270):
        a = math.radians(ang)
        g.append(cyl(0.006, 0.11, (0.15 * math.cos(a), 0.15 * math.sin(a), 0.116), SILVER))
    # clear acrylic enclosure + ESP32 carrier + module
    g.append(box((0.08, 0.055, 0.025), (0, 0, BASE_T / 2 + 0.025), ACRYLIC))
    g.append(box((0.06, 0.035, 0.002), (0, 0, BASE_T / 2 + 0.006), PCB))
    g.append(box((0.026, 0.014, 0.003), (0, 0, BASE_T / 2 + 0.011), STEPPER))
    # LiDAR on the middle plate (visual) + rangefinder module under top plate
    lidar_z = MID_Z + MID_T / 2 + 0.04
    g.append(cyl(0.07, 0.04, (0, 0, lidar_z), LIDAR))
    g.append(box((0.06, 0.035, 0.02), (0, 0, TOP_Z - TOP_T / 2 - 0.02), DARK))
    # steppers A1 (+y) / T1 (-y), drive pulleys, idler posts + pulleys, belts
    for s in (+1, -1):
        g.append(box((0.025, 0.025, 0.0225), (-0.12, s * 0.10, TOP_SURFACE_Z + 0.0225), STEPPER))
        g.append(cyl(0.012, 0.007, (-0.12, s * 0.10, TOP_SURFACE_Z + 0.052), SILVER))
        g.append(cyl(0.004, 0.026, (0.15, s * 0.10, TOP_SURFACE_Z + 0.026), SILVER))
        g.append(cyl(0.012, 0.007, (0.15, s * 0.10, TOP_SURFACE_Z + 0.059), SILVER))
        for off in (0.112, 0.088):
            g.append(box((0.135, 0.0015, 0.005), (0.015, s * off, TOP_SURFACE_Z + 0.059), ROLLER))
    # linear guide rail
    g.append(box((0.22, 0.012, RAIL_H / 2), (0.07, 0, TOP_SURFACE_Z + RAIL_H / 2), SILVER))
    return g


def hub_and_wheel(name, angle_deg):
    a = math.radians(angle_deg)
    q = rpy_quat(math.pi / 2, 0, a + math.pi / 2)     # local +z = radial outward
    ca, sa = math.cos(a), math.sin(a)
    z = -AXLE_DROP

    # DC gear motor (inboard of wheel along the axle)
    mr = 0.1475
    hub = f"""
    <body name="{name}_hub" pos="{f((mr * ca, mr * sa, z))}" quat="{f(q)}">
      <inertial pos="0 0 0" mass="0.25" diaginertia="2e-4 2e-4 1e-4"/>
      {cyl(0.025, 0.0375, (0, 0, 0), MOTOR_GRAY)}
      {box((0.025, 0.025, 0.015), (0, 0, 0.0525), SILVER)}
      {cyl(0.02, 0.01, (0, 0, -0.0475), STEPPER)}
    </body>"""

    # omni wheel: core + 2 staggered rows of rollers. Rollers are sized so that
    #   * each row is exactly half the wheel width (flush with both wheel faces)
    #   * the roller ends stay inside the r = WHEEL_RADIUS circle
    n = ROLLERS_PER_ROW
    rollers = [cyl(ROLLER_RING_R - ROLLER_R, WHEEL_WIDTH / 2, (0, 0, 0), SILVER)]
    for dz, phase in [(+ROLLER_R, 0.0), (-ROLLER_R, math.pi / n)]:
        for i in range(n):
            phi = phase + i * 2 * math.pi / n
            rollers.append(cyl(ROLLER_R, ROLLER_HALF_LEN,
                               (ROLLER_RING_R * math.cos(phi), ROLLER_RING_R * math.sin(phi), dz),
                               ROLLER, quat=rpy_quat(math.pi / 2, 0, phi)))
    m = 0.9
    ixx = m / 12 * (3 * WHEEL_RADIUS ** 2 + WHEEL_WIDTH ** 2)
    izz = 0.5 * m * WHEEL_RADIUS ** 2
    wheel = f"""
    <body name="{name}_wheel" pos="{f((WHEEL_OFFSET * ca, WHEEL_OFFSET * sa, z))}" quat="{f(q)}">
      <inertial pos="0 0 0" mass="{m}" diaginertia="{f((ixx, ixx, izz))}"/>
      <joint name="{name}_wheel_joint" type="hinge" axis="0 0 1" damping="0.01" armature="0.01"/>
      {chr(10).join('      ' + r for r in rollers)}
    </body>"""
    return hub + wheel


def tray_body():
    """Slotted (fork) tray on a prismatic joint along +x."""
    g = []
    g.append(box((0.035, 0.025, 0.006), (-0.05, 0, 0), DARK))                     # carriage
    g.append(box((0.025, 0.08, 0.003), (-0.055, 0, 0.009), PLATE_WHITE, collide=True))   # back plate
    for s in (+1, -1):
        g.append(box((0.075, 0.025, 0.003), (0.045, s * 0.055, 0.009), PLATE_WHITE, collide=True))   # fork strips
        g.append(box((0.075, 0.0025, 0.010), (0.045, s * 0.0775, 0.022), PLATE_WHITE, collide=True))  # outer side rails
    # NOTE: no front lips. Anything standing above the strips at the front is the
    # LEADING edge as the tray slides in, and it hits the apple's equator.
    g.append(box((0.003, 0.08, 0.015), (-0.077, 0, 0.027), PLATE_WHITE, collide=True))  # back wall
    ixx, iyy, izz = box_inertia(0.5, 0.20, 0.16, 0.03)
    return f"""
    <body name="tray_link" pos="0 0 {TRAY_JOINT_Z}">
      <inertial pos="0.02 0 0.01" mass="0.5" diaginertia="{f((ixx, iyy, izz))}"/>
      <joint name="tray_joint" type="slide" axis="1 0 0" range="0 {TRAY_TRAVEL}" damping="40"/>
      {chr(10).join('      ' + x for x in g)}
    </body>"""


def station_xml(sx, sy):
    S, h, T = SLAB_HALF, HOLE_HALF, SLAB_T
    slab_z = STATION_TOP - T / 2.0
    gray = (0.65, 0.67, 0.70, 1)
    g = [
        box((S, (S - h) / 2, T / 2), (sx, sy + (S + h) / 2, slab_z), gray, collide=True),
        box((S, (S - h) / 2, T / 2), (sx, sy - (S + h) / 2, slab_z), gray, collide=True),
        box(((S - h) / 2, h, T / 2), (sx + (S + h) / 2, sy, slab_z), gray, collide=True),
        box(((S - h) / 2, h, T / 2), (sx - (S + h) / 2, sy, slab_z), gray, collide=True),
    ]
    leg_len = STATION_TOP - T
    off = S - 0.012
    for ox in (+1, -1):
        for oy in (+1, -1):
            g.append(cyl(0.01, leg_len / 2, (sx + ox * off, sy + oy * off, leg_len / 2),
                         (0.25, 0.25, 0.28, 1), collide=True))

    cyl_center = CYL_TOP - CYL_LEN / 2.0
    m = 0.1
    ixx = m / 12 * (3 * CYL_R ** 2 + CYL_LEN ** 2)
    izz = 0.5 * m * CYL_R ** 2
    plunger = f"""
    <body name="plunger" pos="{f((sx, sy, cyl_center))}">
      <inertial pos="0 0 0" mass="{m}" diaginertia="{f((ixx, ixx, izz))}"/>
      <joint name="plunger_joint" type="slide" axis="0 0 1" range="{-CYL_TRAVEL} 0" damping="100"/>
      {cyl(CYL_R, CYL_LEN / 2, (0, 0, 0), ORANGE, collide=True)}
    </body>"""
    return "\n    ".join(g) + plunger


def apple_xml(sx, sy):
    i = 0.4 * APPLE_MASS * APPLE_R ** 2
    z = CYL_TOP + APPLE_R + 0.002
    return f"""
    <body name="apple" pos="{f((sx, sy, z))}">
      <freejoint name="apple_free"/>
      <inertial pos="0 0 0" mass="{APPLE_MASS}" diaginertia="{f((i, i, i))}"/>
      <geom type="sphere" size="{APPLE_R}" rgba="0.85 0.08 0.08 1" contype="1" conaffinity="1"
            condim="6" friction="1 0.01 0.005"/>
      {cyl(0.003, 0.008, (0, 0, APPLE_R + 0.002), (0.35, 0.2, 0.08, 1))}
      {box((0.01, 0.005, 0.001), (0.009, 0, APPLE_R + 0.008), (0.1, 0.55, 0.15, 1),
           quat=rpy_quat(0, 0.5, 0))}
    </body>"""


# --------------------------------------------------------------------------
def build_xml(station_x=1.5, station_y=0.0):
    wheel_blocks = "".join(hub_and_wheel(n, a) for n, a in WHEELS)
    base_geoms = "\n      ".join(robot_static_geoms())
    wheel_acts = "\n    ".join(
        f'<velocity name="act_{n}_wheel" joint="{n}_wheel_joint" kv="0.5"/>' for n, _ in WHEELS)

    return f"""<mujoco model="kiwi_pickup">
  <compiler angle="degree"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>

  <visual>
    <global offwidth="1280" offheight="720"/>
    <quality shadowsize="4096"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.35 0.55 0.75" rgb2="0.02 0.02 0.05" width="512" height="512"/>
    <texture type="2d" name="grid" builtin="checker" rgb1="0.22 0.30 0.38" rgb2="0.14 0.21 0.28"
             width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="6 6" reflectance="0.1"/>
  </asset>

  <worldbody>
    <light pos="1 -1 3" dir="0 0.3 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="floor" type="plane" size="10 10 0.05" material="grid" friction="1 0.005 0.0001"/>

    <body name="cam_target_wide" pos="0.9 0 0.2"/>
    <body name="cam_target_close" pos="{station_x - 0.2} {station_y} 0.38"/>
    <camera name="wide" pos="0.5 -2.6 1.4" mode="targetbody" target="cam_target_wide"/>
    <camera name="close" pos="{station_x - 0.5} {station_y - 0.75} 0.65" mode="targetbody" target="cam_target_close"/>

    <!-- ===================== STATION: table + cylinder ===================== -->
    {station_xml(station_x, station_y)}

    <!-- ===================== APPLE ===================== -->
    {apple_xml(station_x, station_y)}

    <!-- ===================== ROBOT ===================== -->
    <body name="base_link" pos="0 0 {BASE_LINK_Z}">
      <joint name="base_x" type="slide" axis="1 0 0"/>
      <joint name="base_y" type="slide" axis="0 1 0"/>
      <joint name="base_yaw" type="hinge" axis="0 0 1"/>
      <inertial pos="0 0 0.05" mass="7.0" diaginertia="0.2 0.2 0.3"/>
      {base_geoms}
      {wheel_blocks}
      {tray_body()}
    </body>
  </worldbody>

  <actuator>
    <!-- planar holonomic base: world-frame velocity commands -->
    <velocity name="act_x"   joint="base_x"   kv="500"/>
    <velocity name="act_y"   joint="base_y"   kv="500"/>
    <velocity name="act_yaw" joint="base_yaw" kv="30"/>
    <!-- omni wheels (spin to match the base motion) -->
    {wheel_acts}
    <!-- tray slide and station cylinder -->
    <position name="act_tray"    joint="tray_joint"    kp="400"  ctrlrange="0 {TRAY_TRAVEL}"
              forcerange="-60 60"/>
    <position name="act_plunger" joint="plunger_joint" kp="4000" ctrlrange="{-CYL_TRAVEL} 0"
              forcerange="-100 100"/>
  </actuator>
</mujoco>
"""


if __name__ == "__main__":
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else "kiwi_pickup.xml"
    with open(out, "w") as fh:
        fh.write(build_xml())
    print("wrote", out)
