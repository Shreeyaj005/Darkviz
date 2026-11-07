#!/usr/bin/env python3
"""
kiwi_mujoco_model.py - builds the MuJoCo (MJCF) model of the kiwi robot and a
transparent grocery shelf with a star-wheel dispenser.

Pure Python (no mujoco import needed to generate the XML).

    python3 kiwi_mujoco_model.py            # writes kiwi_pickup.xml
    python3 -m mujoco.viewer --mjcf=kiwi_pickup.xml

HOW THE SHELF WORKS
  * Apples are stacked single-file in a vertical hopper above a 5-spoke star wheel.
  * The lowest apple sits in a pocket (V) between two spokes.
  * The robot's tray extends under the wheel. A hinged TAB at the tray tip pushes
    the lowest spoke forward, turning the wheel one step (72 deg).
  * The pocket tips outward/downward past the horizontal, the apple rolls off the
    spoke and drops onto the tray. The next apple falls into the new pocket.
  * Tray retracts: the tab is one-way (folds flat under the spoke), and the wheel
    has a ratchet (cannot turn back) and a detent (snaps to each 72 deg step).
    Ratchet + detent are applied by the demo script each simulation step.

Modelling notes
  * The base moves on a planar holonomic joint set (slide x, slide y, hinge yaw)
    driven by velocity actuators. The omni wheels spin with the exact kinematics of
    the motion (visual) but do not touch the floor.
  * Tray, tab, star wheel and apples use full contact physics.
"""
import math

# --------------------------------------------------------------------------
# Robot geometry (mirrors kiwi_robot_urdf.xacro). Heights above ground, metres.
# --------------------------------------------------------------------------
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

# Tray: solid floor spanning local x -0.08..0.12, hinged pusher tab at the tip
TAB_H = 0.04                # tab height above the tray floor
TAB_HALF_W = 0.045          # tab half width (must fit between the shelf side plates)
TAB_PIVOT_X = 0.12          # tab hinge, local to tray_link

# Omni roller layout (two rows flush with the wheel faces, ends inside the r circle)
ROLLERS_PER_ROW = 16
ROLLER_R = WHEEL_WIDTH / 4.0
ROLLER_HALF_LEN = 0.010
ROLLER_RING_R = math.sqrt(WHEEL_RADIUS ** 2 - ROLLER_HALF_LEN ** 2) - ROLLER_R

WHEELS = [("front", 90.0), ("rear_left", 210.0), ("rear_right", -30.0)]

# --------------------------------------------------------------------------
# Item + star-wheel shelf geometry
# --------------------------------------------------------------------------
APPLE_R = 0.035
APPLE_MASS = 0.15
N_APPLES = 3

STAR_SPOKES = 5
STAR_PITCH = 2 * math.pi / STAR_SPOKES            # 72 deg per step
STAR_SPOKE_LEN = 0.060                            # hub centre -> tip
STAR_SPOKE_HT = 0.004                             # half thickness
STAR_SPOKE_HW = 0.040                             # half width (along the wheel axle, y)
STAR_HUB_R = 0.012
# apple centre distance from the axle when seated between two spokes
POCKET_R = (APPLE_R + STAR_SPOKE_HT) / math.sin(STAR_PITCH / 2)
LOAD_BISECTOR = math.radians(90)                  # loaded pocket points straight up at rest
STAR_PHASE = math.radians(54)                     # spokes at rest: 54,126,198,270(down),342 deg
# the lowest spoke hangs straight down at rest; its tip sits 14 mm above the tray floor
STAR_ZW = TRAY_FLOOR_TOP + 0.014 + STAR_SPOKE_LEN
STAR_EPS = 0.05                                   # rad of play in the ratchet window
DETENT_K = 0.25                                   # N m, detent torque amplitude
STAR_DAMPING = 0.03

# hopper / shelf (all relative to the wheel axle)
CH_XC = POCKET_R * math.cos(LOAD_BISECTOR)        # channel centre (x)
CH_HALF = 0.043                                   # channel inner half width
RIGHT_WALL_BOTTOM = 0.066                         # back wall: clears the spoke tips
LEFT_WALL_BOTTOM = 0.112                          # front wall: clears the apple leaving
HOPPER_H = 0.31
PLATE_Y = 0.049                                   # side plates (transparent), centre |y|

# approach: tab front face meets the hanging spoke at tray travel ~0.176 m; the stroke
# ends 23 mm after the spoke has slipped past the tab
ROBOT_STANDOFF = 0.30                             # shelf axle x - robot x
TRAY_PUSH_END = 0.255                             # tray travel for the pickup

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
GLASS = (0.70, 0.85, 1.0, 0.22)
SHELF_GRAY = (0.55, 0.57, 0.60, 1)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
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


def geom(kind, size, pos=(0, 0, 0), quat=None, rgba=SILVER, collide=False, extra=""):
    """One <geom>. Visual-only unless collide=True."""
    q = f' quat="{f(quat)}"' if quat is not None else ""
    if collide:
        phys = ' contype="1" conaffinity="1" friction="1 0.005 0.0001"'
    else:
        phys = ' contype="0" conaffinity="0"'
    return (f'<geom type="{kind}" size="{f(size)}" pos="{f(pos)}"{q} '
            f'rgba="{f(rgba)}"{phys}{extra}/>')


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


# --------------------------------------------------------------------------
# robot pieces
# --------------------------------------------------------------------------
def robot_static_geoms():
    g = []
    g += hex_plate(PLATE_R, BASE_T, 0.0, PLATE_WHITE)
    g += hex_plate(PLATE_R, MID_T, MID_Z, PLATE_WHITE)
    g += hex_plate(PLATE_R, TOP_T, TOP_Z, PLATE_WHITE)
    for ang in (30, 150, 270):
        a = math.radians(ang)
        g.append(cyl(0.006, 0.11, (0.15 * math.cos(a), 0.15 * math.sin(a), 0.116), SILVER))
    g.append(box((0.08, 0.055, 0.025), (0, 0, BASE_T / 2 + 0.025), ACRYLIC))
    g.append(box((0.06, 0.035, 0.002), (0, 0, BASE_T / 2 + 0.006), PCB))
    g.append(box((0.026, 0.014, 0.003), (0, 0, BASE_T / 2 + 0.011), STEPPER))
    lidar_z = MID_Z + MID_T / 2 + 0.04
    g.append(cyl(0.07, 0.04, (0, 0, lidar_z), LIDAR))
    g.append(box((0.06, 0.035, 0.02), (0, 0, TOP_Z - TOP_T / 2 - 0.02), DARK))
    for s in (+1, -1):
        g.append(box((0.025, 0.025, 0.0225), (-0.12, s * 0.106, TOP_SURFACE_Z + 0.0225), STEPPER))
        g.append(cyl(0.012, 0.007, (-0.12, s * 0.106, TOP_SURFACE_Z + 0.052), SILVER))
        g.append(cyl(0.004, 0.026, (0.15, s * 0.106, TOP_SURFACE_Z + 0.026), SILVER))
        g.append(cyl(0.012, 0.007, (0.15, s * 0.106, TOP_SURFACE_Z + 0.059), SILVER))
        for off in (0.118, 0.094):
            g.append(box((0.135, 0.0015, 0.005), (0.015, s * off, TOP_SURFACE_Z + 0.059), ROLLER))
    g.append(box((0.22, 0.012, RAIL_H / 2), (0.07, 0, TOP_SURFACE_Z + RAIL_H / 2), SILVER))
    return g


def hub_and_wheel(name, angle_deg):
    a = math.radians(angle_deg)
    q = rpy_quat(math.pi / 2, 0, a + math.pi / 2)     # local +z = radial outward
    ca, sa = math.cos(a), math.sin(a)
    z = -AXLE_DROP

    mr = 0.1475
    hub = f"""
    <body name="{name}_hub" pos="{f((mr * ca, mr * sa, z))}" quat="{f(q)}">
      <inertial pos="0 0 0" mass="0.25" diaginertia="2e-4 2e-4 1e-4"/>
      {cyl(0.025, 0.0375, (0, 0, 0), MOTOR_GRAY)}
      {box((0.025, 0.025, 0.015), (0, 0, 0.0525), SILVER)}
      {cyl(0.02, 0.01, (0, 0, -0.0475), STEPPER)}
    </body>"""

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
    """Solid tray on a prismatic joint along +x, with a one-way hinged pusher tab."""
    g = []
    g.append(box((0.035, 0.025, 0.006), (-0.05, 0, 0), DARK))                              # carriage
    g.append(box((0.115, 0.08, 0.003), (0.005, 0, 0.009), PLATE_WHITE, collide=True))      # floor
    g.append(box((0.003, 0.08, 0.0225), (-0.107, 0, 0.012 + 0.0225), PLATE_WHITE, collide=True))  # back wall
    for s in (+1, -1):
        g.append(box((0.115, 0.0025, 0.015), (0.005, s * 0.0775, 0.027), PLATE_WHITE, collide=True))  # side rails
    ixx, iyy, izz = box_inertia(0.5, 0.20, 0.16, 0.03)

    # pusher tab: hinged at the tip of the tray floor.
    #   - rigid against the push direction (joint stop at 0)
    #   - folds forward (flat) when pushed from the front, spring returns it upright
    th = TAB_H
    mt = 0.02
    tx, ty, tz = box_inertia(mt, 0.006, 2 * TAB_HALF_W, th)
    tab = f"""
      <body name="tray_tab" pos="{TAB_PIVOT_X} 0 0.012">
        <inertial pos="-0.003 0 {th / 2}" mass="{mt}" diaginertia="{f((tx, ty, tz))}"/>
        <joint name="tab_joint" type="hinge" axis="0 1 0" range="0 1.5" stiffness="0.08"
               damping="0.01" armature="5e-5"/>
        {box((0.003, TAB_HALF_W, th / 2), (-0.003, 0, th / 2), ORANGE, collide=True)}
      </body>"""
    return f"""
    <body name="tray_link" pos="0 0 {TRAY_JOINT_Z}">
      <inertial pos="0.02 0 0.01" mass="0.5" diaginertia="{f((ixx, iyy, izz))}"/>
      <joint name="tray_joint" type="slide" axis="1 0 0" range="0 {TRAY_TRAVEL}" damping="40"/>
      {chr(10).join('      ' + x for x in g)}{tab}
    </body>"""


# --------------------------------------------------------------------------
# the shelf: transparent hopper + star wheel
# --------------------------------------------------------------------------
def shelf_xml(sx, sy):
    zw = STAR_ZW
    top = zw + HOPPER_H
    z_lo = zw - 0.019                                   # side plates end above tray rails + tab
    g = []

    def panel(half, pos):
        return box(half, pos, GLASS, collide=True)

    # side plates (transparent) - tab and spokes live between them
    for s in (+1, -1):
        g.append(panel((0.11, 0.002, (top - z_lo) / 2), (sx + 0.01, sy + s * PLATE_Y, (top + z_lo) / 2)))
    # hopper channel walls
    xcc = sx + CH_XC
    rw_x = xcc + CH_HALF + 0.002
    lw_x = xcc - CH_HALF - 0.002
    rb, lb = zw + RIGHT_WALL_BOTTOM, zw + LEFT_WALL_BOTTOM
    g.append(panel((0.002, PLATE_Y - 0.002, (top - rb) / 2), (rw_x, sy, (top + rb) / 2)))
    g.append(panel((0.002, PLATE_Y - 0.002, (top - lb) / 2), (lw_x, sy, (top + lb) / 2)))
    # back panel, post and foot
    g.append(panel((0.002, PLATE_Y + 0.002, (top - z_lo) / 2), (sx + 0.12, sy, (top + z_lo) / 2)))
    g.append(box((0.01, 0.05, z_lo / 2), (sx + 0.11, sy, z_lo / 2), SHELF_GRAY, collide=True))
    g.append(box((0.10, 0.10, 0.01), (sx + 0.10, sy, 0.01), SHELF_GRAY, collide=True))
    # flared funnel lips at the top (visual only, like the sketch)
    for sgn, wx in ((-1, lw_x), (+1, rw_x)):
        b = 0.6 * sgn
        c = (wx + 0.03 * math.sin(b), sy, top + 0.03 * math.cos(b))
        g.append(box((0.002, PLATE_Y - 0.002, 0.03), c, GLASS, quat=rpy_quat(0, b, 0)))

    # ----- star wheel -----
    spokes = []
    for k in range(STAR_SPOKES):
        phi = STAR_PHASE + k * STAR_PITCH              # rest angles 54,126,198,270,342 deg
        rc = (STAR_HUB_R + STAR_SPOKE_LEN) / 2.0
        half_len = (STAR_SPOKE_LEN - STAR_HUB_R) / 2.0
        spokes.append(geom("box", (half_len, STAR_SPOKE_HW, STAR_SPOKE_HT),
                           (rc * math.cos(phi), 0, rc * math.sin(phi)),
                           rpy_quat(0, -phi, 0), ORANGE, collide=True))
    axle_q = rpy_quat(math.pi / 2, 0, 0)               # cylinder axis -> y
    wheel = f"""
    <body name="star_wheel" pos="{f((sx, sy, zw))}">
      <inertial pos="0 0 0" mass="0.25" diaginertia="2e-4 3.2e-4 2e-4"/>
      <joint name="star_joint" type="hinge" axis="0 -1 0" range="{-STAR_EPS} {STAR_PITCH + STAR_EPS}"
             damping="{STAR_DAMPING}" armature="3e-4"/>
      {cyl(0.006, PLATE_Y + 0.004, (0, 0, 0), SILVER, quat=axle_q)}
      {cyl(STAR_HUB_R, 0.03, (0, 0, 0), SILVER, quat=axle_q)}
      {chr(10).join('      ' + s for s in spokes)}
    </body>"""
    return "\n    ".join(g) + wheel


def apples_xml(sx, sy, zw=None):
    zw = STAR_ZW if zw is None else zw
    i = 0.4 * APPLE_MASS * APPLE_R ** 2
    out = []
    for k in range(N_APPLES):
        x = sx + CH_XC + (0.0015 if k % 2 else -0.0015) * (k > 0)
        z = zw + POCKET_R * math.sin(LOAD_BISECTOR) + k * (2 * APPLE_R + 0.002) + 0.001
        out.append(f"""
    <body name="apple_{k + 1}" pos="{f((x, sy, z))}">
      <freejoint name="apple_{k + 1}_free"/>
      <inertial pos="0 0 0" mass="{APPLE_MASS}" diaginertia="{f((i, i, i))}"/>
      <geom type="sphere" size="{APPLE_R}" rgba="0.85 0.08 0.08 1" contype="1" conaffinity="1"
            condim="6" friction="1 0.01 0.003"/>
      {cyl(0.003, 0.008, (0, 0, APPLE_R + 0.002), (0.35, 0.2, 0.08, 1))}
      {box((0.01, 0.005, 0.001), (0.009, 0, APPLE_R + 0.008), (0.1, 0.55, 0.15, 1),
           quat=rpy_quat(0, 0.5, 0))}
    </body>""")
    return "".join(out)


# --------------------------------------------------------------------------
def build_xml(station_x=1.5, station_y=0.0):
    wheel_blocks = "".join(hub_and_wheel(n, a) for n, a in WHEELS)
    base_geoms = "\n      ".join(robot_static_geoms())
    wheel_acts = "\n    ".join(
        f'<velocity name="act_{n}_wheel" joint="{n}_wheel_joint" kv="0.5"/>' for n, _ in WHEELS)

    return f"""<mujoco model="kiwi_pickup">
  <compiler angle="radian"/>
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

    <body name="cam_target_wide" pos="{station_x * 0.6} 0 0.3"/>
    <body name="cam_target_close" pos="{station_x - 0.08} {station_y} {STAR_ZW}"/>
    <camera name="wide" pos="0.4 -2.8 1.5" mode="targetbody" target="cam_target_wide"/>
    <camera name="close" pos="{station_x - 0.55} {station_y - 0.75} 0.8" mode="targetbody" target="cam_target_close"/>

    <!-- ===================== SHELF: hopper + star wheel ===================== -->
    {shelf_xml(station_x, station_y)}

    <!-- ===================== APPLES (stacked in the hopper) ===================== -->
    {apples_xml(station_x, station_y)}

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
    <velocity name="act_x"   joint="base_x"   kv="500"/>
    <velocity name="act_y"   joint="base_y"   kv="500"/>
    <velocity name="act_yaw" joint="base_yaw" kv="30"/>
    {wheel_acts}
    <position name="act_tray" joint="tray_joint" kp="400" ctrlrange="0 {TRAY_TRAVEL}"
              forcerange="-60 60"/>
  </actuator>
</mujoco>
"""


if __name__ == "__main__":
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else "kiwi_pickup.xml"
    with open(out, "w") as fh:
        fh.write(build_xml())
    print("wrote", out)
