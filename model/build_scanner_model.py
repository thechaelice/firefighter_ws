"""Build the 3D-scanner proof-of-concept variants of the firefighter robot.

Run headless (LinkForge must be enabled, as for build_firefighter_model.py)::

    blender --background --python model/build_scanner_model.py              # both
    blender --background --python model/build_scanner_model.py -- vl53l1x
    blender --background --python model/build_scanner_model.py -- tfmini_s

Writes ``model/firefighter_scanner_<tof>.blend``. The chassis, wheels, caster
and thermal camera come unchanged from ``build_firefighter_model.py``; the dead
RPLIDAR A1 ranging head is replaced by the spinning camera rig described in
``docs/3d-scanner-poc.md``:

* The A1 base, motor, belt and turntable bearing are kept. A printed hub and
  arm bolt to the turntable (``scanner_rotor``).
* The arm carries a tilt-adjustable sensor head ``CAMERA_RADIUS`` off the spin
  axis, so each revolution moves the camera round a circle of known size: the
  metric baseline. A counterweight sits on the other end of the arm.
* The head holds a Raspberry Pi Camera Module 3 Wide and, under it, a
  single-point ToF sensor (VL53L1X or TFmini-S) looking along the camera axis.
* A magnet on the turntable rim and a hall sensor on the base give a
  once-per-revolution index.

Link origins under ``scan_head`` are given in the head's untilted pose; the
head joint gets its pitch after the objects are created, which tilts them all.
"""
import math
import sys
from pathlib import Path

import bpy
from mathutils import Vector

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_firefighter_model as ff  # noqa: E402
from build_firefighter_model import (  # noqa: E402
    T, box, circle, convex_outline, cylinder, enum_id, plate, rot, rounded_rect)

OUTPUT_DIR = Path(__file__).resolve().parent

# ---- rig dimensions (metres) ------------------------------------------------
AXIS_X = ff.LIDAR_X                 # spin axis = old A1 scan centre
BASE_Z = ff.LIDAR_BASE_Z            # A1 base plate, on the existing standoffs
ROTOR_Z = BASE_Z + 0.018            # top face of the A1 turntable
TURNTABLE_RADIUS = 0.034
HUB_RADIUS, HUB_HEIGHT = 0.022, 0.008
ARM_THICKNESS = 0.004
ARM_TOP_Z = ROTOR_Z + HUB_HEIGHT + ARM_THICKNESS

CAMERA_RADIUS = 0.10                # spin axis -> camera lens; calibrate the real value
TILT_DEG = 15.0                     # head pitch, camera looking up
CAMERA_FRONT, CAMERA_Z = 0.012, 0.010   # lens front relative to the head pivot
HEAD_X = AXIS_X + CAMERA_RADIUS - CAMERA_FRONT
HEAD_Z = ARM_TOP_Z + 0.028          # pivot high enough for the tilted head to clear the arm
HEAD_HEIGHT = 0.048

COUNTERWEIGHT_X = AXIS_X - 0.050
INDEX_ANGLE = -45.0                 # magnet / hall sensor position, degrees about the axis

# Head-local placement of each ToF: beam origin (front face) and body size.
TOF = {
    "vl53l1x": {"front": 0.0067, "z": -0.012, "width": 0.018, "mass": 0.002},
    "tfmini_s": {"front": 0.0175, "z": -0.0135, "width": 0.042, "mass": 0.005},
}

TO_X = rot("Y", 90)                 # cylinder axis Z -> X
TO_Y = rot("X", 90)                 # cylinder axis Z -> Y


def head_width(tof):
    return max(0.025, TOF[tof]["width"]) + 0.004


def make_links(tof):
    links = {k: v for k, v in ff.LINKS.items() if k != "laser"}
    head = Vector((HEAD_X, 0.0, HEAD_Z))
    spec = TOF[tof]
    links.update({
        "scanner_base": ((AXIS_X, 0.0, BASE_Z), "base_link",
                         "scanner_base_joint", "fixed", None, 0.12),
        "scanner_rotor": ((AXIS_X, 0.0, ROTOR_Z), "scanner_base",
                          "scanner_rotor_joint", "continuous", "Z", 0.07),
        "scan_head": (tuple(head), "scanner_rotor", "scan_head_joint", "fixed", None, 0.006),
        "scan_camera": (tuple(head + Vector((CAMERA_FRONT, 0.0, CAMERA_Z))), "scan_head",
                        "scan_camera_joint", "fixed", None, 0.004),
        "scan_tof": (tuple(head + Vector((spec["front"], 0.0, spec["z"]))), "scan_head",
                     "scan_tof_joint", "fixed", None, spec["mass"]),
    })
    return links


def origin(link):
    return Vector(ff.LINKS[link][0])


def on_rim(radius, z):
    a = math.radians(INDEX_ANGLE)
    return Vector((AXIS_X + radius * math.cos(a), radius * math.sin(a), z))


# ---- builders ---------------------------------------------------------------
def build_scanner_base(m):
    """What survives of the RPLIDAR A1: base plate, motor and belt, plus the hall sensor."""
    base_x, motor_x, z = AXIS_X - 0.0135, AXIS_X - 0.047, BASE_Z
    m.add("scanner_base", "black_plastic",
          plate([rounded_rect(base_x, 0, 0.097, 0.070, 0.012)], 0.004), T((0, 0, z)))
    m.add("scanner_base", "steel", cylinder(0.012, 0.020), T((motor_x, 0, z - 0.010)))
    m.add("scanner_base", "black_plastic", cylinder(0.005, 0.006, segments=24),
          T((motor_x, 0, z + 0.007)))
    belt = convex_outline(circle(AXIS_X, 0, 0.0355, steps=64) + circle(motor_x, 0, 0.0065))
    m.add("scanner_base", "rubber", plate([belt], 0.002), T((0, 0, z + 0.006)))

    post = on_rim(0.041, z + 0.0095)
    m.add("scanner_base", "black_plastic", box(0.004, 0.004, 0.011), T(post))
    m.add("scanner_base", "black_plastic", box(0.0015, 0.004, 0.003),
          T(on_rim(0.0385, z + 0.011)) @ rot("Z", INDEX_ANGLE))
    m.collide("scanner_base", "", "box", box(0.097, 0.070, 0.004), T((base_x, 0, z + 0.002)))


def build_scanner_rotor(m, tof):
    """Turntable, printed hub and arm, counterweight and the head's pivot ears."""
    width = head_width(tof)
    m.add("scanner_rotor", "black_plastic", cylinder(TURNTABLE_RADIUS, 0.014, segments=64),
          T((AXIS_X, 0, BASE_Z + 0.011)))
    m.add("scanner_rotor", "steel", cylinder(0.003, 0.002, segments=16),
          T(on_rim(TURNTABLE_RADIUS + 0.001, BASE_Z + 0.011)) @ rot("Z", INDEX_ANGLE) @ TO_X)
    m.add("scanner_rotor", "printed", cylinder(HUB_RADIUS, HUB_HEIGHT, segments=48),
          T((AXIS_X, 0, ROTOR_Z + HUB_HEIGHT / 2)))

    outline = convex_outline(circle(AXIS_X, 0, HUB_RADIUS, steps=32)
                             + circle(COUNTERWEIGHT_X, 0, 0.015, steps=32)
                             + rounded_rect(HEAD_X, 0, 0.020, width + 0.008, 0.004, steps=4))
    holes = [circle(AXIS_X + 0.036, 0, 0.008), circle(AXIS_X + 0.062, 0, 0.007)]
    m.add("scanner_rotor", "printed", plate([outline] + holes, ARM_THICKNESS),
          T((0, 0, ROTOR_Z + HUB_HEIGHT)))

    m.add("scanner_rotor", "steel", cylinder(0.012, 0.010), T((COUNTERWEIGHT_X, 0, ARM_TOP_Z + 0.005)))
    m.add("scanner_rotor", "steel", cylinder(0.004, 0.003, segments=6),
          T((COUNTERWEIGHT_X, 0, ARM_TOP_Z + 0.0115)))

    ear_height = HEAD_Z + 0.006 - ARM_TOP_Z
    for side in (1, -1):
        y = side * (width / 2 + 0.0015)
        m.add("scanner_rotor", "printed", box(0.012, 0.003, ear_height, bevel=0.001),
              T((HEAD_X, y, ARM_TOP_Z + ear_height / 2)))
        m.add("scanner_rotor", "steel", cylinder(0.003, 0.002, segments=6),
              T((HEAD_X, side * (width / 2 + 0.004), HEAD_Z)) @ TO_Y)
    m.add("scanner_rotor", "steel", cylinder(0.0015, width + 0.006, segments=12),
          T((HEAD_X, 0, HEAD_Z)) @ TO_Y)

    arm_x0, arm_x1 = COUNTERWEIGHT_X - 0.015, HEAD_X + 0.010
    m.collide("scanner_rotor", "", "cylinder", cylinder(TURNTABLE_RADIUS, 0.014, segments=32),
              T((AXIS_X, 0, BASE_Z + 0.011)))
    m.collide("scanner_rotor", "_arm", "box", box(arm_x1 - arm_x0, width + 0.008, ARM_THICKNESS),
              T(((arm_x0 + arm_x1) / 2, 0, ROTOR_Z + HUB_HEIGHT + ARM_THICKNESS / 2)))


def build_scan_head(m, tof):
    """Printed back plate the camera and ToF screw to; pivots on the rotor's ears."""
    h = origin("scan_head")
    width = head_width(tof)
    m.add("scan_head", "printed", box(0.003, width, HEAD_HEIGHT, bevel=0.001), T(h))
    for dy in (-0.0105, 0.0105):                       # camera standoffs
        for dz in (-0.0105, 0.0105):
            m.add("scan_head", "brass", cylinder(0.0012, 0.002, segments=8),
                  T(h + Vector((0.0025, dy, CAMERA_Z + dz))) @ TO_X)
    m.collide("scan_head", "", "box", box(0.003, width, HEAD_HEIGHT), T(h))


def build_scan_camera(m):
    """Raspberry Pi Camera Module 3 Wide (25 x 24 mm board); origin at the lens front."""
    c = origin("scan_camera")
    m.add("scan_camera", "pcb_green", box(0.001, 0.025, 0.024), T(c + Vector((-0.008, 0, 0))))
    m.add("scan_camera", "black_plastic", box(0.0045, 0.0085, 0.0085, bevel=0.0004),
          T(c + Vector((-0.00525, 0, 0))))
    m.add("scan_camera", "black_plastic", cylinder(0.0035, 0.003, segments=32),
          T(c + Vector((-0.0015, 0, 0))) @ TO_X)
    m.add("scan_camera", "lens", cylinder(0.0028, 0.0004, segments=32),
          T(c + Vector((-0.0002, 0, 0))) @ TO_X)
    m.collide("scan_camera", "", "box", box(0.0085, 0.025, 0.024), T(c + Vector((-0.00425, 0, 0))))


def build_vl53l1x(m):
    """VL53L1X carrier board (18 x 13 mm); origin at the sensor window."""
    f = origin("scan_tof")
    m.add("scan_tof", "pcb_blue", box(0.0016, 0.018, 0.013), T(f + Vector((-0.0024, 0, 0))))
    m.add("scan_tof", "black_plastic", box(0.0016, 0.0049, 0.0025), T(f + Vector((-0.0008, 0, 0))))
    for dy in (-0.0012, 0.0012):                       # emitter + receiver windows
        m.add("scan_tof", "lens", cylinder(0.0005, 0.0002, segments=12),
              T(f + Vector((0.0001, dy, 0))) @ TO_X)
    for dy in (-0.007, 0.007):
        m.add("scan_tof", "brass", cylinder(0.0012, 0.002, segments=8),
              T(f + Vector((-0.0042, dy, 0.0045))) @ TO_X)
    for i in range(6):                                 # header pins
        m.add("scan_tof", "brass", box(0.0006, 0.0006, 0.002),
              T(f + Vector((-0.0024, -0.00635 + 0.00254 * i, -0.0075))))
    m.collide("scan_tof", "", "box", box(0.0032, 0.018, 0.013), T(f + Vector((-0.0016, 0, 0))))


def build_tfmini_s(m):
    """Benewake TFmini-S (42 x 15 x 16 mm over its flanges); origin between the lenses."""
    f = origin("scan_tof")
    m.add("scan_tof", "black_plastic", box(0.016, 0.030, 0.015, bevel=0.001),
          T(f + Vector((-0.008, 0, 0))))
    for side in (1, -1):
        m.add("scan_tof", "black_plastic", box(0.003, 0.006, 0.015, bevel=0.0005),
              T(f + Vector((-0.0145, side * 0.018, 0))))
        m.add("scan_tof", "steel", cylinder(0.0015, 0.001, segments=12),
              T(f + Vector((-0.0125, side * 0.018, 0))) @ TO_X)
        m.add("scan_tof", "lens", cylinder(0.0045, 0.0004, segments=32),
              T(f + Vector((0.0002, side * 0.0075, 0))) @ TO_X)
    m.collide("scan_tof", "", "box", box(0.016, 0.042, 0.015), T(f + Vector((-0.008, 0, 0))))


TOF_BUILDERS = {"vl53l1x": build_vl53l1x, "tfmini_s": build_tfmini_s}


# ---- main -------------------------------------------------------------------
def build(tof):
    bpy.ops.wm.read_homefile(use_empty=True)
    scene = bpy.context.scene
    if not hasattr(scene, "linkforge_robot"):
        sys.exit("LinkForge is not enabled: enable it in Preferences > Get Extensions.")
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0

    ff.LINKS = make_links(tof)
    ff.JOINT_LIMITS["scanner_rotor_joint"] = (0.2, 6.3)
    ff.MATERIALS["printed"] = ((0.85, 0.30, 0.06, 1.0), 0.0, 0.55)

    robot_collection = bpy.data.collections.new("Robot")
    stage_collection = bpy.data.collections.new("Stage")
    scene.collection.children.link(robot_collection)
    scene.collection.children.link(stage_collection)

    model = ff.Model()
    ff.build_chassis(model)
    ff.build_wheel(model, "left_wheel", 1)
    ff.build_wheel(model, "right_wheel", -1)
    ff.build_caster(model)
    ff.build_thermal_camera(model)
    build_scanner_base(model)
    build_scanner_rotor(model, tof)
    build_scan_head(model, tof)
    build_scan_camera(model)
    TOF_BUILDERS[tof](model)
    ff.create_objects(model, robot_collection)
    ff.create_stage(stage_collection)
    bpy.data.objects["scan_head_joint"].rotation_euler = (0.0, math.radians(-TILT_DEG), 0.0)

    robot = scene.linkforge_robot
    robot.robot_name = "firefighter_scanner"
    robot.export_format = "XACRO"
    robot.mesh_format = enum_id(robot, "mesh_format", "STL")
    robot.use_ros2_control = False

    bpy.context.view_layer.update()
    output = OUTPUT_DIR / f"firefighter_scanner_{tof}.blend"
    bpy.ops.wm.save_as_mainfile(filepath=str(output))
    print(f"Saved {output}")


def main():
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    unknown = [a for a in args if a not in TOF]
    if unknown:
        sys.exit(f"Unknown ToF variant {unknown}; choose from {sorted(TOF)}.")
    for tof in args or TOF:
        build(tof)


if __name__ == "__main__":
    main()
