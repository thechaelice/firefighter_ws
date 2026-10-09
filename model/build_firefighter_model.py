"""Build the clean firefighter robot model, rigged for LinkForge URDF/XACRO export.

Run headless (LinkForge must be enabled in the Blender user preferences)::

    blender --background --python model/build_firefighter_model.py

Writes ``model/firefighter.blend``. Open it, then LinkForge > Export to produce
``firefighter.urdf.xacro``.

Frames follow REP-103: X forward, Y left, Z up. The world origin sits on the
ground under the drive axle; ``base_link`` is the midpoint of the drive axle.
The drive wheels are at the rear and the swivel caster at the front.

Every link is an Empty carrying the LinkForge link properties. Its children
are ``<link>_visual_<material>`` meshes (one per material, so URDF colours
survive an STL export) and primitive ``<link>_collision*`` shapes.
"""
import math
import sys
from pathlib import Path

import bmesh
import bpy
from mathutils import Matrix, Vector
from mathutils.geometry import delaunay_2d_cdt

OUTPUT = Path(__file__).resolve().parent / "firefighter.blend"

# ---- dimensions (metres) ----------------------------------------------------
# Wheel radius and track MUST match wheel_radius_m / track_width_m in
# src/firefighter_bridge/config/bridge.yaml. Everything else is estimated from
# photos of the robot - measure and correct.
WHEEL_RADIUS = 0.0325
WHEEL_WIDTH = 0.026
TRACK = 0.20

BOX_LENGTH, BOX_WIDTH = 0.23, 0.16
BOX_CENTER_X = 0.07                 # box is offset forward of the drive axle
BOX_BOTTOM_Z = 0.046
BOX_BODY_HEIGHT = 0.060
LID_HEIGHT = 0.014
LID_TOP_Z = BOX_BOTTOM_Z + BOX_BODY_HEIGHT - 0.002 + LID_HEIGHT

DECK_STANDOFF = 0.045
DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS = 0.25, 0.15, 0.003
DECK_CENTER_X = 0.075
DECK_BOTTOM_Z = LID_TOP_Z + DECK_STANDOFF
DECK_TOP_Z = DECK_BOTTOM_Z + DECK_THICKNESS

LIDAR_X = 0.09                      # RPLIDAR A1 turret (scan) centre
LIDAR_STANDOFF = 0.025
LIDAR_BASE_Z = DECK_TOP_Z + LIDAR_STANDOFF
LIDAR_SCAN_Z = LIDAR_BASE_Z + 0.0325

THERMAL_Z = DECK_TOP_Z + 0.013
THERMAL_X = 0.2046                  # front face of the MLX90641 can

CASTER_X = 0.155
CASTER_TRAIL = 0.012
CASTER_WHEEL_RADIUS = 0.0125
CASTER_SWIVEL_Z = BOX_BOTTOM_Z - 0.002

# name: (world origin, parent link, joint name, joint type, axis, mass kg)
LINKS = {
    "base_link": ((0.0, 0.0, WHEEL_RADIUS), None, None, None, None, 0.9),
    "left_wheel": ((0.0, TRACK / 2, WHEEL_RADIUS), "base_link",
                   "left_wheel_joint", "continuous", "Y", 0.04),
    "right_wheel": ((0.0, -TRACK / 2, WHEEL_RADIUS), "base_link",
                    "right_wheel_joint", "continuous", "Y", 0.04),
    "caster_swivel": ((CASTER_X, 0.0, CASTER_SWIVEL_Z), "base_link",
                      "caster_swivel_joint", "continuous", "Z", 0.015),
    "caster_wheel": ((CASTER_X - CASTER_TRAIL, 0.0, CASTER_WHEEL_RADIUS), "caster_swivel",
                     "caster_wheel_joint", "continuous", "Y", 0.008),
    # Frame names match the static transforms in firefighter_bringup.
    "laser": ((LIDAR_X, 0.0, LIDAR_SCAN_Z), "base_link",
              "laser_joint", "fixed", None, 0.17),
    "thermal_camera": ((THERMAL_X, 0.0, THERMAL_Z), "base_link",
                       "thermal_camera_joint", "fixed", None, 0.005),
}

# name: (rgba, metallic, roughness)
MATERIALS = {
    "clear_plastic": ((0.80, 0.88, 0.92, 0.45), 0.0, 0.15),
    "latch_blue": ((0.08, 0.16, 0.70, 1.0), 0.0, 0.40),
    "aluminium": ((0.78, 0.79, 0.80, 1.0), 1.0, 0.35),
    "steel": ((0.55, 0.56, 0.58, 1.0), 1.0, 0.30),
    "brass": ((0.72, 0.56, 0.22, 1.0), 1.0, 0.35),
    "black_plastic": ((0.03, 0.03, 0.035, 1.0), 0.0, 0.45),
    "rubber": ((0.012, 0.012, 0.012, 1.0), 0.0, 0.90),
    "rim_blue": ((0.06, 0.42, 0.72, 1.0), 0.9, 0.25),
    "lens": ((0.02, 0.02, 0.03, 1.0), 0.0, 0.05),
    "pcb_blue": ((0.03, 0.16, 0.50, 1.0), 0.0, 0.50),
    "pcb_green": ((0.03, 0.32, 0.10, 1.0), 0.0, 0.50),
    "pcb_red": ((0.55, 0.04, 0.04, 1.0), 0.0, 0.50),
    "battery": ((0.07, 0.09, 0.20, 1.0), 0.0, 0.60),
    "nylon": ((0.20, 0.20, 0.21, 1.0), 0.0, 0.60),
}

T = Matrix.Translation
TAU = 2.0 * math.pi


def rot(axis, degrees):
    return Matrix.Rotation(math.radians(degrees), 4, axis)


# ---- mesh primitives (each returns a fresh bmesh centred on its origin) -----
def box(sx, sy, sz, bevel=0.0, segments=3, vertical_only=False):
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    bmesh.ops.scale(bm, vec=(sx, sy, sz), verts=bm.verts)
    if bevel > 0.0:
        edges = [e for e in bm.edges
                 if not vertical_only or abs(e.verts[0].co.z - e.verts[1].co.z) > 1e-9]
        bmesh.ops.bevel(bm, geom=edges, offset=bevel, segments=segments,
                        profile=0.5, affect="EDGES")
    return bm


def cylinder(radius, height, segments=40):
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segments,
                          radius1=radius, radius2=radius, depth=height)
    return bm


def lathe(profile, segments=56, closed=False):
    """Revolve an (r, z) profile about Z. ``closed`` joins last point to first."""
    bm = bmesh.new()
    verts = [bm.verts.new((r, 0.0, z)) for r, z in profile]
    pairs = list(zip(verts, verts[1:]))
    if closed:
        pairs.append((verts[-1], verts[0]))
    edges = [bm.edges.new(pair) for pair in pairs]
    bmesh.ops.spin(bm, geom=verts + edges, cent=(0, 0, 0), axis=(0, 0, 1),
                   angle=TAU, steps=segments, use_duplicate=False)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-6)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def rounded_rect(cx, cy, sx, sy, radius, steps=8):
    radius = min(radius, sx / 2, sy / 2)
    points = []
    corners = ((1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270))
    for dx, dy, start in corners:
        ox, oy = cx + dx * (sx / 2 - radius), cy + dy * (sy / 2 - radius)
        for i in range(steps + 1):
            a = math.radians(start + 90.0 * i / steps)
            points.append((ox + radius * math.cos(a), oy + radius * math.sin(a)))
    return points


def circle(cx, cy, radius, steps=20):
    return [(cx + radius * math.cos(TAU * i / steps), cy + radius * math.sin(TAU * i / steps))
            for i in range(steps)]


def plate(loops, thickness):
    """Extrude a 2D outline (first loop) with holes (remaining loops) along +Z."""
    points, faces = [], []
    for loop in loops:
        faces.append(list(range(len(points), len(points) + len(loop))))
        points += [Vector(p) for p in loop]
    # Output type 2: triangles inside the outline, with nested loops as holes.
    points, _, triangles, *_ = delaunay_2d_cdt(points, [], faces, 2, 1e-7)

    bm = bmesh.new()
    bottom = [bm.verts.new((p.x, p.y, 0.0)) for p in points]
    top = [bm.verts.new((p.x, p.y, thickness)) for p in points]
    uses = {}
    for tri in triangles:
        bm.faces.new([bottom[i] for i in reversed(tri)])
        bm.faces.new([top[i] for i in tri])
        for a, b in zip(tri, tri[1:] + tri[:1]):
            uses[(a, b)] = uses.get((a, b), 0) + 1
    for a, b in uses:
        if (b, a) not in uses:                      # boundary edge -> side wall
            bm.faces.new((bottom[a], bottom[b], top[b], top[a]))
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return bm


def convex_outline(points):
    """2D convex hull (monotone chain), counter-clockwise."""
    points = sorted(set(points))

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out[:-1]

    return half(points) + half(reversed(points))


def moved(bm, matrix):
    bmesh.ops.transform(bm, matrix=matrix, verts=bm.verts)
    return bm


# ---- scene assembly ---------------------------------------------------------
class Model:
    def __init__(self):
        self.parts = {}        # (link, material) -> bmesh in world coordinates
        self.collisions = []   # (link, suffix, kind, bmesh, world matrix)

    def add(self, link, material, bm, matrix=None):
        if matrix is not None:
            moved(bm, matrix)
        mesh = bpy.data.meshes.new("tmp")
        bm.to_mesh(mesh)
        bm.free()
        self.parts.setdefault((link, material), bmesh.new()).from_mesh(mesh)
        bpy.data.meshes.remove(mesh)

    def collide(self, link, suffix, kind, bm, matrix):
        self.collisions.append((link, suffix, kind, bm, matrix))


def build_chassis(m):
    body_z = BOX_BOTTOM_Z + BOX_BODY_HEIGHT / 2
    lid_z = LID_TOP_Z - LID_HEIGHT / 2
    m.add("base_link", "clear_plastic",
          box(BOX_LENGTH, BOX_WIDTH, BOX_BODY_HEIGHT, bevel=0.016, segments=5, vertical_only=True),
          T((BOX_CENTER_X, 0, body_z)))
    m.add("base_link", "clear_plastic",
          box(BOX_LENGTH + 0.008, BOX_WIDTH + 0.008, LID_HEIGHT, bevel=0.004),
          T((BOX_CENTER_X, 0, lid_z)))
    for x in (0.015, 0.125):                               # latches, right-hand side
        m.add("base_link", "latch_blue", box(0.030, 0.007, 0.030, bevel=0.002),
              T((x, -(BOX_WIDTH / 2 + 0.008), LID_TOP_Z - 0.018)))

    # Electronics, visible through the box.
    floor = BOX_BOTTOM_Z + 0.003
    for material, size, (x, y) in (
        ("battery", (0.105, 0.065, 0.024), (0.125, -0.020)),
        ("pcb_green", (0.085, 0.056, 0.014), (0.005, -0.035)),      # Raspberry Pi
        ("black_plastic", (0.055, 0.028, 0.010), (-0.015, 0.045)),  # mobility ESP32
        ("pcb_red", (0.043, 0.043, 0.020), (0.045, 0.045)),         # motor driver
    ):
        m.add("base_link", material, box(*size, bevel=0.002), T((x, y, floor + size[2] / 2)))

    # Drive motors, brackets and the fixed half of the caster.
    for side in (1, -1):
        to_y = rot("X", -90 * side)
        for material, radius, y0, y1 in (
            ("black_plastic", 0.0115, 0.015, 0.025),   # encoder
            ("steel", 0.0120, 0.025, 0.055),           # motor can
            ("aluminium", 0.0125, 0.055, 0.081),       # gearbox
        ):
            m.add("base_link", material, cylinder(radius, y1 - y0),
                  T((0, side * (y0 + y1) / 2, WHEEL_RADIUS)) @ to_y)
        m.add("base_link", "black_plastic", box(0.034, 0.003, 0.029),
              T((0, side * 0.0825, BOX_BOTTOM_Z - 0.0145)))
        m.add("base_link", "black_plastic", box(0.034, 0.022, 0.001),
              T((0, side * 0.073, BOX_BOTTOM_Z - 0.0005)))
    m.add("base_link", "steel", box(0.032, 0.026, 0.002, bevel=0.0005),
          T((CASTER_X, 0, BOX_BOTTOM_Z - 0.001)))

    # Upper deck on brass standoffs.
    posts = [(x, y) for x in (-0.025, 0.165) for y in (-0.060, 0.060)]
    for x, y in posts:
        m.add("base_link", "brass", cylinder(0.003, DECK_STANDOFF, segments=6),
              T((x, y, LID_TOP_Z + DECK_STANDOFF / 2)))
        m.add("base_link", "steel", cylinder(0.003, 0.002, segments=16),
              T((x, y, DECK_TOP_Z + 0.001)))
    loops = [rounded_rect(DECK_CENTER_X, 0, DECK_LENGTH, DECK_WIDTH, 0.022)]
    loops += [circle(x, y, 0.003) for x in (-0.03, 0.0, 0.03, 0.06, 0.09, 0.12)
              for y in (-0.03, 0.0, 0.03)]
    loops += [rounded_rect(x, y, 0.040, 0.007, 0.0035, steps=5)
              for x in (0.02, 0.075, 0.13) for y in (-0.052, 0.052)]
    loops += [rounded_rect(0.155, y, 0.025, 0.012, 0.002, steps=3) for y in (-0.03, 0.03)]
    loops += [circle(0.185, y, 0.003) for y in (-0.045, 0.045)]
    m.add("base_link", "aluminium", plate(loops, DECK_THICKNESS), T((0, 0, DECK_BOTTOM_Z)))

    # LiDAR USB adapter, LiDAR standoffs and the thermal camera bracket.
    m.add("base_link", "pcb_green", box(0.045, 0.018, 0.006, bevel=0.001),
          T((-0.022, 0, DECK_TOP_Z + 0.003)))
    for x in (-0.040, 0.040):
        for y in (-0.027, 0.027):
            m.add("base_link", "black_plastic", cylinder(0.003, LIDAR_STANDOFF, segments=16),
                  T((LIDAR_X - 0.0135 + x, y, DECK_TOP_Z + LIDAR_STANDOFF / 2)))
    m.add("base_link", "black_plastic", box(0.016, 0.030, 0.003),
          T((0.188, 0, DECK_TOP_Z + 0.0015)))
    m.add("base_link", "black_plastic", box(0.003, 0.030, 0.022),
          T((0.1945, 0, DECK_TOP_Z + 0.011)))

    box_height = LID_TOP_Z - BOX_BOTTOM_Z
    m.collide("base_link", "", "box", box(BOX_LENGTH + 0.008, BOX_WIDTH + 0.008, box_height),
              T((BOX_CENTER_X, 0, BOX_BOTTOM_Z + box_height / 2)))
    m.collide("base_link", "_deck", "box", box(DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS),
              T((DECK_CENTER_X, 0, DECK_BOTTOM_Z + DECK_THICKNESS / 2)))


def build_wheel(m, link, side):
    """Drive wheel modelled about +Z (outer face up), then laid onto the axle."""
    place = T(LINKS[link][0]) @ rot("X", -90 * side)
    half, r = WHEEL_WIDTH / 2, WHEEL_RADIUS
    tyre = [(r - 0.009, -half), (r - 0.0025, -half), (r, -half + 0.0025),
            (r, -0.0045), (r - 0.0012, -0.0040), (r - 0.0012, -0.0030), (r, -0.0025),
            (r, 0.0025), (r - 0.0012, 0.0030), (r - 0.0012, 0.0040), (r, 0.0045),
            (r, half - 0.0025), (r - 0.0025, half), (r - 0.009, half)]
    m.add(link, "rubber", lathe(tyre, closed=True), place)
    barrel = [(r - 0.009, -half + 0.001), (r - 0.009, half - 0.001),
              (r - 0.011, half - 0.001), (r - 0.011, -half + 0.001)]
    m.add(link, "rim_blue", lathe(barrel, closed=True), place)
    m.add(link, "rim_blue", cylinder(0.006, 0.008), place @ T((0, 0, 0.008)))
    for i in range(10):
        m.add(link, "rim_blue", box(0.017, 0.004, 0.003, bevel=0.0008, segments=2),
              place @ rot("Z", 36 * i) @ T((0.0135, 0, 0.0095)))
    m.add(link, "black_plastic", cylinder(r - 0.0105, 0.002), place @ T((0, 0, 0.004)))
    m.add(link, "steel", cylinder(0.0025, 0.003, segments=6), place @ T((0, 0, 0.013)))
    m.add(link, "brass", cylinder(0.006, 0.006), place @ T((0, 0, -half - 0.003)))
    m.collide(link, "", "cylinder", cylinder(r, WHEEL_WIDTH, segments=32), place)


def build_caster(m):
    sx, _, sz = LINKS["caster_swivel"][0]
    wx, _, wz = LINKS["caster_wheel"][0]
    to_y = rot("X", -90)
    m.add("caster_swivel", "steel", cylinder(0.011, 0.004), T((sx, 0, sz - 0.002)))
    m.add("caster_swivel", "steel", box(0.020, 0.0175, 0.0015), T((sx - 0.006, 0, sz - 0.0045)))
    fork_height = sz - 0.004 - (wz - 0.004)
    for y in (-0.008, 0.008):
        m.add("caster_swivel", "steel", box(0.016, 0.0015, fork_height, bevel=0.0005),
              T((sx - 0.008, y, wz - 0.004 + fork_height / 2)))
    m.add("caster_swivel", "steel", cylinder(0.0015, 0.019, segments=12), T((wx, 0, wz)) @ to_y)

    m.collide("caster_swivel", "", "box", box(0.020, 0.018, fork_height),
              T((sx - 0.006, 0, wz - 0.004 + fork_height / 2)))

    r, half = CASTER_WHEEL_RADIUS, 0.006
    tread = [(0.0, -half), (r - 0.0025, -half), (r, -half + 0.0025),
             (r, half - 0.0025), (r - 0.0025, half), (0.0, half)]
    m.add("caster_wheel", "nylon", lathe(tread, segments=40), T((wx, 0, wz)) @ to_y)
    sphere = bmesh.new()
    bmesh.ops.create_uvsphere(sphere, u_segments=24, v_segments=12, radius=r)
    m.collide("caster_wheel", "", "sphere", sphere, T((wx, 0, wz)))


def build_lidar(m):
    """RPLIDAR A1: link origin is the scan centre, motor towards the rear."""
    base_x, motor_x = LIDAR_X - 0.0135, LIDAR_X - 0.047
    z = LIDAR_BASE_Z
    m.add("laser", "black_plastic",
          plate([rounded_rect(base_x, 0, 0.097, 0.070, 0.012)], 0.004), T((0, 0, z)))
    m.add("laser", "steel", cylinder(0.012, 0.020), T((motor_x, 0, z - 0.010)))
    m.add("laser", "black_plastic", cylinder(0.005, 0.006, segments=24), T((motor_x, 0, z + 0.007)))
    m.add("laser", "black_plastic", cylinder(0.034, 0.014, segments=64), T((LIDAR_X, 0, z + 0.011)))
    belt = convex_outline(circle(LIDAR_X, 0, 0.0355, steps=64) + circle(motor_x, 0, 0.0065))
    m.add("laser", "rubber", plate([belt], 0.002), T((0, 0, z + 0.006)))
    head = [(0.0, 0.0), (0.0355, 0.0), (0.0355, 0.021), (0.0315, 0.025), (0.0, 0.025)]
    m.add("laser", "black_plastic", lathe(head, segments=64), T((LIDAR_X, 0, z + 0.020)))
    for y in (-0.011, 0.011):                                  # emitter + receiver
        angle = math.degrees(math.asin(y / 0.0355))
        m.add("laser", "lens", cylinder(0.0045, 0.002, segments=24),
              T((LIDAR_X, 0, LIDAR_SCAN_Z)) @ rot("Z", angle) @ T((0.0352, 0, 0)) @ rot("Y", 90))
    m.collide("laser", "", "cylinder", cylinder(0.036, 0.045, segments=32),
              T((LIDAR_X, 0, z + 0.0225)))


def build_thermal_camera(m):
    """Waveshare MLX90641 breakout, looking along +X."""
    to_x = rot("Y", 90)
    m.add("thermal_camera", "pcb_blue", box(0.0016, 0.028, 0.016),
          T((THERMAL_X - 0.0078, 0, THERMAL_Z)))
    m.add("thermal_camera", "steel", cylinder(0.0046, 0.007, segments=24),
          T((THERMAL_X - 0.0035, 0, THERMAL_Z)) @ to_x)
    m.add("thermal_camera", "lens", cylinder(0.003, 0.0004, segments=24),
          T((THERMAL_X, 0, THERMAL_Z)) @ to_x)
    m.collide("thermal_camera", "", "box", box(0.009, 0.028, 0.016),
              T((THERMAL_X - 0.0045, 0, THERMAL_Z)))


# ---- Blender objects --------------------------------------------------------
def make_material(name, rgba, metallic, roughness):
    mat = bpy.data.materials.new(name)
    if mat.node_tree is None:
        mat.use_nodes = True
    shader = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    # LinkForge reads the URDF colour (including alpha) from Base Color.
    shader.inputs["Base Color"].default_value = rgba
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Alpha"].default_value = rgba[3]
    mat.diffuse_color = rgba
    mat.metallic, mat.roughness = metallic, roughness
    if rgba[3] < 1.0 and hasattr(mat, "surface_render_method"):
        mat.surface_render_method = "BLENDED"
    return mat


def enum_id(props, prop_name, wanted):
    """Resolve an enum identifier case-insensitively."""
    for item in props.bl_rna.properties[prop_name].enum_items:
        if item.identifier.lower() == wanted.lower():
            return item.identifier
    raise ValueError(f"{prop_name} has no option {wanted!r}")


def shade(mesh):
    mesh.polygons.foreach_set("use_smooth", [True] * len(mesh.polygons))
    if hasattr(mesh, "set_sharp_from_angle"):
        mesh.set_sharp_from_angle(angle=math.radians(35))
    mesh.update()


def create_objects(m, collection):
    materials = {name: make_material(name, *spec) for name, spec in MATERIALS.items()}
    links = {}
    for name, (origin, parent, joint_name, joint_type, axis, mass) in LINKS.items():
        link = bpy.data.objects.new(name, None)
        link.empty_display_type = "PLAIN_AXES"
        link.empty_display_size = 0.02
        collection.objects.link(link)
        link.linkforge.is_robot_link = True
        link.linkforge.link_name = name
        link.linkforge.mass = mass
        link.linkforge.use_material = True
        links[name] = link
        if parent is None:
            link.location = origin
            continue
        joint = bpy.data.objects.new(joint_name, None)
        joint.empty_display_type = "ARROWS"
        joint.empty_display_size = 0.03
        collection.objects.link(joint)
        joint.parent = links[parent]
        joint.location = Vector(origin) - Vector(LINKS[parent][0])
        link.parent = joint
        props = joint.linkforge_joint
        props.is_robot_joint = True
        props.joint_name = joint_name
        props.joint_type = enum_id(props, "joint_type", joint_type)
        if axis:
            props.axis = axis
        bpy.context.view_layer.update()
        props.parent_link = links[parent]
        props.child_link = link

    for (link_name, material), bm in m.parts.items():
        moved(bm, T(-Vector(LINKS[link_name][0])))
        mesh = bpy.data.meshes.new(f"{link_name}_visual_{material}")
        bm.to_mesh(mesh)
        bm.free()
        shade(mesh)
        mesh.materials.append(materials[material])
        visual = bpy.data.objects.new(mesh.name, mesh)
        collection.objects.link(visual)
        visual.parent = links[link_name]

    for link_name, suffix, kind, bm, matrix in m.collisions:
        mesh = bpy.data.meshes.new(f"{link_name}_collision{suffix}")
        bm.to_mesh(mesh)
        bm.free()
        collision = bpy.data.objects.new(mesh.name, mesh)
        collection.objects.link(collision)
        collision.parent = links[link_name]
        collision.matrix_local = T(-Vector(LINKS[link_name][0])) @ matrix
        collision.display_type = "WIRE"
        collision.hide_render = True
        collision.hide_viewport = True     # LinkForge > Show Collisions toggles this
        geom = collision.linkforge_geom
        geom.geometry_type = enum_id(geom, "geometry_type", kind)


def create_stage(collection):
    """Ground, lights and camera for renders; ignored by the LinkForge export."""
    ground_mesh = bpy.data.meshes.new("ground")
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=30.0)
    bm.to_mesh(ground_mesh)
    bm.free()
    ground_mesh.materials.append(make_material("ground", (0.22, 0.24, 0.27, 1.0), 0.0, 0.8))
    collection.objects.link(bpy.data.objects.new("ground", ground_mesh))

    target = Vector((0.07, 0.0, 0.11))
    for name, location, power, size in (
        ("key_light", (0.55, -0.60, 0.85), 22.0, 0.6),
        ("fill_light", (0.60, 0.70, 0.45), 7.0, 0.9),
        ("rim_light", (-0.70, 0.20, 0.70), 12.0, 0.5),
    ):
        data = bpy.data.lights.new(name, "AREA")
        data.energy, data.size = power, size
        light = bpy.data.objects.new(name, data)
        light.location = location
        light.rotation_euler = (target - Vector(location)).to_track_quat("-Z", "Y").to_euler()
        collection.objects.link(light)

    camera_data = bpy.data.cameras.new("camera")
    camera_data.lens = 60.0
    camera_data.clip_start = 0.01
    camera = bpy.data.objects.new("camera", camera_data)
    camera.location = (0.62, -0.50, 0.36)
    camera.rotation_euler = (target - camera.location).to_track_quat("-Z", "Y").to_euler()
    collection.objects.link(camera)
    bpy.context.scene.camera = camera

    world = bpy.data.worlds.new("studio")
    if world.node_tree is None:
        world.use_nodes = True
    background = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
    background.inputs["Color"].default_value = (0.80, 0.83, 0.87, 1.0)
    background.inputs["Strength"].default_value = 0.40
    bpy.context.scene.world = world
    bpy.context.scene.view_settings.view_transform = "Standard"


def main():
    bpy.ops.wm.read_homefile(use_empty=True)
    scene = bpy.context.scene
    if not hasattr(scene, "linkforge_robot"):
        sys.exit("LinkForge is not enabled: enable it in Preferences > Get Extensions.")
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0

    robot_collection = bpy.data.collections.new("Robot")
    stage_collection = bpy.data.collections.new("Stage")
    scene.collection.children.link(robot_collection)
    scene.collection.children.link(stage_collection)

    model = Model()
    build_chassis(model)
    build_wheel(model, "left_wheel", 1)
    build_wheel(model, "right_wheel", -1)
    build_caster(model)
    build_lidar(model)
    build_thermal_camera(model)
    create_objects(model, robot_collection)
    create_stage(stage_collection)

    robot = scene.linkforge_robot
    robot.robot_name = "firefighter"
    robot.export_format = "XACRO"
    robot.mesh_format = enum_id(robot, "mesh_format", "STL")
    # The base is driven by the ESP32 bridge, not ros2_control.
    robot.use_ros2_control = False

    bpy.context.view_layer.update()
    bpy.ops.wm.save_as_mainfile(filepath=str(OUTPUT))
    print(f"Saved {OUTPUT}")


if __name__ == "__main__":
    main()
