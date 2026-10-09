"""Render preview images of ``model/firefighter.blend`` into ``model/renders/``.

    blender --background model/firefighter.blend --python model/render_firefighter_model.py

Pass ``-- cycles`` to use Cycles instead of EEVEE.
"""
import sys
from pathlib import Path

import bpy
from mathutils import Vector

OUT_DIR = Path(__file__).resolve().parent / "renders"
TARGET = Vector((0.07, 0.0, 0.11))
VIEWS = {
    "front_right": (0.62, -0.50, 0.36),
    "rear_left": (-0.52, 0.52, 0.34),
    "side": (0.07, -0.85, 0.14),
    "top": (0.071, -0.001, 0.95),
}


def main():
    scene = bpy.context.scene
    engines = [e.identifier for e in scene.render.bl_rna.properties["engine"].enum_items]
    if "cycles" in sys.argv:
        scene.render.engine = "CYCLES"
        scene.cycles.samples = 64
        scene.cycles.use_denoising = True
    else:
        scene.render.engine = next(e for e in engines if e.startswith("BLENDER_EEVEE"))
    scene.render.resolution_x, scene.render.resolution_y = 1600, 1200
    scene.render.image_settings.file_format = "PNG"

    camera = scene.camera
    for name, location in VIEWS.items():
        camera.location = location
        camera.rotation_euler = (TARGET - camera.location).to_track_quat("-Z", "Y").to_euler()
        scene.render.filepath = str(OUT_DIR / f"{name}.png")
        bpy.ops.render.render(write_still=True)
        print(f"Rendered {scene.render.filepath}")


if __name__ == "__main__":
    main()
