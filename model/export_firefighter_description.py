"""Export ``model/firefighter.blend`` into the ``firefighter_description`` package.

    blender --background model/firefighter.blend --python model/export_firefighter_description.py

Runs the LinkForge XACRO export, then rewrites the mesh references to
``package://`` URIs so the description resolves from an installed workspace.
"""
import re
import shutil
import sys
from pathlib import Path

import bpy

PACKAGE = "firefighter_description"
PACKAGE_DIR = Path(__file__).resolve().parents[1] / "src" / PACKAGE
XACRO = PACKAGE_DIR / "urdf" / "firefighter.urdf.xacro"
MESHES = PACKAGE_DIR / "meshes"


def main():
    # LinkForge writes meshes next to the XACRO, so export at the package root
    # and move the XACRO into urdf/ afterwards.
    staged = PACKAGE_DIR / XACRO.name
    shutil.rmtree(MESHES, ignore_errors=True)
    if "FINISHED" not in bpy.ops.linkforge.export_robot_model(filepath=str(staged)):
        sys.exit("LinkForge export failed - run LinkForge > Validate in Blender.")

    text = staged.read_text(encoding="utf-8")
    text, count = re.subn(r'filename="meshes[\\/]', f'filename="package://{PACKAGE}/meshes/', text)
    XACRO.parent.mkdir(parents=True, exist_ok=True)
    XACRO.write_text(text, encoding="utf-8", newline="\n")
    staged.unlink()
    print(f"Exported {XACRO} ({count} mesh references, {len(list(MESHES.glob('*')))} mesh files)")


if __name__ == "__main__":
    main()
