import build123d as b
from build123d import Compound, Plane, Location, Axis

asm = b.import_step(
    r"C:\Users\tosha\Downloads\Roboticarm\src\arm_description\cad\gripper_assembly.step")
bb = asm.bounding_box()
print(f"bbox: x [{bb.min.X:.1f}, {bb.max.X:.1f}] y [{bb.min.Y:.1f}, {bb.max.Y:.1f}] z [{bb.min.Z:.1f}, {bb.max.Z:.1f}]")

# Find pad inner faces: the flat faces at x = -19 and +19 (per design)
for target in (-19.0, 19.0):
    found = []
    for f in asm.faces():
        if abs(f.center().X - target) < 1.0 and abs(f.normal_at().X) > 0.9:
            c = f.center()
            found.append((round(c.X, 1), round(c.Y, 1), round(c.Z, 1)))
    print(f"pad inner face x={target}: centers {found}")

# Pad contact band: min/max Y of faces facing X
for target in (-19.0, 19.0):
    for f in asm.faces():
        if abs(f.center().X - target) < 0.5 and abs(f.normal_at().X) > 0.9:
            fb = f.bounding_box()
            print(f"face x={target}: y [{fb.min.Y:.1f}, {fb.max.Y:.1f}] z [{fb.min.Z:.1f}, {fb.max.Z:.1f}] area {f.area:.0f}")
