import xml.etree.ElementTree as ET
import sys

sys.path.insert(0, r"C:\Users\tosha\Downloads\Roboticarm\src\arm_description\urdf")
import gen_arm_urdf as g

ET.indent(g.gen_urdf())
ET.ElementTree(g.gen_urdf()).write(
    r"C:\Users\tosha\Downloads\Roboticarm\src\arm_description\urdf\arm_6dof.urdf",
    encoding="unicode", xml_declaration=True)
print("regenerated arm_6dof.urdf")
