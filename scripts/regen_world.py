import xml.etree.ElementTree as ET
import sys

sys.path.insert(0, r"C:\Users\tosha\Downloads\Roboticarm\src\arm_description\world")
import gen_arm_world as g

ET.indent(g.gen_sdf())
ET.ElementTree(g.gen_sdf()).write(
    r"C:\Users\tosha\Downloads\Roboticarm\src\arm_description\world\arm_world.sdf",
    encoding="unicode", xml_declaration=True)
print("regenerated arm_world.sdf")
