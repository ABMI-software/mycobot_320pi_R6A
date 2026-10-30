#!/usr/bin/env python3
"""Generate the small, visual-only joint collar used by the realistic robot.

The existing robot meshes are reused unchanged. This hollow unit collar has
radius 1 and width 1 along Z; URDF scales and places it on the mesh collars.
Its silver centre and narrow dark edges approximate the photographed joints.
"""

from math import cos, pi, sin
from pathlib import Path
from xml.etree import ElementTree as ET


def generate(destination: Path) -> None:
    ns = "http://www.collada.org/2005/11/COLLADASchema"
    ET.register_namespace("", ns)

    def element(parent, tag, text=None, **attrs):
        node = ET.SubElement(parent, f"{{{ns}}}{tag}", attrs)
        node.text = text
        return node

    root = ET.Element(f"{{{ns}}}COLLADA", version="1.4.1")
    asset = element(root, "asset")
    element(asset, "created", "2026-09-10T00:00:00Z")
    element(asset, "modified", "2026-09-10T00:00:00Z")
    element(asset, "unit", name="meter", meter="1")
    element(asset, "up_axis", "Z_UP")
    materials = element(root, "library_materials")
    effects = element(root, "library_effects")
    for name, colour, specular, shininess in (
        ("silver", "0.56 0.58 0.59 1", "0.30 0.30 0.30 1", "35"),
        ("seam", "0.18 0.19 0.19 1", "0.05 0.05 0.05 1", "8"),
    ):
        mat = element(materials, "material", id=name, name=name)
        element(mat, "instance_effect", url=f"#{name}-effect")
        effect = element(effects, "effect", id=f"{name}-effect")
        phong = element(element(element(effect, "profile_COMMON"), "technique", sid="common"), "phong")
        for kind, value in (("ambient", colour), ("diffuse", colour), ("specular", specular)):
            element(element(phong, kind), "color", value)
        element(element(phong, "shininess"), "float", shininess)

    geometries = element(root, "library_geometries")
    mesh = element(element(geometries, "geometry", id="collar", name="collar"), "mesh")
    positions, normals, groups = [], [], {"silver": [], "seam": []}
    steps = 192

    def face(points, vertex_normals, material):
        start = len(positions)
        positions.extend(points)
        normals.extend(vertex_normals)
        groups[material].extend((start, start + 1, start + 2, start, start + 2, start + 3))

    for i in range(steps):
        a, b = 2 * pi * i / steps, 2 * pi * (i + 1) / steps
        ca, sa, cb, sb = cos(a), sin(a), cos(b), sin(b)
        for low, high, material in ((-.5, -.32, "seam"), (-.32, .32, "silver"), (.32, .5, "seam")):
            face([(ca, sa, low), (cb, sb, low), (cb, sb, high), (ca, sa, high)],
                 [(ca, sa, 0), (cb, sb, 0), (cb, sb, 0), (ca, sa, 0)], material)
        # Hollow inside and end faces keep the collar closed; no disk crosses
        # the robot, and no collision or inertial geometry is generated.
        inner = .994
        face([(inner * cb, inner * sb, -.5), (inner * ca, inner * sa, -.5),
              (inner * ca, inner * sa, .5), (inner * cb, inner * sb, .5)],
             [(-cb, -sb, 0), (-ca, -sa, 0), (-ca, -sa, 0), (-cb, -sb, 0)], "seam")
        for z, sign in ((-.5, -1), (.5, 1)):
            points = [(ca, sa, z), (cb, sb, z), (inner * cb, inner * sb, z), (inner * ca, inner * sa, z)]
            if sign < 0:
                points.reverse()
            face(points, [(0, 0, sign)] * 4, "seam")

    for name, values in (("positions", positions), ("normals", normals)):
        source = element(mesh, "source", id=name)
        element(source, "float_array", " ".join(f"{v:.8g}" for row in values for v in row),
                id=f"{name}-array", count=str(len(values) * 3))
        accessor = element(element(source, "technique_common"), "accessor",
                           source=f"#{name}-array", count=str(len(values)), stride="3")
        for axis in "XYZ":
            element(accessor, "param", name=axis, type="float")
    vertices = element(mesh, "vertices", id="vertices")
    element(vertices, "input", semantic="POSITION", source="#positions")
    for material, indices in groups.items():
        triangles = element(mesh, "triangles", material=material, count=str(len(indices) // 3))
        element(triangles, "input", semantic="VERTEX", source="#vertices", offset="0")
        element(triangles, "input", semantic="NORMAL", source="#normals", offset="1")
        element(triangles, "p", " ".join(f"{i} {i}" for i in indices))
    scene = element(element(root, "library_visual_scenes"), "visual_scene", id="scene")
    instance = element(element(scene, "node", id="collar-node"), "instance_geometry", url="#collar")
    binding = element(element(instance, "bind_material"), "technique_common")
    for material in groups:
        element(binding, "instance_material", symbol=material, target=f"#{material}")
    element(element(root, "scene"), "instance_visual_scene", url="#scene")
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(destination, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    generate(Path(__file__).resolve().parents[1] / "mycobot_description/models/realistic_robot/meshes/joint_collar.dae")
