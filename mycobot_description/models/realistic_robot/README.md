# Realistic robot appearance

Enabled by `robot_appearance:=realistic` when expanding
`urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf`. The default `original` expansion
preserves the previous model. Existing robot and gripper DAE meshes are reused.

Only visuals change: matte grey base, satin off-white shells, silver wrist
flange, and narrow silver/dark collars. Joint definitions, link frames, mesh
origins, original mesh dimensions, collisions, inertias and controllers are
identical between appearances. This is an approximation from photographs,
not a measured material or mechanical calibration.

Gazebo receives the following material targets:

| Surface | RGB | Roughness |
| --- | --- | --- |
| Base | 0.46, 0.48, 0.47 | 0.72 |
| Shells | 0.88, 0.87, 0.84 | 0.48 |
| Wrist flange | 0.56, 0.58, 0.59 | 0.36 |
| Collar centre | 0.56, 0.58, 0.59 | Same as parent shell/flange |
| Collar edges | 0.18, 0.19, 0.19 | Same as parent shell/flange |

The SDFormat URDF converter bundled with the installed Jazzy environment
multiplies URDF RGB by 1.25; `robot_surface` compensates by 0.8. The SDF
extension supplies specular and PBR roughness, leaving diffuse/ambient to
each URDF visual. Explicit diffuse/ambient in that extension would recolour
all the collars on the same link. Colours and PBR values were checked in
the converted `gz sdf -p` output. That converter also inserts empty contact
and friction `ode` elements in collision surfaces for links with a Gazebo
extension; they contain no parameter overrides and retain the SDF defaults.

## Collar placement

Collars are visual-only hollow meshes. Each assembly is 1.2 mm wide:
a 0.768 mm silver centre and two 0.216 mm dark edges. Centres were obtained
from circular end profiles in the existing DAE scenes, applying their scene
transforms, units and unchanged URDF visual origins. The small existing
offsets of those mesh profiles relative to the joint axes are retained.
Radii add approximately 0.03–0.06 mm to the fitted outer collar surfaces;
the original shell meshes are never scaled or moved.

| Parent link | Centre XYZ (m, link frame) | Outer radius (m) |
| --- | --- | --- |
| link1 | 0, 0, -0.0770 | 0.03754 |
| link2 | 0.0008, 0, 0.0482 | 0.03779 |
| link3 | -0.0002, 0, 0.0430 | 0.03404 |
| link4 | 0, 0.0012, -0.0483 | 0.02914 |
| link5 | 0.0001, 0.00078, -0.0490 | 0.02914 |
| link6 | 0.000105, -0.000204, -0.0104 | 0.02914 |

Regenerate the small shared mesh from the repository root:

```bash
python3 scripts/generate_realistic_robot_assets.py
```

The generated DAE contains fallback silver/dark materials; Gazebo uses
three independently coloured instances so that its URDF material conversion
preserves the contrast of both seams.
