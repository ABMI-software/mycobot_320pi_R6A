# Wooden worktop material

`materials/textures/wood_albedo.png` is a photographic albedo reconstruction
based on the user's real worktop photographs (2026-09-09). Created with the
built-in `image_gen` tool, then copied unmodified into this package. It
reproduces honey-brown pine, long grain, knots, thin board joins and wear;
the exact knot and scratch locations are illustrative, not measured.

The complete image covers one 449 × 622 mm worktop. The UV mapping puts
image height along world X and image width along −Y. The material has zero
metalness and roughness 0.65, a visual approximation of the photographed
wood finish. Physical contact parameters are independent of these rendering
settings. The SDF collision box retains the measured 8.5 mm thickness.

`meshes/tabletop.dae` supplies the matching centered box and explicit UVs.
Its top uses the complete image once; its edges use adjacent narrow strips
at the same texture scale. The Collada material references the PNG by a
relative path, and `worlds/real_table.sdf` supplies the PBR finish. Both files
are installed with the `mycobot_description` package.

Generation prompt:

```text
Use case: photorealistic-natural
Asset type: physically based rendering wood ALBEDO texture for a Gazebo robot workbench.
Input images: first three user photographs are the material references (top view and close views of the same wooden board); the last image is the existing flat-color Gazebo screenshot, only context, NOT a texture reference.
Primary request: extract and reconstruct the bare wooden surface seen in the photographs as an unobstructed, orthographic, straight-down full-surface material texture. Match this used honey-brown pine worktop: golden amber medium-brown varnished wood, visible long fine grain, small dark irregular pine knots, slight scratches, faint scored lines and everyday wear. Roughly three wide glued boards with very thin longitudinal joins, as in the real photographs, not narrow parquet.
Composition: texture fills the ENTIRE rectangular image edge to edge; no border, no perspective, no physical table edge. Portrait 1536 by 2112 px preferred, representing 449 mm wide by 622 mm long. Grain and the long joins run vertically along image height. Distribute about 12–18 modest natural knots irregularly over the entire surface, many tiny, only two or three larger knots. Subtle board-to-board tonal variation.
Lighting: diffuse neutral flat albedo, even exposure throughout; no cast shadows, specular reflections, lighting gradients or ambient occlusion baked into the texture. Natural photographic microdetail, mild satin varnish appearance but no highlight.
Constraints: ONLY wood. Remove/reconstruct all areas covered by the robot, ArUco tags, papers, tape, cables, clamps, screens or background. NO robot, NO black-white fiducials, NO objects, NO readable text, NO logo, NO watermark. Keep the natural moderately warm wood color seen in the real photos, not a uniform beige, not saturated orange, not rustic reclaimed planks with deep gaps. The image itself is a flat usable wood material map, not a rendered table or scene.
```
