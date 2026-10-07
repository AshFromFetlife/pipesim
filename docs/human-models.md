# Human controls and imported character models

Each human uses the same 19 physical segments and 18 joints. An imported model is a visual skin driven by those segments: its mesh, clothes, textures and extra bones do not change collision shapes, mass, inertia, contacts or load calculations. The browser editor, simulation playback, image previews, headless CLI renders and exported animations show the enabled skin. Turning it off uses the engineering mannequin in every output without discarding the import configuration.

## Interchange format and authoring tools

Use **glTF 2.0**, preferably a single **GLB** file. It stores a joint hierarchy, inverse bind transforms, vertex weights, UV coordinates and texture materials together. PipeSim retains this data instead of flattening the import into a rigid mesh. glTF itself does not assign anatomical meanings to bone names; **VRM** adds humanoid bone mappings, which PipeSim reads when available. See the [Khronos glTF specification](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html) and [VRM humanoid specification](https://vrm.dev/en/vrm1/humanoid/).

- **Blender:** select the armature and skinned meshes and export **glTF 2.0**, with **GLB** as the format. Keep skinning, materials and UVs enabled. Use image textures with glTF-compatible materials; bake procedural effects when needed. Export a neutral, straight-legged A or T pose and disable Draco compression. See [Blender's glTF exporter documentation](https://docs.blender.org/manual/en/4.3/addons/import_export/scene_gltf2.html).
- **VRoid Studio:** export a `.vrm` character and import it directly. Saving the editable VRoid project is a different operation from exporting the avatar. PipeSim uses the humanoid skeleton and ordinary glTF materials; full VRM expressions, MToon shading and authored VRM spring physics are not reproduced. See [VRoid's VRM export guide](https://vroid.pixiv.help/hc/en-us/articles/360014383713-How-to-create-a-VRM-file).
- **MakeHuman / MPFB:** prepare the character in Blender, using MPFB's GameEngine rig and texture-based GameEngine materials where practical, then export GLB through Blender. Apply or remove helper and mask geometry on an export copy. MPFB's guide describes preparation and an FBX export path; GLB is the PipeSim interchange step. See [MPFB's export guidance](https://static.makehumancommunity.org/mpfb/docs/exporting.html).

## Importing and mapping

Choose a `.glb`, `.gltf` or `.vrm` in the Add Human dialog or the human's properties. For a separate glTF export, select its `.bin` and PNG, JPEG or WebP textures together with the `.gltf`. Companion filenames must be unique when the browser file picker cannot preserve subfolder names; GLB avoids that ambiguity. Selected files may total up to **20 MiB**; rigs may contain up to **4096 nodes** and **two million skinned vertices**. Draco, Meshopt and KTX2 compression require an uncompressed export first.

The importer embeds companion files and stores the resulting asset under `.pipesim/human-models/` inside the workspace. It rejects network URLs, missing dependencies and paths outside the model folder. Save As updates the relative reference; portable design bundles copy the asset. Keep the workspace's asset folder with ordinary saved designs when moving them to another machine.

VRM metadata and common bone names provide initial mappings. Human properties let you map each physical segment to a named skeleton bone or its node index. Node indices distinguish duplicate names. Check the mapping in a neutral pose, with bent elbows and knees, and during playback before using an unfamiliar character for visual assessment. Missing or unsuitable rigs produce a visible fallback to the simple mannequin. Finger and face bones can stay unmapped.

Turn off **Use imported model** in human properties to show the simple model while editing or rendering. This retains the file, bone mappings and extra-bone settings; turn it back on to restore the skin.

In the editor, a missing or incompatible appearance falls back to the simple mannequin, with an error explaining the problem. An image or animation export with an enabled missing or incompatible appearance fails explicitly instead of silently substituting a mannequin. Disabled appearances do not need their asset to open, edit, simulate or save the design. Portable export retains a missing disabled reference; re-enabling it requires restoring or replacing the original asset.

## Extra bones

The default mode applies to unmapped bones; individual bones can override it.

| Mode | Intended use |
|---|---|
| Fixed to parent | Retain the imported local transform, useful for fingers, faces and rigid accessories. |
| Fixed angle to parent | Set a constant local angular offset where the imported pose needs correction. |
| Weighted ball joint | Visual pendulum motion for hair or hanging dress bones. Mass, damping and an angle limit control the effect. |
| Damped weighted spring | Visual restoring motion for soft body details and pocket contents. Mass, stiffness, damping and an angle limit control the effect. |

These are visual secondary motions. They do not add physical bodies, clothing contact or forces to the structure, and are not a cloth or soft-tissue solver. Model animations are not played over the ragdoll; body movement comes from the physical segments.

## Posture and flexibility

Strength scale multiplies the mannequin's assumed joint torque capacities; passive joint damping resists relative joint rotation without holding a posture. Random spasms, Fidget, Struggle and Destructive provide reproducible procedural loading patterns, not clinical or individual movement predictions. Destructive supplies short maximal torque bursts; it is not a guarantee of the worst possible load in every structure.

The flexibility settings run from Minimum through Athletic, Gymnast, Contortionist, Full socket span and Full 360. Fragile starts at Minimum and releases an overloaded joint's angular stops while retaining its connection; captured frames preserve the dislocation. These ranges and release thresholds are engineering approximations, not population percentiles, diagnoses, measured ligament properties or predictions of a specific injury. Bone fracture is not modelled. Full socket span retains self contact, with a damped contact approximation at directly connected segments that allows their existing joint-cap overlap. Full 360 disables contact between segments of the same person, allowing impossible overlapping poses; collisions with the structure and other people remain enabled. Review the physical [model limits](model-limits.md) alongside any visual result.

Fidget refreshes small random efforts once per second while retaining posture servos. Struggle changes direction smoothly with efforts up to 18% of assumed peak strength. Random spasms and Destructive use brief high-strength bursts and speed-limited joint actuation. The movement seed makes these patterns reproducible. There is no fatigue, balance, deliberate escape planning or clinical movement model. New humans start with Minimum flexibility; older files without a flexibility setting retain the original Athletic envelope. Initial catalog poses adapt to the chosen envelope, while manually entered anatomical angles must fit it. Reducing flexibility on an already edited human requires its current pose to fit the narrower envelope.

## Rendering and video export

In the browser, choose **Export → Render a video**, or **Render recording as video** in the simulation panel. Export the current recording or saved animation tracks as MP4 or GIF, using the current camera and selected size, lighting and background. Recorded motion retains its original frame rate; authored animation has duration and frame-rate controls. The result includes a preview and download link.

MP4 requires FFmpeg on the server PATH; on Windows, `~/bin/ffmpeg.exe` is also checked for Git Bash installations. Set `PIPESIM_FFMPEG` to a full executable path to override discovery. GIF requires no encoder.

Headless rendering uses the existing CPU renderer with imported base-color textures and UVs; studio lighting differs from the browser's PBR lighting. It needs neither a running editor nor Node.js. Exported recordings drive the same bone calibration and visual secondary motion by recorded timestamps.

## Regression checks

`npm run check` includes editor interaction and skin-retargeting regressions. The Python human, import, grouping and server tests cover physical controls, dislocations, save/reopen, portable assets and invalid imports. Set `PYTHON` to the project virtual environment's Python when running Node tests if the shell's Python does not have PipeSim's dependencies.

`npm run human-render-fixtures` creates two weighted test rigs and authored/recorded mannequin poses under `output/human-model-regression/`. Serve the repository with a local static HTTP server and open `tests/human-model-render.html` to compare skins with their collision bodies. The gallery includes standing, raised arms, seated, crouching, floor poses and real Fidget recording frames; its save button exports the comparison as PNG.

The optional real-model gallery cases use the Three.js example [Soldier](https://github.com/mrdoob/three.js/blob/dev/examples/models/gltf/Soldier.glb) and [Xbot](https://github.com/mrdoob/three.js/blob/dev/examples/models/gltf/Xbot.glb) assets, downloaded into `output/human-model-checks/Soldier.glb` and `output/human-model-checks/Xbot.glb`. They are verification inputs, not bundled PipeSim assets. The automated tests generate their own rigs and do not require network downloads.

The headless tests compare all weighted vertices and mapped bone transforms against the browser renderer on four generated rig variants across authored and simulated poses, and check textured PNGs, GIFs, PNG sequences and the CLI. When FFmpeg is available, they encode and decode an MP4 and compare its frames with the renderer. Optional Soldier and Xbot cases can be included with `node tests/helpers/headless-human-fixtures.mjs output/headless-parity output/human-model-checks/Soldier.glb output/human-model-checks/Xbot.glb`.
