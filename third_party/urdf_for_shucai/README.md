## URDF Placeholder

This directory is intentionally kept as a placeholder and does not include the
actual X2 URDF or mesh assets.

To use the offline IK examples, place the contents extracted from
`urdf_with_oh:op.zip` here so that the default model path becomes:

```text
third_party/urdf_for_shucai/x2_ultra_plus_omnipicker_omnipicker.urdf
```

Expected layout after extraction:

```text
third_party/urdf_for_shucai/
├── meshes/
├── omnipicker_omnipicker/
├── omnipicker_omnihand/
├── omnihand_omnipicker/
├── omnihand_omnihand/
└── x2_ultra_plus_omnipicker_omnipicker.urdf
```

If you store the model elsewhere, set:

```bash
export X2_IK_URDF=/path/to/your_robot.urdf
```
