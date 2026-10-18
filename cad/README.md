# CAD — printed mounts for the forklift (CadQuery)

Parametric 3D-printed parts for requirements §2 (ME-04 … ME-12). What each part is, how it is fitted, the
wiring and the parts still to buy: [docs/14-hardware-build.md](../../docs/14-hardware-build.md).

```
params.yaml      EVERY dimension (mm). Only the Huina 1577 Pro outer size is real; the rest is PLACEHOLDER (ME-01)
parts.py         one builder per part (returns one solid in print orientation) + the geometry the tests check
build.py         STL + STEP → out/ (gitignored), PNG previews → docs/media/cad/, out/parts.json (size, volume, mass)
tests/           pytest: valid single solid, fits the 220 × 220 × 250 bed, ME-04/05/06/07/09/10 checks,
                 params equal to the RL config and the firmware
```

```powershell
py -3.11 -m venv .venv
.venv\Scripts\pip install -r requirements.txt
.venv\Scripts\python build.py              # all parts; `build.py lidar_mount` for one, `--no-png` to skip previews
.venv\Scripts\python -m pytest -q
```

After measuring the forklift: edit `params.yaml` → `build.py` → `pytest` → slice `out/stl/*.stl` (PETG,
0.2 mm, 4 perimeters, 30 % gyroid). Values shared with `code/training/forklift_rl/config.yaml` (wheelbase,
fork size/spacing, mast height, lift range, pallet) and `LIFT_MAX_MM` in the firmware must be changed in all
places together — `test_params_match_rl_config` / `test_params_match_firmware_lift_range` fail otherwise.

Previews are rendered with matplotlib (no OpenGL needed), so `build.py` also runs headless in CI.
