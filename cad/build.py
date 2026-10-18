"""Build every part from params.yaml: STL + STEP into out/, PNG previews into docs/media/cad/.

    .venv\\Scripts\\python build.py            # all parts
    .venv\\Scripts\\python build.py --no-png   # skip the previews (CI)
    .venv\\Scripts\\python build.py lidar_mount camera_mount
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cadquery as cq

import parts

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"
MEDIA = HERE.parent.parent / "docs" / "media" / "cad"
PETG_G_PER_CM3 = 1.27


def tessellate(shape: cq.Shape, tol: float = 0.2):
    import numpy as np

    verts, tris = shape.tessellate(tol, 0.3)
    v = np.array([(p.x, p.y, p.z) for p in verts])
    return v, np.array(tris)


def render_png(wp: cq.Workplane, path: Path, title: str, ax=None) -> None:
    """Shaded matplotlib render (no OpenGL / VTK needed, works headless in CI)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    v, t = tessellate(wp.val())
    own = ax is None
    if own:
        fig = plt.figure(figsize=(4.8, 3.6), dpi=100)
        ax = fig.add_subplot(projection="3d")
    tri = v[t]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array([-0.35, -0.45, 0.82])
    light /= np.linalg.norm(light)
    shade = 0.35 + 0.65 * np.abs(n @ light)  # two-sided: OCC triangle winding is not consistent
    base = np.array([0.96, 0.69, 0.15])  # forklift yellow
    colors = np.clip(base[None, :] * shade[:, None], 0, 1)
    ax.add_collection3d(Poly3DCollection(tri, facecolors=colors, edgecolors="none", linewidths=0))
    lo, hi = v.min(axis=0), v.max(axis=0)
    span = np.maximum(hi - lo, 1e-3)
    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(tuple(span))  # a tuple: matplotlib normalises an array in place
    ax.view_init(elev=28, azim=-55)
    ax.set_axis_off()
    ax.set_title(f"{title}\n{span[0]:.0f} × {span[1]:.0f} × {span[2]:.0f} mm", fontsize=9)
    if own:
        fig.tight_layout()
        fig.savefig(path, dpi=100)
        plt.close(fig)


def render_sheet(built: dict[str, cq.Workplane], path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = list(built)
    cols = 4
    rows = (len(names) + cols - 1) // cols
    fig = plt.figure(figsize=(cols * 3.2, rows * 2.8), dpi=90)
    for i, name in enumerate(names):
        ax = fig.add_subplot(rows, cols, i + 1, projection="3d")
        render_png(built[name], path, name, ax=ax)
    fig.suptitle("Forklift printed mounts (params.yaml — PLACEHOLDER dimensions until ME-01)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*", help="parts to build (default: all)")
    ap.add_argument("--no-png", action="store_true")
    args = ap.parse_args(argv)

    p = parts.load_params()
    names = args.names or list(parts.PARTS)
    unknown = [n for n in names if n not in parts.PARTS]
    if unknown:
        print(f"unknown part(s): {unknown}; known: {list(parts.PARTS)}")
        return 2
    (OUT / "stl").mkdir(parents=True, exist_ok=True)
    (OUT / "step").mkdir(parents=True, exist_ok=True)
    if not args.no_png:
        MEDIA.mkdir(parents=True, exist_ok=True)

    built, summary = {}, []
    print(f"{'part':28s} {'size x × y × z (mm)':>24s} {'cm³':>7s} {'g PETG*':>8s}")
    for name in names:
        wp = parts.PARTS[name](p)
        shape = wp.val()
        if not shape.isValid():
            print(f"{name}: INVALID solid")
            return 1
        cq.exporters.export(wp, str(OUT / "stl" / f"{name}.stl"), tolerance=0.05, angularTolerance=0.2)
        cq.exporters.export(wp, str(OUT / "step" / f"{name}.step"))
        bb = shape.BoundingBox()
        vol = shape.Volume() / 1000.0
        mass = vol * PETG_G_PER_CM3
        summary.append({"part": name, "size_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                        "volume_cm3": round(vol, 1), "mass_g_solid_petg": round(mass, 1)})
        size = f"{bb.xlen:.0f} × {bb.ylen:.0f} × {bb.zlen:.0f}"
        print(f"{name:28s} {size:>24s} {vol:7.1f} {mass:8.1f}")
        built[name] = wp
        if not args.no_png:
            render_png(wp, MEDIA / f"{name}.png", name)
    total = sum(s["mass_g_solid_petg"] for s in summary)
    print(f"total printed mass ≤ {total:.0f} g (*100 % infill upper bound; ~40–50 % of it at 30 % infill)")
    (OUT / "parts.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if not args.no_png and not args.names:
        render_sheet(built, MEDIA / "all-parts.png")
    print(f"STL/STEP → {OUT}" + ("" if args.no_png else f"; PNG → {MEDIA}"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
