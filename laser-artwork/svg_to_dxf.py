#!/usr/bin/env python3
"""Convert outlined SVG paths to a resizable DXF (mm) for laser marking."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import ezdxf
from svgpathtools import parse_path

MM_PER_UNIT = 1.0  # viewBox units are mm (width="70mm" viewBox="0 0 70 20")
FLATTEN_STEP_MM = 0.08  # smaller = smoother curves for laser import


def svg_paths(svg_path: Path) -> list[str]:
    root = ET.parse(svg_path).getroot()
    ns = {"svg": "http://www.w3.org/2000/svg"}
    paths: list[str] = []
    for elem in root.findall(".//svg:path", ns):
        d = elem.get("d")
        if d:
            paths.append(d)
    if not paths:
        for elem in root.iter():
            if elem.tag.endswith("path"):
                d = elem.get("d")
                if d:
                    paths.append(d)
    return paths


def flatten_subpath(subpath, step: float) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    length = subpath.length(error=1e-3)
    if length == 0:
        return points

    count = max(int(length / step) + 1, 2)
    for i in range(count):
        t = i / (count - 1)
        pt = subpath.point(t)
        points.append((pt.real * MM_PER_UNIT, pt.imag * MM_PER_UNIT))
    return points


def main() -> None:
    here = Path(__file__).resolve().parent
    svg_file = here / "jcp-outlines.svg"
    dxf_file = here / "jcp-outlines.dxf"

    doc = ezdxf.new("R12", setup=True)
    msp = doc.modelspace()

    subpaths_pts: list[list[tuple[float, float]]] = []

    for d in svg_paths(svg_file):
        for sub in parse_path(d).continuous_subpaths():
            pts = flatten_subpath(sub, FLATTEN_STEP_MM)
            if len(pts) >= 2:
                subpaths_pts.append(pts)

    if not subpaths_pts:
        raise SystemExit("No path data found in SVG")

    all_y = [y for pts in subpaths_pts for _, y in pts]
    y_max = max(all_y)

    for pts in subpaths_pts:
        poly = [(x, y_max - y) for x, y in pts]
        closed = (poly[0][0] - poly[-1][0]) ** 2 + (poly[0][1] - poly[-1][1]) ** 2 < 0.05
        msp.add_polyline2d(poly, close=closed, dxfattribs={"layer": "0"})

    doc.saveas(dxf_file)
    flat = [(x, y_max - y) for pts in subpaths_pts for x, y in pts]
    xs = [p[0] for p in flat]
    ys = [p[1] for p in flat]
    print(
        f"Wrote {dxf_file}  ({len(subpaths_pts)} paths, "
        f"{max(xs) - min(xs):.1f} x {max(ys) - min(ys):.1f} mm)"
    )


if __name__ == "__main__":
    main()
