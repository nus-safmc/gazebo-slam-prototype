from __future__ import annotations

import math
import os
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class Bounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class Box2D:
    cx: float
    cy: float
    sx: float
    sy: float
    yaw: float


@dataclass(frozen=True)
class Circle2D:
    cx: float
    cy: float
    r: float


@dataclass(frozen=True)
class SpawnSpot:
    spot_id: int
    x: float
    y: float
    yaw: float = 0.0


def default_spawn_spots(*, bounds: Bounds, margin_m: float = 4.0) -> list[SpawnSpot]:
    """Return 15 fixed spawn spots distributed across the arena."""
    span_x = max(1e-6, bounds.max_x - bounds.min_x)
    span_y = max(1e-6, bounds.max_y - bounds.min_y)
    usable_min_x = bounds.min_x + min(margin_m, 0.45 * span_x)
    usable_max_x = bounds.max_x - min(margin_m, 0.45 * span_x)
    usable_min_y = bounds.min_y + min(margin_m, 0.45 * span_y)
    usable_max_y = bounds.max_y - min(margin_m, 0.45 * span_y)

    xs = [
        usable_min_x,
        usable_min_x + 0.25 * (usable_max_x - usable_min_x),
        0.5 * (usable_min_x + usable_max_x),
        usable_min_x + 0.75 * (usable_max_x - usable_min_x),
        usable_max_x,
    ]
    ys = [usable_min_y, 0.5 * (usable_min_y + usable_max_y), usable_max_y]

    out: list[SpawnSpot] = []
    spot = 1
    for y in ys:
        for x in xs:
            out.append(SpawnSpot(spot_id=spot, x=float(x), y=float(y), yaw=0.0))
            spot += 1
    return out


def _parse_pose(text: Optional[str]) -> tuple[float, float, float, float, float, float]:
    if not text:
        return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    parts = [p for p in str(text).strip().split() if p]
    vals = [float(p) for p in parts[:6]] + [0.0] * max(0, 6 - len(parts))
    return vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]


def _compose_xy_yaw(
    parent: tuple[float, float, float],
    child: tuple[float, float, float],
) -> tuple[float, float, float]:
    px, py, pyaw = parent
    cx, cy, cyaw = child
    c = math.cos(pyaw)
    s = math.sin(pyaw)
    wx = px + c * cx - s * cy
    wy = py + s * cx + c * cy
    return wx, wy, pyaw + cyaw


def _extract_obstacles_and_bounds(*, world_sdf_path: str) -> tuple[list[Box2D], list[Circle2D], Bounds]:
    tree = ET.parse(world_sdf_path)
    root = tree.getroot()

    boxes: list[Box2D] = []
    circles: list[Circle2D] = []

    # Prefer ground plane size (present in this repo's worlds) as our view bounds.
    bounds: Optional[Bounds] = None

    for model in root.findall(".//world/model"):
        model_name = model.get("name", "")
        mx, my, _mz, _mr, _mp, myaw = _parse_pose((model.findtext("pose") or "").strip())

        if model_name == "ground_plane":
            plane_size = model.find(".//geometry/plane/size")
            if plane_size is not None and (plane_size.text or "").strip():
                parts = [p for p in plane_size.text.strip().split() if p]
                if len(parts) >= 2:
                    sx = float(parts[0])
                    sy = float(parts[1])
                    half_x = 0.5 * sx
                    half_y = 0.5 * sy
                    bounds = Bounds(
                        min_x=-half_x,
                        max_x=half_x,
                        min_y=-half_y,
                        max_y=half_y,
                    )

        for collision in model.findall(".//link/collision"):
            cx, cy, _cz, _cr, _cp, cyaw = _parse_pose((collision.findtext("pose") or "").strip())
            wx, wy, wyaw = _compose_xy_yaw((mx, my, myaw), (cx, cy, cyaw))

            box = collision.find(".//geometry/box/size")
            if box is not None and (box.text or "").strip():
                parts = [p for p in box.text.strip().split() if p]
                if len(parts) >= 2:
                    sx = float(parts[0])
                    sy = float(parts[1])
                    boxes.append(Box2D(cx=wx, cy=wy, sx=sx, sy=sy, yaw=wyaw))
                continue

            cyl = collision.find(".//geometry/cylinder")
            if cyl is not None:
                r_el = cyl.find("radius")
                if r_el is not None and (r_el.text or "").strip():
                    circles.append(Circle2D(cx=wx, cy=wy, r=float(r_el.text.strip())))
                continue

    if bounds is None:
        # Fallback (matches the current 40×40 playfield).
        bounds = Bounds(min_x=-20.0, max_x=20.0, min_y=-20.0, max_y=20.0)

    return boxes, circles, bounds


def _rect_corners(box: Box2D) -> list[tuple[float, float]]:
    hx = 0.5 * box.sx
    hy = 0.5 * box.sy
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    c = math.cos(box.yaw)
    s = math.sin(box.yaw)
    out = []
    for x, y in corners:
        out.append((box.cx + c * x - s * y, box.cy + s * x + c * y))
    return out


def select_spawn_points(
    *,
    world_sdf_path: str,
    robots: list[str],
    title: str = "Spawn drones: click to place each robot",
    spots: Optional[list[SpawnSpot]] = None,
) -> Optional[dict[str, tuple[float, float, float]]]:
    """Blocking UI: returns {robot: (x, y, yaw)} or None if canceled."""
    try:
        import tkinter as tk
        from tkinter import ttk
    except Exception:
        return None

    boxes, circles, bounds = _extract_obstacles_and_bounds(world_sdf_path=world_sdf_path)
    _ = spots  # kept for backward compatibility; UI now allows free placement.

    w_px = 800
    h_px = 800
    pad_px = 16
    canvas_w = w_px + 2 * pad_px
    canvas_h = h_px + 2 * pad_px

    def world_to_px(wx: float, wy: float) -> tuple[float, float]:
        x = (wx - bounds.min_x) / (bounds.max_x - bounds.min_x)
        y = (wy - bounds.min_y) / (bounds.max_y - bounds.min_y)
        px = pad_px + x * w_px
        py = pad_px + (1.0 - y) * h_px
        return px, py

    def px_to_world(px: float, py: float) -> tuple[float, float]:
        x = (px - pad_px) / w_px
        y = 1.0 - (py - pad_px) / h_px
        wx = bounds.min_x + x * (bounds.max_x - bounds.min_x)
        wy = bounds.min_y + y * (bounds.max_y - bounds.min_y)
        return wx, wy

    color_cycle = ['#00d1ff', '#ff5fa2', '#7CFF6B', '#FFD34D', '#b59bff', '#ff9b4a']
    robot_colors = {r: color_cycle[i % len(color_cycle)] for i, r in enumerate(robots)}

    root = tk.Tk()
    root.title(title)
    root.geometry(f"{canvas_w + 340}x{canvas_h + 40}")

    main = ttk.Frame(root, padding=10)
    main.grid(row=0, column=0, sticky="nsew")
    root.rowconfigure(0, weight=1)
    root.columnconfigure(0, weight=1)

    left = ttk.Frame(main)
    left.grid(row=0, column=0, sticky="nsew")
    right = ttk.Frame(main)
    right.grid(row=0, column=1, sticky="ns", padx=(14, 0))
    main.columnconfigure(0, weight=1)
    main.rowconfigure(0, weight=1)

    canvas = tk.Canvas(left, width=canvas_w, height=canvas_h, background="#101216", highlightthickness=0)
    canvas.grid(row=0, column=0, sticky="nsew")
    left.rowconfigure(0, weight=1)
    left.columnconfigure(0, weight=1)

    status_var = tk.StringVar(value="")
    status = ttk.Label(left, textvariable=status_var)
    status.grid(row=1, column=0, sticky="w", pady=(8, 0))

    info = ttk.Label(
        right,
        text=(
            "Click inside the arena to place robots.\n"
            "Order: " + ", ".join(robots) + "\n\n"
            "Left click: place next\n"
            "Right click: undo\n"
        ),
        justify="left",
    )
    info.grid(row=0, column=0, sticky="w")

    list_var = tk.StringVar(value="")
    list_lbl = ttk.Label(right, textvariable=list_var, justify="left")
    list_lbl.grid(row=1, column=0, sticky="w", pady=(10, 0))

    btn_row = ttk.Frame(right)
    btn_row.grid(row=2, column=0, sticky="w", pady=(14, 0))

    result: list[Optional[dict[str, tuple[float, float, float]]]] = [None]
    selected: dict[str, tuple[float, float, float]] = {}
    order_idx = 0

    # Placement safety margins.
    spawn_clearance_m = float(os.environ.get("SPAWN_CLEARANCE_M", "0.45"))
    spawn_min_separation_m = float(os.environ.get("SPAWN_MIN_SEPARATION_M", "0.85"))
    spawn_bounds_margin_m = float(os.environ.get("SPAWN_BOUNDS_MARGIN_M", "0.40"))

    def _point_in_box(wx: float, wy: float, b: Box2D, margin: float) -> bool:
        dx = wx - b.cx
        dy = wy - b.cy
        c = math.cos(b.yaw)
        s = math.sin(b.yaw)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        return abs(lx) <= 0.5 * b.sx + margin and abs(ly) <= 0.5 * b.sy + margin

    def _point_in_circle(wx: float, wy: float, c_: Circle2D, margin: float) -> bool:
        return math.hypot(wx - c_.cx, wy - c_.cy) <= c_.r + margin

    def _is_valid_spawn(wx: float, wy: float) -> tuple[bool, str]:
        if not (
            bounds.min_x + spawn_bounds_margin_m <= wx <= bounds.max_x - spawn_bounds_margin_m
            and bounds.min_y + spawn_bounds_margin_m <= wy <= bounds.max_y - spawn_bounds_margin_m
        ):
            return False, "Outside arena bounds"

        for b in boxes:
            if _point_in_box(wx, wy, b, spawn_clearance_m):
                return False, "Too close to obstacle"
        for c_ in circles:
            if _point_in_circle(wx, wy, c_, spawn_clearance_m):
                return False, "Too close to obstacle"

        for _r, (sx, sy, _syaw) in selected.items():
            if math.hypot(wx - sx, wy - sy) < spawn_min_separation_m:
                return False, "Too close to another robot"

        return True, ""

    def redraw() -> None:
        canvas.delete("all")

        # Draw bounds.
        x0, y0 = world_to_px(bounds.min_x, bounds.max_y)
        x1, y1 = world_to_px(bounds.max_x, bounds.min_y)
        canvas.create_rectangle(x0, y0, x1, y1, outline="#2a2f3a", width=2)

        # Obstacles.
        for b in boxes:
            pts = _rect_corners(b)
            flat = []
            for wx, wy in pts:
                px, py = world_to_px(wx, wy)
                flat.extend([px, py])
            canvas.create_polygon(flat, fill="#3b3f46", outline="#505662", width=1)

        for c in circles:
            px, py = world_to_px(c.cx, c.cy)
            pr = (c.r / (bounds.max_x - bounds.min_x)) * w_px
            canvas.create_oval(px - pr, py - pr, px + pr, py + pr, fill="#3b3f46", outline="#505662", width=1)

        # Markers.
        for r, (sx, sy, syaw) in selected.items():
            px, py = world_to_px(sx, sy)
            col = robot_colors.get(r, "#00d1ff")
            canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill=col, outline="#ffffff", width=1)
            canvas.create_text(px + 12, py - 12, text=r, fill=col, anchor="nw")

            # Orientation tick.
            tick_len = 14
            tx = px + tick_len * math.cos(syaw)
            ty = py - tick_len * math.sin(syaw)
            canvas.create_line(px, py, tx, ty, fill=col, width=2)

        # Right-side text.
        lines = []
        for r in robots:
            if r in selected:
                x, y, yaw = selected[r]
                lines.append(f"{r}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
            else:
                lines.append(f"{r}: (not set)")
        list_var.set("\n".join(lines))

        next_robot = robots[order_idx] if order_idx < len(robots) else "(done)"
        status_var.set(f"Next: {next_robot}")

    def finish() -> None:
        if len(selected) != len(robots):
            return
        result[0] = dict(selected)
        root.destroy()

    def cancel() -> None:
        result[0] = None
        root.destroy()

    def reset() -> None:
        nonlocal order_idx
        selected.clear()
        order_idx = 0
        redraw()

    def undo() -> None:
        nonlocal order_idx
        if order_idx <= 0:
            return
        order_idx -= 1
        r = robots[order_idx]
        selected.pop(r, None)
        redraw()

    def on_left_click(event) -> None:
        nonlocal order_idx
        if order_idx >= len(robots):
            return
        wx, wy = px_to_world(event.x, event.y)
        ok, reason = _is_valid_spawn(wx, wy)
        if not ok:
            status_var.set(f"Invalid spawn: {reason}")
            return
        r = robots[order_idx]
        selected[r] = (float(wx), float(wy), 0.0)
        order_idx += 1
        redraw()

    def on_right_click(_event) -> None:
        undo()

    done_btn = ttk.Button(btn_row, text="Done", command=finish)
    done_btn.grid(row=0, column=0, padx=(0, 8))
    ttk.Button(btn_row, text="Undo", command=undo).grid(row=0, column=1, padx=(0, 8))
    ttk.Button(btn_row, text="Reset", command=reset).grid(row=0, column=2, padx=(0, 8))
    ttk.Button(btn_row, text="Cancel", command=cancel).grid(row=0, column=3)

    canvas.bind("<Button-1>", on_left_click)
    canvas.bind("<Button-3>", on_right_click)

    redraw()
    root.mainloop()
    return result[0]


def write_world_with_robot_spawns(
    *,
    base_world_sdf_path: str,
    robots: list[str],
    spawns: dict[str, tuple[float, float, float]],
    z_m: float = 0.05,
    prune_other_robots: bool = True,
) -> str:
    """Write a temporary world SDF with updated <include><pose> for the selected robots."""
    tree = ET.parse(base_world_sdf_path)
    root = tree.getroot()

    world = root.find("world")
    if world is None:
        world = root.find(".//world")
    if world is None:
        raise ValueError("No <world> element found")

    includes = world.findall("include")
    include_by_name: dict[str, ET.Element] = {}
    uri_by_name: dict[str, str] = {}
    pose_by_name: dict[str, str] = {}

    for inc in includes:
        name_el = inc.find("name")
        if name_el is None or not (name_el.text or "").strip():
            continue
        name = name_el.text.strip()
        include_by_name[name] = inc

        uri_el = inc.find("uri")
        if uri_el is not None and (uri_el.text or "").strip():
            uri_by_name[name] = uri_el.text.strip()

        pose_el = inc.find("pose")
        if pose_el is not None and (pose_el.text or "").strip():
            pose_by_name[name] = pose_el.text.strip()

    def default_uri(robot: str) -> str:
        if robot == "robot":
            return "model://rex_quadcopter"
        if robot.startswith("robot"):
            suffix = robot.replace("robot", "", 1)
            if suffix.isdigit():
                # Models exist for 2..15; fall back to base model otherwise.
                n = int(suffix)
                if 2 <= n <= 15:
                    return f"model://rex_quadcopter_{n}"
                return "model://rex_quadcopter"
        return "model://rex_quadcopter"

    if prune_other_robots:
        keep = set(robots)
        for inc in list(includes):
            name_el = inc.find("name")
            if name_el is None or not (name_el.text or "").strip():
                continue
            name = name_el.text.strip()
            if name in keep:
                continue
            uri_el = inc.find("uri")
            uri = (uri_el.text or "").strip() if uri_el is not None else ""
            if uri.startswith("model://rex_quadcopter"):
                world.remove(inc)

    for r in robots:
        if r not in spawns:
            continue
        x, y, yaw = spawns[r]
        inc = include_by_name.get(r)
        if inc is None:
            inc = ET.SubElement(world, "include")
            uri_el = ET.SubElement(inc, "uri")
            uri_el.text = uri_by_name.get(r, default_uri(r))
            name_el = ET.SubElement(inc, "name")
            name_el.text = r
            pose_el = ET.SubElement(inc, "pose")
            pose_el.text = f"{x} {y} {z_m} 0 0 {yaw}"
            continue

        pose_el = inc.find("pose")
        if pose_el is None:
            pose_el = ET.SubElement(inc, "pose")
        pose_el.text = f"{x} {y} {z_m} 0 0 {yaw}"

    stamp = int(time.time() * 1000)
    out_path = os.path.join(
        tempfile.gettempdir(), f"playfield_spawn_{os.getpid()}_{stamp}.sdf"
    )
    tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path


def extract_robot_includes(
    *,
    world_sdf_path: str,
) -> dict[str, tuple[str, tuple[float, float, float]]]:
    """Return {name: (uri, (x,y,yaw))} for drone-like includes in a world SDF."""
    tree = ET.parse(world_sdf_path)
    root = tree.getroot()
    world = root.find("world") or root.find(".//world")
    if world is None:
        return {}

    out: dict[str, tuple[str, tuple[float, float, float]]] = {}
    for inc in world.findall("include"):
        name = (inc.findtext("name") or "").strip()
        if not name:
            continue
        uri = (inc.findtext("uri") or "").strip()
        if not uri.startswith("model://rex_quadcopter"):
            continue
        px, py, _pz, _r, _p, yaw = _parse_pose((inc.findtext("pose") or "").strip())
        out[name] = (uri, (px, py, yaw))
    return out
