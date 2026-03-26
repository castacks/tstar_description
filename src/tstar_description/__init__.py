from __future__ import annotations

from importlib.resources import files
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from xacrodoc import XacroDoc

_PKG_ROOT = Path(str(files("tstar_description")))
_XACRO_PATH = _PKG_ROOT / "urdf" / "tartanstar.urdf.xacro"


def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY (extrinsic XYZ) angles to 3x3 rotation matrix."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _make_transform(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from xyz translation and rpy angles."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3, 3] = xyz
    return T


def _parse_urdf(urdf_string: str) -> dict[str, dict]:
    """Parse URDF XML into joint and link graphs.

    Returns a dict with:
      - 'joints': {joint_name: {parent, child, transform}}
      - 'parent_of': {child_link: joint_name}   (for tree traversal)
    """
    root = ElementTree.fromstring(urdf_string)

    joints: dict[str, dict] = {}
    parent_of: dict[str, str] = {}  # child_link -> joint_name

    for joint in root.findall("joint"):
        name = joint.get("name")
        assert name is not None
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        assert parent_el is not None and child_el is not None
        parent_link = parent_el.get("link")
        child_link = child_el.get("link")
        assert parent_link is not None and child_link is not None

        origin = joint.find("origin")
        if origin is not None:
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        else:
            xyz, rpy = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        joints[name] = {
            "parent": parent_link,
            "child": child_link,
            "transform": _make_transform(xyz, rpy),
        }
        parent_of[child_link] = name

    links: dict[str, dict] = {}
    for link in root.findall("link"):
        name = link.get("name")
        assert name is not None
        visual = link.find("visual")
        mesh_file: str | None = None
        mesh_el: ElementTree.Element | None = None
        visual_transform: np.ndarray = np.eye(4)
        if visual is not None:
            mesh_el = visual.find("geometry/mesh")
            if mesh_el is not None:
                mesh_file = mesh_el.get("filename")
            origin = visual.find("origin")
            if origin is not None:
                xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
                rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
                visual_transform = _make_transform(xyz, rpy)
        color_rgba: list[int] | None = None
        if visual is not None:
            color_el = visual.find("material/color")
            if color_el is not None:
                rgba_str = color_el.get("rgba")
                if rgba_str:
                    color_rgba = [round(float(v) * 255) for v in rgba_str.split()]
        mesh_path: Path | None = None
        mesh_scale: list[float] = [1.0, 1.0, 1.0]
        if mesh_file is not None:
            if mesh_file.startswith("file://"):
                mesh_path = Path(mesh_file.removeprefix("file://"))
            elif mesh_file.startswith("package://"):
                rel = mesh_file.removeprefix("package://").split("/", 1)[1]
                mesh_path = _XACRO_PATH.parents[1] / rel
            if mesh_el is not None:
                scale_str = mesh_el.get("scale")
                if scale_str:
                    mesh_scale = [float(v) for v in scale_str.split()]
        links[name] = {"mesh": mesh_path, "mesh_scale": mesh_scale, "visual_transform": visual_transform, "color": color_rgba}

    return {"joints": joints, "parent_of": parent_of, "links": links}



def _root_transform(link: str, parent_of: dict, joints: dict) -> np.ndarray:
    """Cumulative transform from root frame to ``link`` frame."""
    T = np.eye(4)
    current = link
    segments: list[np.ndarray] = []
    while current in parent_of:
        joint = joints[parent_of[current]]
        segments.append(joint["transform"])
        current = joint["parent"]
    for seg in reversed(segments):
        T = T @ seg
    return T


def load_urdf(xacro_path: str | Path = _XACRO_PATH) -> dict:
    """Load and parse tartanstar.urdf.xacro.

    Returns the parsed graph dict (joints + parent_of).
    """
    doc = XacroDoc.from_file(str(xacro_path))
    return _parse_urdf(doc.to_urdf_string())


def list_frames(robot: dict | None = None) -> list[str]:
    """Return all link names defined in the robot description."""
    if robot is None:
        robot = load_urdf()
    joints = robot["joints"]
    links: set[str] = set()
    for j in joints.values():
        links.add(j["parent"])
        links.add(j["child"])
    return sorted(links)


def get_transform(
    from_frame: str,
    to_frame: str,
    robot: dict | None = None,
) -> np.ndarray:
    """Get the 4x4 homogeneous transform T such that p_to = T @ p_from.

    Parameters
    ----------
    from_frame:
        Source link name (e.g. ``"base_link"``).
    to_frame:
        Target link name (e.g. ``"zed_camera_link"``).
    robot:
        Pre-loaded robot graph from :func:`load_urdf`.  If *None* the
        xacro file is parsed on every call.

    Returns
    -------
    np.ndarray
        4x4 homogeneous transform matrix.
    """
    if robot is None:
        robot = load_urdf()
    joints = robot["joints"]
    parent_of = robot["parent_of"]

    T_root_from = _root_transform(from_frame, parent_of, joints)
    T_root_to = _root_transform(to_frame, parent_of, joints)

    # T_from_to = inv(T_root_from) @ T_root_to
    return np.linalg.inv(T_root_from) @ T_root_to
