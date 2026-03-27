"""Visualize the TartanStar URDF in Rerun."""

from __future__ import annotations

from collections import defaultdict

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from tstar_description import load_urdf


def _children_of(joints: dict) -> dict[str, list[str]]:
    children: dict[str, list[str]] = defaultdict(list)
    for j in joints.values():
        children[j["parent"]].append(j["child"])
    return children


def _root_link(robot: dict) -> str:
    children = {j["child"] for j in robot["joints"].values()}
    all_links = {j["parent"] for j in robot["joints"].values()} | children
    roots = all_links - children
    return next(iter(roots))


def _log_frames(robot: dict) -> None:
    joints = robot["joints"]
    parent_of = robot["parent_of"]
    children = _children_of(joints)
    root = _root_link(robot)

    root_path = root
    rr.log(root_path, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Animate root spinning around Z over 120 frames
    n_frames = 120
    for i in range(n_frames):
        angle = 2 * np.pi * i / n_frames
        c, s = np.cos(angle), np.sin(angle)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        rr.set_time("frame", sequence=i)
        rr.log(root_path, rr.Transform3D(mat3x3=Rz))

    # BFS to log each child frame with its joint transform relative to parent
    queue = [root]
    path_of: dict[str, str] = {root: root}

    while queue:
        link = queue.pop()
        for child in children.get(link, []):
            joint_name = parent_of[child]
            T = joints[joint_name]["transform"]
            translation = T[:3, 3]
            mat3x3 = T[:3, :3]

            child_path = path_of[link] + "/" + child
            path_of[child] = child_path

            rr.log(
                child_path,
                rr.Transform3D(translation=translation, mat3x3=mat3x3),
                static=True,
            )
            queue.append(child)

    # Log coordinate axes at every frame
    origins = np.zeros((3, 3))
    vectors = np.eye(3) * 0.05  # 5 cm axes
    colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]  # X=red, Y=green, Z=blue

    links = robot["links"]

    for link, path in path_of.items():
        rr.log(
            path + "/axes",
            rr.Arrows3D(origins=origins, vectors=vectors, colors=colors),
            static=True,
        )
        rr.log(
            path + "/label",
            rr.Points3D([[0, 0, 0]], labels=[link]),
            static=True,
        )

        info = links.get(link, {})
        mesh_path = info.get("mesh")
        if mesh_path is not None:
            T = info["visual_transform"]
            scale = info["mesh_scale"]
            rr.log(
                path + "/mesh",
                rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3], scale=scale),
                rr.Asset3D(path=mesh_path, albedo_factor=info.get("color")),
                static=True,
            )


def main() -> None:
    rr.init("tstar_description")

    blueprint = rrb.Blueprint(
        rrb.Spatial3DView(name="TartanStar", origin="/"),
        collapse_panels=False,
    )

    uri = rr.serve_grpc(default_blueprint=blueprint)
    print("Connect with:  rerun " + uri)

    robot = load_urdf()
    _log_frames(robot)

    print("Serving — press Ctrl+C to stop.")
    try:
        import time

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
