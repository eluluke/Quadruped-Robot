from __future__ import annotations

import math
import struct
from pathlib import Path

import numpy as np


SOURCE_MESH_DIR = Path("/home/littlemoomoo/Desktop/ORIGINAL_3_DOF_ARM.URDF/meshes")
OUTPUT_MESH_DIR = Path("generated_meshes")


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform_matrix(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rpy_matrix(*rpy)
    out[:3, 3] = np.array(xyz)
    return out


def transform_binary_stl(src: Path, dst: Path, world_from_link: np.ndarray) -> None:
    data = bytearray(src.read_bytes())
    tri_count = struct.unpack_from("<I", data, 80)[0]
    link_from_world = np.linalg.inv(world_from_link)
    rot = link_from_world[:3, :3]
    trans = link_from_world[:3, 3]

    offset = 84
    for _ in range(tri_count):
        normal = np.array(struct.unpack_from("<fff", data, offset))
        normal = rot @ normal
        norm = np.linalg.norm(normal)
        if norm > 0:
            normal = normal / norm
        struct.pack_into("<fff", data, offset, *normal)
        offset += 12

        for _ in range(3):
            point = np.array(struct.unpack_from("<fff", data, offset))
            point = rot @ point + trans
            struct.pack_into("<fff", data, offset, *point)
            offset += 12

        offset += 2

    dst.write_bytes(data)


def main() -> None:
    OUTPUT_MESH_DIR.mkdir(exist_ok=True)

    world_from_base = np.eye(4)
    world_from_link1 = world_from_base @ transform_matrix(
        (0.00857185004749453, 0.027789077031409, 0.0204145814676108),
        (-1.8776081329983, -1.57079631189374, 1.5707963267949),
    )
    world_from_link2 = world_from_link1 @ transform_matrix(
        (0.181766743813069, 0, 0),
        (-1.5707963267949, 0, -1.7091290571648),
    )
    world_from_link3 = world_from_link2 @ transform_matrix(
        (0.18176674381307, 0, 0),
        (-1.5707963267949, 0, -1.87964671311042),
    )

    transforms = {
        "base_link.STL": world_from_base,
        "link1.STL": world_from_link1,
        "link2.STL": world_from_link2,
        "link3.STL": world_from_link3,
    }

    for filename, world_from_link in transforms.items():
        dst = OUTPUT_MESH_DIR / filename
        transform_binary_stl(SOURCE_MESH_DIR / filename, dst, world_from_link)
        print(f"wrote {dst}")


if __name__ == "__main__":
    main()
