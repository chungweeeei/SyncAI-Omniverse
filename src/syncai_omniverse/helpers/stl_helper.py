"""STL parsing helpers.

Pure-Python STL reader — no pxr / Isaac Sim dependency, so it can be unit-tested
on any machine with just the standard library.

STL file layout reference:
    Binary : 80-byte header | uint32 triangle count | N * 50-byte triangle records
    ASCII  : "solid <name>" ... "facet normal nx ny nz / outer loop / vertex * 3
             / endloop / endfacet" ... "endsolid"
"""

import os
import struct

from typing import List, Tuple

# Binary STL layout constants. Named so the magic numbers below read as intent,
# not as arbitrary offsets.
BINARY_HEADER_SIZE = 80           # Fixed-size header; content undefined by spec
BINARY_COUNT_SIZE = 4             # uint32 little-endian triangle count
BINARY_TRIANGLE_SIZE = 50         # 12 floats (normal + 3 vertices) + 2-byte attr
BINARY_MIN_SIZE = BINARY_HEADER_SIZE + BINARY_COUNT_SIZE  # 84 bytes

# struct format for one binary triangle:
#   12f -> normal.xyz + v1.xyz + v2.xyz + v3.xyz (all little-endian float32)
#   H   -> uint16 "attribute byte count" (almost always 0; some tools abuse it for color)
TRIANGLE_STRUCT = "<12fH"


def parse_stl(file_path: str) -> Tuple[list, list, int]:
    """Parse an STL file (binary or ASCII) and return raw triangles.

    Returns:
        raw_vertices: list of (x, y, z) tuples, 3 per triangle (NOT yet
            deduplicated — a vertex shared by K triangles appears K times).
        raw_normals:  list of (nx, ny, nz) tuples, 1 per triangle.
        num_triangles: int.
    """
    if is_binary_stl(file_path):
        return parse_binary_stl(file_path)

    return parse_ascii_stl(file_path)


def is_binary_stl(file_path: str) -> bool:
    """Detect whether an STL file is binary or ASCII.

    The naive check — "does the file start with 'solid'?" — is unreliable because
    some binary exporters also put the word "solid" in the 80-byte header. Instead
    we verify the file size matches the exact formula required by the binary spec:
    84 + 50 * num_triangles. An ASCII file coincidentally satisfying that equation
    is astronomically unlikely.
    """
    file_size = os.path.getsize(file_path)

    # Any file smaller than header + count cannot be binary, and reading the
    # uint32 below would raise struct.error on truncated input.
    if file_size < BINARY_MIN_SIZE:
        return False

    with open(file_path, "rb") as f:
        f.read(BINARY_HEADER_SIZE)  # skip header — content is not standardized
        num_triangles = struct.unpack("<I", f.read(BINARY_COUNT_SIZE))[0]

    expected_size = BINARY_MIN_SIZE + num_triangles * BINARY_TRIANGLE_SIZE
    return file_size == expected_size


def parse_binary_stl(file_path: str) -> Tuple[list, list, int]:
    """Parse a binary STL file.

    Triangles are read one at a time via struct.unpack rather than a single bulk
    read so the code stays obvious; for >1M-triangle meshes this becomes a
    bottleneck and numpy.frombuffer would be the upgrade path.
    """
    raw_vertices: List[Tuple[float, float, float]] = []
    raw_normals: List[Tuple[float, float, float]] = []

    with open(file_path, "rb") as f:
        f.read(BINARY_HEADER_SIZE)  # header is vendor-specific, ignore it
        num_triangles = struct.unpack("<I", f.read(BINARY_COUNT_SIZE))[0]

        for _ in range(num_triangles):
            # data = (nx, ny, nz, v1x, v1y, v1z, v2x, v2y, v2z, v3x, v3y, v3z, attr)
            data = struct.unpack(TRIANGLE_STRUCT, f.read(BINARY_TRIANGLE_SIZE))
            raw_normals.append((data[0], data[1], data[2]))
            raw_vertices.append((data[3], data[4], data[5]))
            raw_vertices.append((data[6], data[7], data[8]))
            raw_vertices.append((data[9], data[10], data[11]))
            # data[12] (attribute byte count) intentionally discarded

    return raw_vertices, raw_normals, num_triangles


def parse_ascii_stl(file_path: str) -> Tuple[list, list, int]:
    """Parse an ASCII STL file.

    We key off the leading token of each line rather than a strict grammar because
    real-world ASCII STLs have inconsistent whitespace / capitalization / line
    endings between exporters, and a tolerant parser survives more files.
    """
    raw_vertices: List[Tuple[float, float, float]] = []
    raw_normals: List[Tuple[float, float, float]] = []
    num_triangles = 0

    with open(file_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue
            # "facet normal nx ny nz" — one per triangle, counted here.
            if parts[0] == "facet" and parts[1] == "normal":
                raw_normals.append(
                    (float(parts[2]), float(parts[3]), float(parts[4]))
                )
                num_triangles += 1
            # "vertex x y z" — appears exactly 3 times per facet.
            elif parts[0] == "vertex":
                raw_vertices.append(
                    (float(parts[1]), float(parts[2]), float(parts[3]))
                )

    return raw_vertices, raw_normals, num_triangles


def deduplicate_vertices(raw_vertices) -> Tuple[list, list]:
    """Deduplicate vertices and build a face index array.

    STL stores every triangle's 3 vertices independently, so a vertex shared by K
    triangles is duplicated K times. USD (and GPU pipelines in general) expect an
    indexed mesh: one unique point list + an index per face-corner. This pass
    collapses duplicates and emits the index array.

    The 6-decimal rounding in the dict key is a numerical-equality trick: two
    vertices that *should* be the same point can differ by a few ULPs after an
    exporter's float math, so a naive tuple-as-key would treat them as distinct
    and leave the mesh with invisible seams that break smooth shading and
    collision. 1e-6 is tight enough to preserve sub-mm detail at meter scale and
    loose enough to absorb float round-trip noise.

    Returns:
        unique_points: list of (x, y, z) tuples
        face_indices:  list of int indices into unique_points
    """
    vertex_map: dict = {}
    unique_points: List[Tuple[float, float, float]] = []
    face_indices: List[int] = []

    for v in raw_vertices:
        key = (round(v[0], 6), round(v[1], 6), round(v[2], 6))
        if key not in vertex_map:
            vertex_map[key] = len(unique_points)
            unique_points.append(v)
        face_indices.append(vertex_map[key])

    return unique_points, face_indices


def compute_bounds(points) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Compute the axis-aligned bounding box of a point list.

    Hand-rolled rather than `min(points, key=...)` / numpy because this module
    stays stdlib-only, and a single pass with scalar compares is faster than
    six builtin calls that each traverse the list.
    """
    min_x = min_y = min_z = float("inf")
    max_x = max_y = max_z = float("-inf")

    for x, y, z in points:
        if x < min_x:
            min_x = x
        if x > max_x:
            max_x = x
        if y < min_y:
            min_y = y
        if y > max_y:
            max_y = y
        if z < min_z:
            min_z = z
        if z > max_z:
            max_z = z

    return (min_x, min_y, min_z), (max_x, max_y, max_z)
