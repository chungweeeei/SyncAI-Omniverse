"""Download an Isaac Sim 5.1 ConveyorBelt USD asset with FULL dependencies.

NVIDIA's public S3 mirror at
    https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Props/Conveyors/
hosts the conveyor and its sibling files (Material Library/, Textures/).
S3 is not browsable without auth, so we discover the dependency tree by
following references inside each downloaded file.

Three classes of dependencies must be followed:

1. **USD layer-level refs** (subLayers, references, payloads) -- from
   `Sdf.Layer.GetExternalReferences()`.
2. **Asset-typed attributes** -- e.g. MDL shader inputs that point at
   `../Material Library/.../X.mdl` or `./T_Foo_Albedo.png`. These do NOT
   show up in `GetExternalReferences()`; we must traverse all prims and
   harvest `Asset`-valued attribute values via `Usd.Stage`.
3. **MDL → texture refs** -- `.mdl` files are plain text and reference
   `.png/.jpg/.exr` files via `texture_2d("...")`. Regex-scan to extract.

Local-path encoding rule:
    Some refs are URL-encoded (`Material%20Library/...usd`) and some are
    literal (`Material Library/...mdl`) -- inconsistent within the same
    asset. We URL-decode every path before saving so the local tree uses
    literal spaces (matches what the Kit MDL resolver expects). The HTTP
    URL stays URL-encoded so S3 returns the correct file.

Skipped:
    Isaac-bundled standard MDLs (`OmniPBR.mdl`, `OmniGlass.mdl`,
    `nvidia/...mdl`) -- already on Kit's MDL search path inside the
    container at `/isaac-sim/kit/mdl/core/Base/`. Absolute paths,
    `omniverse://`, and `http(s)://` refs.

Usage:
    python3 tools/fetch_isaac_conveyor.py                   # default A08
    python3 tools/fetch_isaac_conveyor.py --variant A05
    python3 tools/fetch_isaac_conveyor.py --variant A05 --dest assets/usd/conveyors

Output layout (literal-space directories):
    assets/usd/conveyors/
        ConveyorBelt_A08.usd
        Material Library/
            Metal/Misc/Metal_Rough_A.mdl
            Metal/Textures/T_Metal_Rough_A_*.png
            Plastic/Misc/Plastic_*.mdl
            ...
        Textures/
            *.usd
            T_ConveyorBelt_*.png
"""
from __future__ import annotations

import argparse
import re
import sys
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path, PurePosixPath

try:
    from pxr import Sdf, Usd
except ImportError:
    sys.stderr.write(
        "[fetch_isaac_conveyor] pxr (usd-core) is required: pip install usd-core\n"
    )
    raise

BASE_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Props/Conveyors/"
)

# Standard / Isaac-bundled MDLs that ship inside the Kit install -- skip
# them because (a) they aren't in this S3 prefix and (b) they're already
# resolvable via Kit's MDL search path.
_BUNDLED_MDLS = {
    "OmniPBR.mdl", "OmniGlass.mdl", "OmniSurface.mdl", "OmniHair.mdl",
    "OmniSurfaceLite.mdl",
}

_TEXTURE_EXT = (".png", ".jpg", ".jpeg", ".exr", ".hdr", ".tga", ".tif", ".tiff")
_MDL_TEXTURE_RE = re.compile(r'"([^"\n]+\.(?:png|jpg|jpeg|exr|hdr|tga|tif|tiff))"',
                             re.IGNORECASE)


def _is_skippable(ref: str) -> bool:
    """True if this ref shouldn't be fetched (absolute, web, or bundled)."""
    if not ref:
        return True
    if ref.startswith(("/", "http://", "https://", "omniverse://")):
        return True
    base = PurePosixPath(ref).name
    if base in _BUNDLED_MDLS:
        return True
    # `nvidia/aux_definitions.mdl` etc. -- standard MDL libraries.
    if ref.startswith("nvidia/"):
        return True
    return False


def _decode(rel: str) -> str:
    """URL-decode ('%20' -> ' ') so local paths match what MDL expects."""
    return urllib.parse.unquote(rel)


def _normalize(rel: str) -> str:
    """POSIX normpath -- collapse '.', '..' segments without escaping the root."""
    parts: list[str] = []
    for part in PurePosixPath(rel).parts:
        if part in ("", "."):
            continue
        if part == "..":
            if parts:
                parts.pop()
            continue
        parts.append(part)
    return "/".join(parts)


def _download(rel_path: str, dest_root: Path) -> Path | None:
    """Fetch BASE_URL+rel_path → dest_root/decoded(rel_path).

    Returns the local path if the file exists after the call (newly
    downloaded OR previously cached). None if the fetch failed.
    """
    decoded = _decode(rel_path)
    local = dest_root / decoded
    if local.exists() and local.stat().st_size > 0:
        return local
    local.parent.mkdir(parents=True, exist_ok=True)

    # Build the URL: each component must be URL-encoded for S3, but slashes
    # stay as path separators. urllib.parse.quote with safe='/' does that.
    # If the input rel_path is already encoded (contains '%20'), unquote
    # first and re-encode so we don't double-encode.
    url = BASE_URL + urllib.parse.quote(decoded, safe="/")
    print(f"  GET  {url}")
    try:
        urllib.request.urlretrieve(url, local)
    except urllib.error.HTTPError as exc:
        print(f"  SKIP {url}  ({exc.code} {exc.reason})")
        return None
    except urllib.error.URLError as exc:
        print(f"  SKIP {url}  ({exc.reason})")
        return None
    return local


def _layer_external_refs(local: Path) -> list[str]:
    """USD layer-level refs (subLayers/references/payloads). Empty for non-USD."""
    if local.suffix.lower() not in (".usd", ".usda", ".usdc", ".usdz"):
        return []
    layer = Sdf.Layer.FindOrOpen(str(local))
    if not layer:
        return []
    return [r for r in layer.GetExternalReferences() if r]


def _asset_attribute_refs(local: Path) -> list[str]:
    """Asset-typed attribute values across all prims (MDL shader inputs etc.)."""
    if local.suffix.lower() not in (".usd", ".usda", ".usdc", ".usdz"):
        return []
    try:
        stage = Usd.Stage.Open(str(local), Usd.Stage.LoadNone)
    except Exception:
        return []
    if not stage:
        return []
    seen: set[str] = set()
    refs: list[str] = []
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            v = attr.Get()
            if not v:
                continue
            p = v.path
            if p and p not in seen:
                seen.add(p)
                refs.append(p)
    return refs


def _mdl_texture_refs(local: Path) -> list[str]:
    """Regex-extract texture paths from a .mdl text file."""
    if local.suffix.lower() != ".mdl":
        return []
    try:
        text = local.read_text(encoding="utf-8", errors="ignore")
    except OSError:
        return []
    found = []
    seen: set[str] = set()
    for m in _MDL_TEXTURE_RE.finditer(text):
        p = m.group(1)
        if p not in seen and p.lower().endswith(_TEXTURE_EXT):
            seen.add(p)
            found.append(p)
    return found


def _resolve_relative(parent_dir: PurePosixPath, dep: str) -> str:
    """Join `dep` (relative path from a layer in parent_dir/) onto parent_dir
    and normalize. parent_dir is decoded (literal spaces); dep stays raw.
    """
    if str(parent_dir) == "" or str(parent_dir) == ".":
        return _normalize(_decode(dep))
    joined = f"{parent_dir}/{_decode(dep)}"
    return _normalize(joined)


def fetch_recursive(rel_path: str, dest_root: Path, visited: set[str]) -> None:
    norm = _normalize(_decode(rel_path))
    if norm in visited:
        return
    visited.add(norm)

    if _is_skippable(rel_path):
        return

    local = _download(rel_path, dest_root)
    if local is None:
        return

    parent_dir = PurePosixPath(norm).parent
    deps: list[str] = []
    deps += _layer_external_refs(local)
    deps += _asset_attribute_refs(local)
    deps += _mdl_texture_refs(local)
    for dep in deps:
        if _is_skippable(dep):
            continue
        joined = _resolve_relative(parent_dir, dep)
        fetch_recursive(joined, dest_root, visited)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", default="A08",
                        help="Conveyor variant suffix (A01..A49). Default: A08.")
    parser.add_argument("--dest", default="assets/usd/conveyors",
                        help="Output directory. Default: assets/usd/conveyors.")
    args = parser.parse_args()

    variant = args.variant.upper().lstrip("_")
    root_file = f"ConveyorBelt_{variant}.usd"
    dest_root = Path(args.dest).resolve()
    dest_root.mkdir(parents=True, exist_ok=True)

    print(f"Fetching ConveyorBelt_{variant} → {dest_root}")
    visited: set[str] = set()
    fetch_recursive(root_file, dest_root, visited)
    print(f"Done. {len(visited)} path(s) considered; "
          f"variant root: {dest_root / root_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
