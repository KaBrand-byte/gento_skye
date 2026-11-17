"""Resume official public GCS checkpoint files with bounded curl concurrency."""

import argparse
import base64
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.parse
import urllib.request


PREFIX = "checkpoints/pi05_droid/"


def verify_file(path, item):
    if not path.is_file() or path.stat().st_size != int(item["size"]):
        raise RuntimeError(f"Incomplete checkpoint file: {path}")
    if item.get("md5Hash"):
        digest = hashlib.md5()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(chunk)
        if base64.b64encode(digest.digest()).decode() != item["md5Hash"]:
            raise RuntimeError(f"Checksum mismatch: {path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--verify-only", action="store_true", help="verify an existing copy without downloading")
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("--workers must be positive")
    root = args.root
    (root / "bootstrap").mkdir(parents=True, exist_ok=True)
    (root / "logs").mkdir(parents=True, exist_ok=True)
    endpoint = "https://storage.googleapis.com/storage/v1/b/openpi-assets/o?" + urllib.parse.urlencode({
        "prefix": PREFIX, "fields": "items(name,size,generation,md5Hash),nextPageToken",
    })
    manifest_file = root / "bootstrap/pi05_droid-http-manifest.json"
    if manifest_file.exists():
        manifest = json.loads(manifest_file.read_text())
    else:
        with urllib.request.urlopen(endpoint, timeout=30) as response:
            manifest = json.load(response)
        manifest_file.write_text(json.dumps(manifest, indent=2))
    if manifest.get("nextPageToken"):
        raise RuntimeError("Unexpected paginated checkpoint manifest")
    destination = root / "cache/openpi/openpi-assets/checkpoints/pi05_droid"
    if destination.exists():
        for item in manifest["items"]:
            path = destination / item["name"].removeprefix(PREFIX)
            if not path.is_file() or path.stat().st_size != int(item["size"]):
                raise RuntimeError(f"Incomplete existing checkpoint: {path}")
            if args.verify_only:
                verify_file(path, item)
        (root / "logs/checkpoint-path.txt").write_text(str(destination) + "\n")
        print(json.dumps({"checkpoint": str(destination), "cached": True,
                          "checksums_verified": args.verify_only,
                          "objects": len(manifest["items"])}), flush=True)
        return
    if args.verify_only:
        raise RuntimeError(f"Checkpoint directory does not exist: {destination}")
    partial = destination.with_suffix(".partial")
    partial.mkdir(parents=True, exist_ok=True)

    def fetch(item):
        relative = item["name"].removeprefix(PREFIX)
        path = partial / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        size = int(item["size"])
        url = "https://storage.googleapis.com/openpi-assets/" + urllib.parse.quote(item["name"], safe="/")
        url += "?generation=" + item["generation"]
        offset = path.stat().st_size if path.is_file() else 0
        next_report = (offset // (256 * 1024 * 1024) + 1) * (256 * 1024 * 1024)
        while offset < size:
            # Long multi-GB responses were repeatedly cut off on this network.
            end = min(offset + 32 * 1024 * 1024, size) - 1
            chunk_path = path.with_suffix(".http-chunk")
            subprocess.run([
                "curl", "--fail", "--location", "--silent", "--show-error",
                "--connect-timeout", "20", "--retry", "3", "--retry-all-errors", "--retry-delay", "2",
                "--range", f"{offset}-{end}", "--max-filesize", str(end - offset + 1),
                "--output", str(chunk_path), url,
            ], check=True)
            if chunk_path.stat().st_size != end - offset + 1:
                raise RuntimeError(f"Incomplete HTTP chunk: {relative} at {offset}")
            with path.open("ab") as stream, chunk_path.open("rb") as chunk:
                for data in iter(lambda: chunk.read(4 * 1024 * 1024), b""):
                    stream.write(data)
            chunk_path.unlink()
            offset = end + 1
            if offset >= next_report:
                print(json.dumps({"received": relative, "bytes": offset, "total": size}), flush=True)
                next_report += 256 * 1024 * 1024
        verify_file(path, item)
        print(json.dumps({"verified": relative, "bytes": size}), flush=True)

    # Download large objects first; existing verified prefixes from gcsfs are reusable.
    items = sorted(manifest["items"], key=lambda item: int(item["size"]), reverse=True)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(fetch, item) for item in items]
        for future in as_completed(futures):
            future.result()
    if destination.exists():
        raise RuntimeError("Checkpoint destination already exists; refusing to overwrite")
    partial.rename(destination)
    (root / "logs/checkpoint-path.txt").write_text(str(destination) + "\n")
    print(json.dumps({"checkpoint": str(destination), "verified_objects": len(items)}), flush=True)


if __name__ == "__main__":
    main()
