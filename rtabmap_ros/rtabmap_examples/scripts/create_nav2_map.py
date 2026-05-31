#!/usr/bin/env python3
"""
Create a Nav2-style map (PGM + YAML) with pure white cells.

Usage:
  create_nav2_white_map.py [--size N] [--resolution R] [--output NAME]

The generated files are written next to this script.
"""
import argparse
import os
import sys


def write_pgm(path: str, width: int, height: int, value: int = 255) -> None:
    header = f"P5\n{width} {height}\n255\n"
    # create mutable pixel buffer
    data = bytearray([value] * (width * height))
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(data)


def write_yaml(path: str, image_name: str, resolution: float, origin=(0.0, 0.0, 0.0)) -> None:
    content = (
        f"image: {image_name}\n"
        f"resolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    with open(path, "w") as f:
        f.write(content)


def parse_args():
    p = argparse.ArgumentParser(description="Generate a Nav2 PGM+YAML map filled with white cells")
    p.add_argument("--size", type=int, default=255, help="width and height in pixels (square)")
    p.add_argument("--resolution", type=float, default=0.05, help="map resolution in meters/cell")
    p.add_argument("--output", type=str, default="nav2_white_map", help="output base filename (no ext)")
    p.add_argument("--block-w", type=int, default=0, help="black block width in pixels (0 = none)")
    p.add_argument("--block-h", type=int, default=0, help="black block height in pixels (0 = none)")
    return p.parse_args()


def main():
    args = parse_args()
    size = args.size
    out_base = args.output

    script_dir = os.path.dirname(os.path.realpath(__file__))
    pgm_path = os.path.join(script_dir, out_base + ".pgm")
    yaml_path = os.path.join(script_dir, out_base + ".yaml")

    try:
        # create base white image
        # we'll write then optionally overwrite a centered black rectangle
        write_pgm(pgm_path, size, size, 255)
        # if block requested, modify the PGM in-place
        if args.block_w > 0 and args.block_h > 0:
            # read the file header to find pixel offset
            with open(pgm_path, "rb+") as f:
                # read and parse header
                header = b""
                # read until we see the whitespace after maxval
                # simple parser: read lines
                f.seek(0)
                magic = f.readline()
                dims = f.readline()
                # handle possible comments - naive: if dims starts with b'#', skip
                while dims.startswith(b"#"):
                    dims = f.readline()
                # dims may contain width height
                try:
                    parts = dims.split()
                    if len(parts) < 2:
                        # maybe width/height on next line
                        dims2 = f.readline()
                        parts += dims2.split()
                    width = int(parts[0])
                    height = int(parts[1])
                except Exception:
                    # fallback to provided size
                    width = size
                    height = size
                maxval = f.readline()
                # compute pixel array start
                pixel_start = f.tell()
                # load pixel data
                f.seek(pixel_start)
                pixels = bytearray(f.read())

                bw = args.block_w
                bh = args.block_h
                # clamp block to image
                bw = min(bw, width)
                bh = min(bh, height)
                left = (width - bw) // 2
                top = (height - bh) // 2

                for r in range(top, top + bh):
                    row_start = r * width
                    for c in range(left, left + bw):
                        pixels[row_start + c] = 0

                # rewrite pixel data
                f.seek(pixel_start)
                f.write(pixels)
        write_yaml(yaml_path, os.path.basename(pgm_path), args.resolution)
    except Exception as e:
        print("Error writing files:", e, file=sys.stderr)
        sys.exit(1)

    print("Wrote:")
    print("  ", pgm_path)
    print("  ", yaml_path)


if __name__ == "__main__":
    main()
