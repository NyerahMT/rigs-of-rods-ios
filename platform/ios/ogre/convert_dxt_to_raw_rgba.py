#!/usr/bin/env python3
"""Deterministically decode legacy RoR DXT1/DXT3 DDS textures to RGBA8.

Output format is intentionally tiny and private to the iOS port:
    0..7   ASCII b'RORRGBA1'
    8..11  uint32 little-endian width
    12..15 uint32 little-endian height
    16..   width*height RGBA8 pixels, row-major, top mip only

This avoids relying on OGRE's legacy DDS -> Metal iOS fallback while preserving
exactly the artwork and UV layout authored by Rigs of Rods content.
"""

from __future__ import annotations

import argparse
import hashlib
import struct
from pathlib import Path

MAGIC = b"RORRGBA1"
DDS_MAGIC = b"DDS "


def rgb565(value: int) -> tuple[int, int, int, int]:
    r5 = (value >> 11) & 0x1F
    g6 = (value >> 5) & 0x3F
    b5 = value & 0x1F
    return (
        (r5 * 255 + 15) // 31,
        (g6 * 255 + 31) // 63,
        (b5 * 255 + 15) // 31,
        255,
    )


def mix(a: tuple[int, int, int, int], b: tuple[int, int, int, int], wa: int, wb: int, div: int) -> tuple[int, int, int, int]:
    return tuple((a[i] * wa + b[i] * wb) // div for i in range(4))  # type: ignore[return-value]


def dxt_colour_table(c0: int, c1: int, allow_dxt1_transparency: bool) -> list[tuple[int, int, int, int]]:
    a = rgb565(c0)
    b = rgb565(c1)
    if allow_dxt1_transparency and c0 <= c1:
        return [a, b, mix(a, b, 1, 1, 2), (0, 0, 0, 0)]
    return [a, b, mix(a, b, 2, 1, 3), mix(a, b, 1, 2, 3)]


def decode_block_dxt1(block: bytes) -> list[tuple[int, int, int, int]]:
    c0, c1, indices = struct.unpack("<HHI", block)
    colours = dxt_colour_table(c0, c1, True)
    return [colours[(indices >> (2 * i)) & 0x3] for i in range(16)]


def decode_block_dxt3(block: bytes) -> list[tuple[int, int, int, int]]:
    alpha_bits = int.from_bytes(block[:8], "little")
    c0, c1, indices = struct.unpack("<HHI", block[8:16])
    colours = dxt_colour_table(c0, c1, False)
    out: list[tuple[int, int, int, int]] = []
    for i in range(16):
        r, g, b, _ = colours[(indices >> (2 * i)) & 0x3]
        a4 = (alpha_bits >> (4 * i)) & 0xF
        out.append((r, g, b, a4 * 17))
    return out


def decode_dds_top_mip(data: bytes) -> tuple[int, int, bytes, str]:
    if len(data) < 128 or data[:4] != DDS_MAGIC:
        raise ValueError("not a classic DDS file")
    if struct.unpack_from("<I", data, 4)[0] != 124:
        raise ValueError("unsupported DDS header size")

    height = struct.unpack_from("<I", data, 12)[0]
    width = struct.unpack_from("<I", data, 16)[0]
    pf_size = struct.unpack_from("<I", data, 76)[0]
    pf_flags = struct.unpack_from("<I", data, 80)[0]
    fourcc = data[84:88]
    if pf_size != 32 or not (pf_flags & 0x4):
        raise ValueError("DDS is not FourCC-compressed")
    if width <= 0 or height <= 0 or width > 16384 or height > 16384:
        raise ValueError(f"invalid DDS dimensions {width}x{height}")

    if fourcc == b"DXT1":
        block_bytes = 8
        decoder = decode_block_dxt1
        format_name = "DXT1"
    elif fourcc == b"DXT3":
        block_bytes = 16
        decoder = decode_block_dxt3
        format_name = "DXT3"
    else:
        raise ValueError(f"unsupported DDS FourCC {fourcc!r}; expected DXT1 or DXT3")

    blocks_x = (width + 3) // 4
    blocks_y = (height + 3) // 4
    needed = blocks_x * blocks_y * block_bytes
    payload = data[128:128 + needed]
    if len(payload) != needed:
        raise ValueError("DDS top mip is truncated")

    rgba = bytearray(width * height * 4)
    offset = 0
    for by in range(blocks_y):
        for bx in range(blocks_x):
            pixels = decoder(payload[offset:offset + block_bytes])
            offset += block_bytes
            for py in range(4):
                y = by * 4 + py
                if y >= height:
                    continue
                for px in range(4):
                    x = bx * 4 + px
                    if x >= width:
                        continue
                    dst = (y * width + x) * 4
                    rgba[dst:dst + 4] = bytes(pixels[py * 4 + px])

    return width, height, bytes(rgba), format_name


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_dds", type=Path)
    parser.add_argument("dest_rgba", type=Path)
    args = parser.parse_args()

    source = args.source_dds.read_bytes()
    width, height, rgba, format_name = decode_dds_top_mip(source)
    output = MAGIC + struct.pack("<II", width, height) + rgba
    args.dest_rgba.parent.mkdir(parents=True, exist_ok=True)
    args.dest_rgba.write_bytes(output)

    expected = 16 + width * height * 4
    if len(output) != expected:
        raise RuntimeError("raw RGBA output size validation failed")

    digest = hashlib.sha256(rgba).hexdigest()
    print(f"{args.source_dds.name}: {format_name} {width}x{height} -> RGBA8 sha256={digest}")


if __name__ == "__main__":
    main()
