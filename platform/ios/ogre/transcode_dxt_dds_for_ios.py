#!/usr/bin/env python3
"""Normalize legacy RoR DDS textures to uncompressed BGRA8 for iOS Metal.

Classic RoR content uses a mixture of DXT1/DXT3/DXT5 and old uncompressed
16/24/32-bit RGB DDS files. OGRE's Metal backend on iOS is much happier with a
single explicit 32-bit BGRA representation, so this tool decodes the top mip and
rewrites it as canonical A8R8G8B8/BGRA8 while preserving the original filename.

The converter intentionally handles only classic DDS RGB and DXT formats. It
rejects unknown layouts instead of guessing channel order.
"""

from __future__ import annotations

import argparse
import hashlib
import struct
from pathlib import Path

DDS_MAGIC = b"DDS "
DDPF_ALPHAPIXELS = 0x1
DDPF_FOURCC = 0x4
DDPF_RGB = 0x40
DDSD_CAPS = 0x1
DDSD_HEIGHT = 0x2
DDSD_WIDTH = 0x4
DDSD_PITCH = 0x8
DDSD_PIXELFORMAT = 0x1000
DDSCAPS_TEXTURE = 0x1000

CANONICAL_MASKS = (0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000)


def _rgb565(value: int) -> tuple[int, int, int, int]:
    r5 = (value >> 11) & 0x1F
    g6 = (value >> 5) & 0x3F
    b5 = value & 0x1F
    return (
        (r5 * 255 + 15) // 31,
        (g6 * 255 + 31) // 63,
        (b5 * 255 + 15) // 31,
        255,
    )


def _mix(a: tuple[int, int, int, int], b: tuple[int, int, int, int], wa: int, wb: int, div: int) -> tuple[int, int, int, int]:
    return tuple((a[i] * wa + b[i] * wb) // div for i in range(4))  # type: ignore[return-value]


def _colour_table(c0: int, c1: int, dxt1_transparency: bool) -> list[tuple[int, int, int, int]]:
    a = _rgb565(c0)
    b = _rgb565(c1)
    if dxt1_transparency and c0 <= c1:
        return [a, b, _mix(a, b, 1, 1, 2), (0, 0, 0, 0)]
    return [a, b, _mix(a, b, 2, 1, 3), _mix(a, b, 1, 2, 3)]


def _decode_dxt1(block: bytes) -> list[tuple[int, int, int, int]]:
    c0, c1, indices = struct.unpack("<HHI", block)
    colours = _colour_table(c0, c1, True)
    return [colours[(indices >> (2 * i)) & 0x3] for i in range(16)]


def _decode_dxt3(block: bytes) -> list[tuple[int, int, int, int]]:
    alpha_bits = int.from_bytes(block[:8], "little")
    c0, c1, indices = struct.unpack("<HHI", block[8:16])
    colours = _colour_table(c0, c1, False)
    result: list[tuple[int, int, int, int]] = []
    for i in range(16):
        r, g, b, _ = colours[(indices >> (2 * i)) & 0x3]
        a = ((alpha_bits >> (4 * i)) & 0xF) * 17
        result.append((r, g, b, a))
    return result


def _dxt5_alpha_table(a0: int, a1: int) -> list[int]:
    if a0 > a1:
        return [
            a0, a1,
            (6 * a0 + 1 * a1) // 7,
            (5 * a0 + 2 * a1) // 7,
            (4 * a0 + 3 * a1) // 7,
            (3 * a0 + 4 * a1) // 7,
            (2 * a0 + 5 * a1) // 7,
            (1 * a0 + 6 * a1) // 7,
        ]
    return [
        a0, a1,
        (4 * a0 + 1 * a1) // 5,
        (3 * a0 + 2 * a1) // 5,
        (2 * a0 + 3 * a1) // 5,
        (1 * a0 + 4 * a1) // 5,
        0, 255,
    ]


def _decode_dxt5(block: bytes) -> list[tuple[int, int, int, int]]:
    a0, a1 = block[0], block[1]
    alpha_indices = int.from_bytes(block[2:8], "little")
    alphas = _dxt5_alpha_table(a0, a1)
    c0, c1, colour_indices = struct.unpack("<HHI", block[8:16])
    colours = _colour_table(c0, c1, False)
    result: list[tuple[int, int, int, int]] = []
    for i in range(16):
        r, g, b, _ = colours[(colour_indices >> (2 * i)) & 0x3]
        a = alphas[(alpha_indices >> (3 * i)) & 0x7]
        result.append((r, g, b, a))
    return result


def _decode_masked_channel(value: int, mask: int, default: int) -> int:
    if mask == 0:
        return default
    shift = (mask & -mask).bit_length() - 1
    bits = mask.bit_count()
    raw = (value & mask) >> shift
    maximum = (1 << bits) - 1
    if maximum <= 0:
        return default
    return (raw * 255 + maximum // 2) // maximum


def _decode_uncompressed_rgb(data: bytes, width: int, height: int, pf_flags: int) -> tuple[bytes, str]:
    bpp = struct.unpack_from("<I", data, 88)[0]
    if bpp not in (16, 24, 32):
        raise ValueError(f"unsupported uncompressed RGB bit depth {bpp}")

    masks = tuple(struct.unpack_from("<I", data, offset)[0] for offset in (92, 96, 100, 104))
    rmask, gmask, bmask, amask = masks
    if not rmask or not gmask or not bmask:
        raise ValueError("uncompressed RGB DDS is missing colour masks")

    bytes_per_pixel = bpp // 8
    packed_row_bytes = width * bytes_per_pixel
    header_flags = struct.unpack_from("<I", data, 8)[0]
    pitch_or_linear = struct.unpack_from("<I", data, 20)[0]
    row_pitch = pitch_or_linear if (header_flags & DDSD_PITCH) and pitch_or_linear >= packed_row_bytes else packed_row_bytes
    required = row_pitch * height
    payload = data[128:128 + required]
    if len(payload) < required:
        raise ValueError("uncompressed RGB top mip is truncated")

    rgba = bytearray(width * height * 4)
    has_alpha = bool(pf_flags & DDPF_ALPHAPIXELS) and amask != 0
    for y in range(height):
        row = payload[y * row_pitch:y * row_pitch + packed_row_bytes]
        for x in range(width):
            begin = x * bytes_per_pixel
            value = int.from_bytes(row[begin:begin + bytes_per_pixel], "little")
            dst = (y * width + x) * 4
            rgba[dst + 0] = _decode_masked_channel(value, rmask, 0)
            rgba[dst + 1] = _decode_masked_channel(value, gmask, 0)
            rgba[dst + 2] = _decode_masked_channel(value, bmask, 0)
            rgba[dst + 3] = _decode_masked_channel(value, amask, 255) if has_alpha else 255

    return bytes(rgba), f"RGB{bpp}"


def decode_top_mip(data: bytes) -> tuple[int, int, bytes, str]:
    if len(data) < 128 or data[:4] != DDS_MAGIC:
        raise ValueError("input is not a classic DDS file")
    if struct.unpack_from("<I", data, 4)[0] != 124:
        raise ValueError("unsupported DDS header size")

    height = struct.unpack_from("<I", data, 12)[0]
    width = struct.unpack_from("<I", data, 16)[0]
    pf_size = struct.unpack_from("<I", data, 76)[0]
    pf_flags = struct.unpack_from("<I", data, 80)[0]
    fourcc = data[84:88]

    if pf_size != 32:
        raise ValueError("unsupported DDS pixel-format header size")
    if width < 1 or height < 1 or width > 16384 or height > 16384:
        raise ValueError(f"invalid DDS dimensions {width}x{height}")

    if pf_flags & DDPF_FOURCC:
        if fourcc == b"DXT1":
            block_size = 8
            decoder = _decode_dxt1
            format_name = "DXT1"
        elif fourcc == b"DXT3":
            block_size = 16
            decoder = _decode_dxt3
            format_name = "DXT3"
        elif fourcc == b"DXT5":
            block_size = 16
            decoder = _decode_dxt5
            format_name = "DXT5"
        else:
            raise ValueError(f"unsupported DDS FourCC {fourcc!r}; expected DXT1, DXT3, or DXT5")

        blocks_x = (width + 3) // 4
        blocks_y = (height + 3) // 4
        required = blocks_x * blocks_y * block_size
        payload = data[128:128 + required]
        if len(payload) != required:
            raise ValueError("compressed top mip is truncated")

        rgba = bytearray(width * height * 4)
        offset = 0
        for block_y in range(blocks_y):
            for block_x in range(blocks_x):
                pixels = decoder(payload[offset:offset + block_size])
                offset += block_size
                for py in range(4):
                    y = block_y * 4 + py
                    if y >= height:
                        continue
                    for px in range(4):
                        x = block_x * 4 + px
                        if x >= width:
                            continue
                        dst = (y * width + x) * 4
                        rgba[dst:dst + 4] = bytes(pixels[py * 4 + px])
        return width, height, bytes(rgba), format_name

    if pf_flags & DDPF_RGB:
        rgba, format_name = _decode_uncompressed_rgb(data, width, height, pf_flags)
        return width, height, rgba, format_name

    raise ValueError(f"unsupported classic DDS pixel flags 0x{pf_flags:x}")


def encode_bgra8_dds(width: int, height: int, rgba: bytes) -> bytes:
    if len(rgba) != width * height * 4:
        raise ValueError("RGBA payload size does not match dimensions")

    bgra = bytearray(len(rgba))
    for i in range(0, len(rgba), 4):
        r, g, b, a = rgba[i:i + 4]
        bgra[i:i + 4] = bytes((b, g, r, a))

    flags = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH | DDSD_PITCH | DDSD_PIXELFORMAT
    header = bytearray()
    header += struct.pack("<I", 124)
    header += struct.pack("<IIIIII", flags, height, width, width * 4, 0, 1)
    header += struct.pack("<11I", *([0] * 11))
    header += struct.pack(
        "<IIIIIIII",
        32,
        DDPF_RGB | DDPF_ALPHAPIXELS,
        0,
        32,
        *CANONICAL_MASKS,
    )
    header += struct.pack("<IIIII", DDSCAPS_TEXTURE, 0, 0, 0, 0)
    if len(header) != 124:
        raise AssertionError("DDS header assembly bug")
    return DDS_MAGIC + header + bytes(bgra)


def validate_output(data: bytes, width: int, height: int) -> None:
    if len(data) != 128 + width * height * 4:
        raise ValueError("transcoded DDS has unexpected size")
    if data[:4] != DDS_MAGIC:
        raise ValueError("transcoded DDS lost magic")
    if data[84:88] != b"\0\0\0\0":
        raise ValueError("transcoded DDS is still FourCC-compressed")
    if struct.unpack_from("<I", data, 88)[0] != 32:
        raise ValueError("transcoded DDS is not 32bpp")
    masks = tuple(struct.unpack_from("<I", data, offset)[0] for offset in (92, 96, 100, 104))
    if masks != CANONICAL_MASKS:
        raise ValueError("transcoded DDS channel masks are wrong")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    source = args.source.read_bytes()
    width, height, rgba, source_format = decode_top_mip(source)
    output = encode_bgra8_dds(width, height, rgba)
    validate_output(output, width, height)

    args.destination.parent.mkdir(parents=True, exist_ok=True)
    args.destination.write_bytes(output)
    print(
        f"{args.source.name}: {source_format} {width}x{height} -> "
        f"A8R8G8B8/BGRA8 DDS, rgba_sha256={hashlib.sha256(rgba).hexdigest()}"
    )


if __name__ == "__main__":
    main()
