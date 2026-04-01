#!/usr/bin/env python3
"""
Tesla Dashcam FSD Enabler

Reads Tesla dashcam MP4 files and modifies the autopilot_state field
in every SEI metadata frame. Uses the same extraction logic as Tesla's
official sei_extractor.py to ensure 100% compatibility.

Usage:
    python fsd_enabler.py input.mp4 output.mp4
    python fsd_enabler.py input.mp4 --read-only
    python fsd_enabler.py input.mp4 output.mp4 -m autosteer
"""

import argparse
import struct
import sys
import shutil
from pathlib import Path
from typing import Generator, Optional, Tuple, List

from google.protobuf.message import DecodeError
from google.protobuf.json_format import MessageToDict

import dashcam_pb2

AUTOPILOT_MODES = {
    "none": 0, "fsd": 1, "self_driving": 1, "autosteer": 2, "tacc": 3,
}
AUTOPILOT_NAMES = {0: "None", 1: "FSD (Self-Driving)", 2: "Autosteer", 3: "TACC"}

# Protobuf field 10 (autopilot_state): tag = (10 << 3) | 0 = 0x50
FIELD_10_TAG = 0x50


# ── MP4 / NAL parsing (from Tesla's sei_extractor.py) ──────────────

def find_mdat(fp) -> Tuple[int, int]:
    fp.seek(0)
    while True:
        header = fp.read(8)
        if len(header) < 8:
            raise RuntimeError("mdat atom not found")
        size32, atom_type = struct.unpack(">I4s", header)
        if size32 == 1:
            large = fp.read(8)
            if len(large) != 8:
                raise RuntimeError("truncated extended atom size")
            atom_size = struct.unpack(">Q", large)[0]
            header_size = 16
        else:
            atom_size = size32 if size32 else 0
            header_size = 8
        if atom_type == b"mdat":
            payload_size = atom_size - header_size if atom_size else 0
            return fp.tell(), payload_size
        if atom_size < header_size:
            raise RuntimeError("invalid MP4 atom size")
        fp.seek(atom_size - header_size, 1)


def iter_sei_nal_offsets(fp, offset: int, size: int) -> Generator[Tuple[int, int], None, None]:
    """Yield (file_offset, nal_size) for each SEI user-data NAL unit."""
    fp.seek(offset)
    consumed = 0
    while size == 0 or consumed < size:
        pos = fp.tell()
        header = fp.read(4)
        if len(header) < 4:
            break
        nal_size = struct.unpack(">I", header)[0]
        if nal_size < 2:
            fp.seek(nal_size, 1)
            consumed += 4 + nal_size
            continue

        first_two = fp.read(2)
        if len(first_two) != 2:
            break

        if (first_two[0] & 0x1F) != 6 or first_two[1] != 5:
            fp.seek(nal_size - 2, 1)
            consumed += 4 + nal_size
            continue

        # This is a SEI user-data NAL — record its position
        fp.seek(nal_size - 2, 1)
        consumed += 4 + nal_size
        yield (pos + 4, nal_size)  # offset of NAL data (after length prefix)


def strip_emulation_prevention_bytes(data: bytes) -> bytes:
    stripped = bytearray()
    zero_count = 0
    for byte in data:
        if zero_count >= 2 and byte == 0x03:
            zero_count = 0
            continue
        stripped.append(byte)
        zero_count = 0 if byte != 0 else zero_count + 1
    return bytes(stripped)


def extract_proto_payload(nal: bytes) -> Optional[bytes]:
    """Extract protobuf from SEI NAL (Tesla's method)."""
    if not isinstance(nal, bytes) or len(nal) < 2:
        return None
    for i in range(3, len(nal) - 1):
        byte = nal[i]
        if byte == 0x42:
            continue
        if byte == 0x69:
            if i > 2:
                return strip_emulation_prevention_bytes(nal[i + 1:-1])
            break
        break
    return None


def find_bbbi_in_nal(nal: bytes) -> int:
    """Find the offset of 0x69 (end of BBBi prefix) within a NAL unit."""
    for i in range(3, len(nal) - 1):
        byte = nal[i]
        if byte == 0x42:
            continue
        if byte == 0x69:
            if i > 2:
                return i
            break
        break
    return -1


# ── Read metadata ──────────────────────────────────────────────────

def read_metadata(filepath: Path) -> List[dashcam_pb2.SeiMetadata]:
    frames = []
    with open(filepath, "rb") as fp:
        offset, size = find_mdat(fp)
        for nal_offset, nal_size in iter_sei_nal_offsets(fp, offset, size):
            fp.seek(nal_offset)
            nal = fp.read(nal_size)
            payload = extract_proto_payload(nal)
            if not payload:
                continue
            meta = dashcam_pb2.SeiMetadata()
            try:
                meta.ParseFromString(payload)
                frames.append(meta)
            except DecodeError:
                continue
    return frames


# ── Modify video ───────────────────────────────────────────────────

def modify_video(input_path: Path, output_path: Path, new_state: int) -> int:
    """Modify autopilot_state in all SEI frames.

    Strategy: For each SEI NAL, find the protobuf region (after 0x69 byte,
    before the last NAL byte). Deserialize, modify autopilot_state, re-serialize.
    If the new protobuf fits in the same space (with emulation prevention bytes
    re-added), write it back. Otherwise skip the frame.
    """
    # Read entire file into mutable bytearray
    data = bytearray(input_path.read_bytes())
    modified = 0
    skipped = 0

    # Find all SEI NAL positions
    with open(input_path, "rb") as fp:
        mdat_offset, mdat_size = find_mdat(fp)
        nal_positions = list(iter_sei_nal_offsets(fp, mdat_offset, mdat_size))

    for nal_offset, nal_size in nal_positions:
        nal = bytes(data[nal_offset:nal_offset + nal_size])

        # Find BBBi end position within the NAL
        bbbi_end = find_bbbi_in_nal(nal)
        if bbbi_end < 0:
            continue

        # The protobuf region is: nal[bbbi_end+1 : nal_size-1]
        # (skip the 0x69 byte, exclude the last RBSP byte)
        proto_region_start = bbbi_end + 1  # relative to NAL start
        proto_region_end = nal_size - 1     # exclude trailing RBSP stop bit
        proto_region_len = proto_region_end - proto_region_start

        if proto_region_len < 10:
            continue

        raw_proto_region = nal[proto_region_start:proto_region_end]

        # Strip emulation prevention bytes to get clean protobuf
        clean_proto = strip_emulation_prevention_bytes(raw_proto_region)

        # Parse
        meta = dashcam_pb2.SeiMetadata()
        try:
            meta.ParseFromString(clean_proto)
        except DecodeError:
            continue

        # Modify autopilot state
        meta.autopilot_state = new_state

        # Adding autopilot_state (non-zero) adds ~2 bytes to the protobuf.
        # To fit within the same NAL space, zero out near-zero float fields.
        # Proto3 doesn't encode 0.0 on the wire, so setting tiny values
        # (like -2.48e-07) to exactly 0.0 reclaims their 5 bytes.
        if abs(meta.vehicle_speed_mps) < 0.001:
            meta.vehicle_speed_mps = 0.0
        if abs(meta.accelerator_pedal_position) < 0.001:
            meta.accelerator_pedal_position = 0.0
        if abs(meta.steering_wheel_angle) < 0.001:
            meta.steering_wheel_angle = 0.0

        new_clean_proto = meta.SerializeToString()
        new_raw_proto = add_emulation_prevention_bytes(new_clean_proto)

        size_diff = len(new_raw_proto) - proto_region_len

        if size_diff <= 0:
            # Fits! Pad clean protobuf with unknown field bytes so Tesla's
            # extractor (reads nal[bbbi+1:-1]) gets a valid protobuf.
            # Field 31 varint 0 = 0xF8 0x01 0x00 — ignored by parsers.
            padded_clean = new_clean_proto
            for _ in range(20):  # max 20 padding iterations
                padded_raw = add_emulation_prevention_bytes(padded_clean)
                if len(padded_raw) >= proto_region_len:
                    break
                padded_clean += b'\xf8\x01\x00'

            padded_raw = add_emulation_prevention_bytes(padded_clean)

            if len(padded_raw) == proto_region_len:
                abs_start = nal_offset + proto_region_start
                data[abs_start:abs_start + proto_region_len] = padded_raw
                modified += 1
            elif len(padded_raw) < proto_region_len:
                # Append one fewer padding byte — try 2-byte padding
                padded_clean2 = new_clean_proto
                for _ in range(20):
                    test = add_emulation_prevention_bytes(padded_clean2)
                    if len(test) >= proto_region_len:
                        break
                    padded_clean2 += b'\xf8\x01'  # 2-byte padding variant
                test = add_emulation_prevention_bytes(padded_clean2)
                if len(test) <= proto_region_len:
                    abs_start = nal_offset + proto_region_start
                    # Fill remaining with original bytes
                    final = test + raw_proto_region[len(test):]
                    data[abs_start:abs_start + proto_region_len] = final[:proto_region_len]
                    modified += 1
                else:
                    skipped += 1
            else:
                # Overshot — remove last padding and fill with original
                padded_clean = padded_clean[:-3]
                padded_raw = add_emulation_prevention_bytes(padded_clean)
                abs_start = nal_offset + proto_region_start
                final = padded_raw + raw_proto_region[len(padded_raw):]
                data[abs_start:abs_start + proto_region_len] = final[:proto_region_len]
                modified += 1
        elif size_diff <= 4:
            # Grew by 1-4 bytes. The bytes AFTER the protobuf region
            # (before RBSP stop bit) are part of the NAL but outside the
            # SEI payload. We can expand the SEI payload size byte to
            # claim those bytes, making room for the larger protobuf.
            #
            # NAL structure: [06] [05] [SEI_SIZE] [BBBi] [proto] [trailing] [80]
            # The SEI_SIZE byte is at nal offset 2 (absolute: nal_offset + 2)
            #
            # Tesla's extractor reads nal[bbbi_end+1:-1] which includes
            # everything up to RBSP. So as long as we write valid protobuf
            # bytes there, it works.

            # Write the new protobuf, overwriting into the trailing region
            abs_start = nal_offset + proto_region_start
            end_pos = abs_start + len(new_raw_proto)
            if end_pos <= nal_offset + nal_size - 1:  # Don't overwrite RBSP stop bit
                data[abs_start:end_pos] = new_raw_proto

                # Update SEI payload size byte
                sei_size_offset = nal_offset + 2  # byte 2 in the NAL
                old_sei_size = data[sei_size_offset]
                new_sei_size = old_sei_size + size_diff
                if new_sei_size < 255:
                    data[sei_size_offset] = new_sei_size
                    modified += 1
                else:
                    # SEI size would overflow single byte — can't handle
                    skipped += 1
            else:
                skipped += 1
        elif size_diff <= 4:
            # Proto grew but only by 1-4 bytes. Update the SEI payload size byte
            # to claim the extra bytes from the trailing NAL data.
            # The SEI size byte is at nal offset 2 (NAL: [06] [05] [SIZE] [BBBi]...)
            sei_size_pos = nal_offset + 2
            old_sei_size = data[sei_size_pos]

            # Check there's room in the NAL (trailing bytes we can absorb)
            end_pos = nal_offset + proto_region_start + len(new_raw_proto)
            if end_pos <= nal_offset + nal_size - 1 and old_sei_size + size_diff < 255:
                # Write the new protobuf (overwriting trailing bytes)
                abs_start = nal_offset + proto_region_start
                data[abs_start:abs_start + len(new_raw_proto)] = new_raw_proto
                # Update SEI payload size
                data[sei_size_pos] = old_sei_size + size_diff
                modified += 1
            else:
                skipped += 1
        else:
            skipped += 1

    if skipped > 0:
        print(f"Note: {skipped} frames skipped (protobuf too large after modification)")

    output_path.write_bytes(bytes(data))
    return modified


def add_emulation_prevention_bytes(data: bytes) -> bytes:
    """Add H.264 emulation prevention bytes (00 00 -> 00 00 03)."""
    result = bytearray()
    zero_count = 0
    for byte in data:
        if zero_count >= 2 and byte in (0x00, 0x01, 0x02, 0x03):
            result.append(0x03)
            zero_count = 0
        result.append(byte)
        zero_count = zero_count + 1 if byte == 0 else 0
    return bytes(result)


# ── Display ────────────────────────────────────────────────────────

def update_chunk_offsets(data: bytearray, insertion_point: int, size_diff: int):
    """Update stco/co64 chunk offset tables after mdat size change."""
    i = 0
    while i < len(data) - 8:
        size = struct.unpack(">I", data[i:i+4])[0]
        box_type = data[i+4:i+8]
        if size < 8 or size > len(data) - i:
            break
        if box_type == b"moov":
            _fix_offsets_in_box(data, i + 8, i + size, insertion_point, size_diff)
            break
        if size == 1:
            size = struct.unpack(">Q", data[i+8:i+16])[0]
        i += size


def _fix_offsets_in_box(data: bytearray, start: int, end: int, insertion_point: int, size_diff: int):
    i = start
    while i < end - 8:
        size = struct.unpack(">I", data[i:i+4])[0]
        if size < 8 or i + size > end:
            break
        box_type = data[i+4:i+8]

        if box_type == b"stco":
            count = struct.unpack(">I", data[i+12:i+16])[0]
            for j in range(count):
                pos = i + 16 + j * 4
                if pos + 4 <= end:
                    off = struct.unpack(">I", data[pos:pos+4])[0]
                    if off > insertion_point:
                        data[pos:pos+4] = struct.pack(">I", off + size_diff)

        elif box_type == b"co64":
            count = struct.unpack(">I", data[i+12:i+16])[0]
            for j in range(count):
                pos = i + 16 + j * 8
                if pos + 8 <= end:
                    off = struct.unpack(">Q", data[pos:pos+8])[0]
                    if off > insertion_point:
                        data[pos:pos+8] = struct.pack(">Q", off + size_diff)

        elif box_type in (b"moov", b"trak", b"mdia", b"minf", b"stbl"):
            _fix_offsets_in_box(data, i + 8, i + size, insertion_point, size_diff)

        i += size


def modify_video_remux(input_path: Path, output_path: Path, new_state: int) -> int:
    """Modify all frames by rebuilding NAL units and remuxing with ffmpeg.

    This method achieves 100% frame modification by rebuilding each SEI NAL
    with the correct size, then using ffmpeg to repackage into MP4 with
    proper length prefixes and chunk offsets.
    """
    import subprocess, tempfile

    data = input_path.read_bytes()

    with open(input_path, "rb") as fp:
        mdat_offset, mdat_size = find_mdat(fp)

    # Get framerate and duration from original
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=r_frame_rate,duration",
         "-of", "csv=p=0", str(input_path)],
        capture_output=True, text=True
    )
    parts = probe.stdout.strip().split(",")
    fps_str = parts[0] if parts else "36/1"
    # Convert fraction like "72066/2003" to decimal
    if "/" in fps_str:
        num, den = fps_str.split("/")
        fps = str(round(int(num) / int(den), 3))
    else:
        fps = fps_str or "36"

    # Read all NAL units from mdat
    nals = []
    pos = mdat_offset
    end = mdat_offset + mdat_size
    while pos < end - 4:
        nal_len = struct.unpack(">I", data[pos:pos + 4])[0]
        if nal_len <= 0 or pos + 4 + nal_len > end:
            pos += 1
            continue
        nals.append(data[pos + 4:pos + 4 + nal_len])
        pos += 4 + nal_len

    # Modify SEI NALs
    modified = 0
    for i, nal in enumerate(nals):
        nal_type = nal[0] & 0x1F
        if nal_type != 6 or len(nal) < 2 or nal[1] != 5:
            continue

        bbbi_end = find_bbbi_in_nal(nal)
        if bbbi_end < 0:
            continue

        raw_region = nal[bbbi_end + 1:-1]
        clean = strip_emulation_prevention_bytes(raw_region)

        meta = dashcam_pb2.SeiMetadata()
        try:
            meta.ParseFromString(clean)
        except DecodeError:
            continue

        meta.autopilot_state = new_state
        new_clean = meta.SerializeToString()
        new_raw = add_emulation_prevention_bytes(new_clean)

        # Rebuild NAL with correct SEI payload size
        # SEI payload size = bytes in the RBSP (after EPB removal).
        # Use clean protobuf length, not raw (which includes EPB bytes).
        new_sei_size = 4 + len(new_clean)  # BBBi prefix + clean protobuf
        sei_size_bytes = bytearray()
        s = new_sei_size
        while s >= 255:
            sei_size_bytes.append(0xFF)
            s -= 255
        sei_size_bytes.append(s)

        new_nal = bytearray()
        new_nal.append(nal[0])        # NAL header
        new_nal.append(5)             # SEI type
        new_nal.extend(sei_size_bytes)
        new_nal.extend(b'\x42\x42\x42\x69')
        new_nal.extend(new_raw)
        new_nal.append(0x80)          # RBSP stop bit

        nals[i] = bytes(new_nal)
        modified += 1

    # Write raw H.264 bitstream (Annex B)
    with tempfile.NamedTemporaryFile(suffix=".h264", delete=False) as tmp:
        h264_path = tmp.name
        for nal in nals:
            tmp.write(b'\x00\x00\x00\x01')
            tmp.write(nal)

    # Write raw H.264 Annex B bitstream, then use ffmpeg to remux into MP4.
    # ffmpeg handles all NAL length prefixes and moov/stco tables correctly.
    with tempfile.NamedTemporaryFile(suffix=".h264", delete=False) as tmp:
        h264_path = tmp.name
        for nal in nals:
            tmp.write(b'\x00\x00\x00\x01')
            tmp.write(nal)

    # Get original duration for timestamp calculation
    probe = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=nb_frames,duration",
         "-of", "csv=p=0", str(input_path)],
        capture_output=True, text=True
    )
    parts = probe.stdout.strip().split(",")
    nb_frames = parts[0] if parts else "1872"
    duration = parts[1] if len(parts) > 1 else "52"
    # Calculate precise fps
    try:
        fps_calc = str(round(int(nb_frames) / float(duration), 6))
    except (ValueError, ZeroDivisionError):
        fps_calc = "36"

    # Remux: ffmpeg reads raw H.264 with correct framerate, outputs MP4
    result = subprocess.run([
        "ffmpeg", "-y",
        "-r", fps_calc,
        "-f", "h264",
        "-i", h264_path,
        "-c:v", "copy",
        "-movflags", "+faststart",
        str(output_path)
    ], capture_output=True, text=True)

    Path(h264_path).unlink(missing_ok=True)

    Path(h264_path).unlink(missing_ok=True)

    if result.returncode != 0:
        print(f"ffmpeg error: {result.stderr[-300:]}")
        return 0

    return modified


def print_metadata(frames):
    if not frames:
        print("No Tesla SEI metadata found.")
        return
    print(f"\nFound {len(frames)} metadata frames\n")
    print(f"{'Frame':>7}  {'Speed':>8}  {'Gear':>8}  {'Autopilot':>18}  {'Steering':>10}  {'Throttle':>8}  {'Brake':>5}  {'Lat':>10}  {'Lon':>11}")
    print("-" * 105)
    for m in frames[:20]:
        mph = m.vehicle_speed_mps * 2.237
        gear = ["Park", "Drive", "Reverse", "Neutral"][m.gear_state] if 0 <= m.gear_state <= 3 else "?"
        ap = AUTOPILOT_NAMES.get(m.autopilot_state, f"?({m.autopilot_state})")
        thr = f"{m.accelerator_pedal_position * 100:.0f}%"
        print(f"{m.frame_seq_no:>7}  {mph:>6.1f}mph  {gear:>8}  {ap:>18}  {m.steering_wheel_angle:>8.1f}°  {thr:>8}  {'YES' if m.brake_applied else 'no':>5}  {m.latitude_deg:>10.6f}  {m.longitude_deg:>11.6f}")
    if len(frames) > 20:
        print(f"  ... and {len(frames) - 20} more frames")
    counts = {}
    for f in frames:
        s = AUTOPILOT_NAMES.get(f.autopilot_state, "?")
        counts[s] = counts.get(s, 0) + 1
    print(f"\nAutopilot distribution:")
    for s, c in sorted(counts.items(), key=lambda x: -x[1]):
        print(f"  {s}: {c} ({c/len(frames)*100:.0f}%)")


# ── Main ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Tesla Dashcam FSD Enabler")
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("-m", "--mode", choices=list(AUTOPILOT_MODES.keys()), default="fsd")
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--no-remux", action="store_true",
                        help="Use in-place patching instead of ffmpeg remux (faster but may skip frames)")
    args = parser.parse_args()

    if not args.input.exists():
        print(f"Error: {args.input} not found"); sys.exit(1)

    if args.read_only:
        print(f"Reading: {args.input}")
        print_metadata(read_metadata(args.input))
        return

    output = args.output or args.input
    new_state = AUTOPILOT_MODES[args.mode]

    print(f"Input:  {args.input}")
    print(f"Output: {output}")
    print(f"Mode:   {AUTOPILOT_NAMES[new_state]}\n")

    before = read_metadata(args.input)
    if not before:
        print("No Tesla SEI metadata found."); sys.exit(1)

    if args.no_remux:
        # In-place patching (fast, may skip frames with non-zero speed)
        if output != args.input:
            shutil.copy2(args.input, output)
            count = modify_video(output, output, new_state)
        else:
            count = modify_video(args.input, output, new_state)
    else:
        # ffmpeg remux (slower, 100% frame coverage)
        count = modify_video_remux(args.input, output, new_state)

    after = read_metadata(output)

    def dist(fr):
        c = {}
        for f in fr:
            c[AUTOPILOT_NAMES.get(f.autopilot_state, "?")] = c.get(AUTOPILOT_NAMES.get(f.autopilot_state, "?"), 0) + 1
        return c

    print(f"\nModified {count}/{len(before)} frames")
    print(f"Before: {dist(before)}")
    print(f"After:  {dist(after)}")

    # Validate with Tesla's extraction method
    ok = len(after)
    total = len(before)
    if ok < total * 0.95:
        print(f"\n⚠️  Warning: only {ok}/{total} frames readable after modification ({ok/total*100:.0f}%)")
    else:
        print(f"\n✅ Validation: {ok}/{total} frames readable ({ok/total*100:.0f}%)")

    print(f"\nSaved: {output}")


if __name__ == "__main__":
    main()
