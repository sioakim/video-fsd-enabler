# Tesla Dashcam FSD Enabler

> **Disclaimer:** This is a novelty/joke tool. It does **not** enable Full Self-Driving on your Tesla. It only modifies the metadata overlay displayed in dashcam viewers — your car's actual driving capabilities are completely unchanged. Use responsibly and don't mislead anyone with modified footage.

Read and modify Tesla dashcam SEI metadata to change the autopilot/FSD display state.

Tesla dashcam videos embed telemetry data (speed, steering, GPS, autopilot state, etc.) in H.264 SEI NAL units using a custom protobuf format. This tool reads that metadata and can modify the `autopilot_state` field to show FSD, Autosteer, TACC, or None.

## How it works

Tesla encodes vehicle telemetry in **SEI (Supplemental Enhancement Information)** messages within the H.264 video stream. Each frame contains a protobuf-serialized `SeiMetadata` message prefixed with 4 magic bytes (`0x42 0x42 0x42 0x69` = "BBBi").

This tool:
1. Parses the MP4 container to find the `mdat` atom
2. Scans for H.264 NAL units of type 6 (SEI)
3. Finds Tesla-prefixed payloads within SEI type 5 (user data unregistered)
4. Deserializes the protobuf, modifies `autopilot_state`, re-serializes
5. Writes the modified data back in-place (same file size)

## Installation

```bash
pip install protobuf
```

## Usage

### Read metadata (no modifications)

```bash
python fsd_enabler.py video.mp4 --read-only
```

Output:
```
Found 1800 metadata frames

  Frame     Speed      Gear       Autopilot   Steering  Throttle  Brake         Lat          Lon
----------------------------------------------------------------------------------------------------
  31500   45.2mph     Drive            None      -2.3°       15%     no   37.774900  -122.419400
  31501   45.1mph     Drive            None      -2.1°       14%     no   37.774910  -122.419380
  ...

Autopilot state distribution:
  None: 1800 frames (100%)
```

### Enable FSD on all frames

```bash
python fsd_enabler.py input.mp4 output.mp4
```

### Set to a specific mode

```bash
python fsd_enabler.py input.mp4 output.mp4 -m fsd         # Full Self-Driving
python fsd_enabler.py input.mp4 output.mp4 -m autosteer   # Autosteer
python fsd_enabler.py input.mp4 output.mp4 -m tacc        # Traffic-Aware Cruise Control
python fsd_enabler.py input.mp4 output.mp4 -m none        # Remove autopilot state
```

### Modify in-place (overwrites original!)

```bash
python fsd_enabler.py input.mp4
```

## Tesla SEI Metadata Fields

| Field | Type | Description |
|-------|------|-------------|
| `version` | uint32 | Protocol version |
| `gear_state` | enum | Park, Drive, Reverse, Neutral |
| `frame_seq_no` | uint64 | Sequential frame number |
| `vehicle_speed_mps` | float | Speed in meters/second |
| `accelerator_pedal_position` | float | 0.0 (off) to 1.0 (full) |
| `steering_wheel_angle` | float | Degrees (+ = right, - = left) |
| `blinker_on_left` | bool | Left turn signal |
| `blinker_on_right` | bool | Right turn signal |
| `brake_applied` | bool | Brake pedal pressed |
| `autopilot_state` | enum | None, Self-Driving, Autosteer, TACC |
| `latitude_deg` | double | GPS latitude |
| `longitude_deg` | double | GPS longitude |
| `heading_deg` | double | Vehicle heading (0=North) |
| `linear_acceleration_mps2_x` | double | Lateral acceleration |
| `linear_acceleration_mps2_y` | double | Longitudinal acceleration |
| `linear_acceleration_mps2_z` | double | Vertical acceleration |

## Requirements

- Python 3.10+
- `protobuf` package
- Tesla dashcam video with SEI metadata (firmware 2025.44.25+)

## Viewing Modified Videos

Use [Sentry Clip Viewer](https://apps.apple.com/app/id6756430882) on iPhone, iPad, or Mac to view dashcam footage with the telemetry overlay. The FSD indicator will show the modified autopilot state.

## License

MIT
