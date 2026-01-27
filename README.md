# MocapUtility

A Python library for interfacing with OptiTrack motion capture systems via the NatNet protocol. Provides both low-level NatNet client access and a high-level utility for easy retrieval of rigid body positions, orientations, and velocities.

## Features

- **NatNet Protocol Support**: Compatible with NatNet versions 3.0, 4.0, and 4.1
- **Multicast and Unicast**: Supports both transmission modes
- **High-level API**: Simple methods to get position, quaternion, linear velocity, and angular velocity
- **Velocity Smoothing**: Configurable window size for velocity calculation
- **Auto IP Detection**: Automatically detects local IP address for server communication
- **Async Support**: Both synchronous and asynchronous data streaming
- **Remote Commands**: Control recording, playback, sessions, and assets on the OptiTrack server

## Requirements

- Python 3.10+
- OptiTrack Motive software with NatNet streaming enabled

## Installation

Clone or copy this package into your project:

```bash
git clone <repository-url> MocapUtility
```

## Quick Start

### Using MoCapUtility (Recommended)

```python
from MocapUtility import MoCapUtility
import time

# Auto-detect local IP
mocap = MoCapUtility(server_address="192.168.0.232")

# Or specify local IP manually
# mocap = MoCapUtility(server_address="192.168.0.232", local_ip_address="192.168.0.136")

with mocap:
    time.sleep(1)  # Wait for data

    # Get available rigid bodies
    robots = mocap.get_robot_names()
    print(f"Available robots: {robots}")

    for robot in robots:
        # Get position (x, y, z)
        pos = mocap.get_pos(robot)

        # Get quaternion (x, y, z, w)
        quat = mocap.get_quat(robot)

        # Get linear velocity (vx, vy, vz)
        lin_vel = mocap.get_lin_vel(robot)

        # Get angular velocity (roll_rate, pitch_rate, yaw_rate)
        ang_vel = mocap.get_ang_vel(robot)

        # Check tracking status
        tracking = mocap.get_tracking_status(robot)

        # Get time since last valid tracking
        staleness = mocap.get_position_staleness(robot)
```

### Using NatNetClient (Low-level)

```python
from MocapUtility.Client import NatNetClient, NatNetParams

params = NatNetParams(
    server_address="192.168.0.232",
    local_ip_address="192.168.0.136",
    use_multicast=True,
    multicast_address="239.255.42.99",
    command_port=1510,
    data_port=1511
)

with NatNetClient(params) as client:
    if client is not None:
        # Iterate over motion capture frames
        for frame in client.MoCap(timeout=10.0):
            for rb_id, rigid_body in frame.rigid_body_data.rigid_bodies_d.items():
                print(f"ID: {rb_id}, Pos: {rigid_body.pos}, Tracking: {rigid_body.tracking}")
```

### Async Usage

```python
import asyncio
from MocapUtility.Client import NatNetClient, NatNetParams

async def main():
    params = NatNetParams(server_address="192.168.0.232", local_ip_address="192.168.0.136")

    with NatNetClient(params) as client:
        if client is None:
            return
        async for frame in client.MoCapAsync(timeout=10.0):
            print(f"Frame: {frame.prefix_data.frame_number}")

asyncio.run(main())
```

## API Reference

### MoCapUtility

| Method | Description |
|--------|-------------|
| `connect()` | Connect to the OptiTrack server |
| `disconnect()` | Disconnect from the server |
| `get_robot_names()` | Get list of available rigid body names |
| `get_pos(robot_name)` | Get position as (x, y, z) tuple |
| `get_quat(robot_name)` | Get quaternion as (x, y, z, w) tuple |
| `get_lin_vel(robot_name)` | Get linear velocity as (vx, vy, vz) tuple |
| `get_ang_vel(robot_name)` | Get angular velocity as (roll_rate, pitch_rate, yaw_rate) tuple |
| `get_tracking_status(robot_name)` | Get tracking status (True/False) |
| `get_position_staleness(robot_name)` | Get time since last valid tracking update |

### NatNetClient Remote Commands

| Method | Description |
|--------|-------------|
| `StartRecording()` | Start recording in Motive |
| `StopRecording()` | Stop recording in Motive |
| `LiveMode()` | Switch to live mode |
| `EditMode()` | Switch to edit mode |
| `TimelinePlay()` | Play timeline |
| `TimelineStop()` | Stop timeline |
| `CurrentMode()` | Get current mode (live/recording/playback/edit) |
| `FrameRate()` | Get current frame rate |
| `UnitesToMillimeters()` | Get units to millimeters conversion factor |
| `SetRecordTakeName(name)` | Set the name for the next recording take |
| `SetPlaybackTakeName(name)` | Set the playback take |
| `EnableAsset(name)` | Enable an asset |
| `DisableAsset(name)` | Disable an asset |

## Configuration

### NatNetParams

| Parameter | Default | Description |
|-----------|---------|-------------|
| `server_address` | "127.0.0.1" | IP address of the Motive computer |
| `local_ip_address` | "127.0.0.1" | Local IP for receiving data (None for auto-detect) |
| `use_multicast` | True | Use multicast mode (must match Motive settings) |
| `multicast_address` | "239.255.42.99" | Multicast address (check Motive settings) |
| `command_port` | 1510 | Command port (check Motive settings) |
| `data_port` | 1511 | Data port (check Motive settings) |
| `max_buffer_size` | None | Max size for server messages buffer |
| `connection_timeout` | None | Timeout for server connection |

## Motive Setup

1. Open OptiTrack Motive software
2. Go to **View > Data Streaming Pane**
3. Enable **Broadcast Frame Data**
4. Configure:
   - **Local Interface**: Your computer's IP
   - **Transmission Type**: Multicast or Unicast
   - **Command Port**: 1510 (default)
   - **Data Port**: 1511 (default)
   - **Multicast Interface**: 239.255.42.99 (default)

## Data Types

### Position
```python
@dataclass(frozen=True)
class Position:
    x: float
    y: float
    z: float
```

### Quaternion
```python
@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float
    roll: float   # Computed from quaternion
    pitch: float  # Computed from quaternion
    yaw: float    # Computed from quaternion
```

## Supported Data Types

- Rigid Bodies (position, orientation, tracking status)
- Markers (labeled and unlabeled)
- Skeletons
- Force Plates
- Devices
- Assets (NatNet 4.1+)

## NatNet Protocol Version Differences

This library supports NatNet versions 3.0, 4.0, and 4.1. The client automatically detects the server's protocol version and selects the appropriate unpacker. However, understanding the differences is important for troubleshooting.

### Version Detection

The protocol version is determined from the server's response during connection:
- The client sends a `CONNECT` request with version `[4, 1, 0, 0]`
- The server responds with its `nat_net_major` and `nat_net_minor` version
- The appropriate unpacker is selected based on these values

### Key Differences Between Versions

| Feature | NatNet 3.0 | NatNet 4.0 | NatNet 4.1 |
|---------|------------|------------|------------|
| Data size fields in frames | No | No | Yes |
| Asset data support | No | No | Yes |
| Precision timestamps | No | No | Yes |
| Marker names in rigid body descriptors | No | Yes | Yes |
| Size field after descriptor tag | No | No | Yes |
| Frame suffix size | 42 bytes | 42 bytes | 50 bytes |

### Detailed Version Behavior

#### NatNet 3.0 (`DataUnpackerV3_0`)
- **Frame Data**: No size fields preceding data sections; parser relies on count fields
- **Descriptors**: No marker names in rigid body descriptions; no size field after descriptor type tag
- **Frame Suffix**: 42 bytes (no precision timestamp fields)
- **Assets**: Not supported; `asset_data` will be `None` in MoCap frames

#### NatNet 4.0 (`DataUnpackerV4_0`)
- **Frame Data**: Same as 3.0 (no size fields, no asset data)
- **Descriptors**: Includes marker names in rigid body descriptions (like 4.1), but no size field after descriptor type tag (like 3.0)
- **Frame Suffix**: 42 bytes (same as 3.0)
- **Assets**: Not supported in frame data
- **Note**: This is a transitional version - a hybrid between 3.0 frame format and 4.1 descriptor format

#### NatNet 4.1 (`DataUnpackerV4_1`)
- **Frame Data**: Size fields precede each data section for validation/skipping
- **Descriptors**: Full marker names; size field after each descriptor type tag
- **Frame Suffix**: 50 bytes (includes `precision_timestamp_sec` and `precision_timestamp_frac_sec`)
- **Assets**: Fully supported; `asset_data` contains asset rigid bodies and markers

### Motive Version Compatibility

| Motive Version | NatNet Version | Notes |
|----------------|----------------|-------|
| Motive 2.x | NatNet 3.0 | Legacy format |
| Motive 3.0 | NatNet 4.0 | Transitional format |
| Motive 3.1+ | NatNet 4.1 | Current format with full features |

### Checking Protocol Version

You can check which protocol version is being used after connection:

```python
with NatNetClient(params) as client:
    if client is not None:
        print(f"Server: {client.server_info.application_name}")
        print(f"NatNet Version: {client.server_info.nat_net_major}.{client.server_info.nat_net_minor}")
```

## Command Line Arguments

The `NatNetParams` class provides argparse integration:

```python
import argparse
from MocapUtility.Client import NatNetParams

parser = argparse.ArgumentParser()
NatNetParams.argparse_group(parser)
args = parser.parse_args()

params = NatNetParams.from_parser(args)
```

## Troubleshooting

### Connection Issues

- **No data received**: Ensure Motive is streaming and firewall allows UDP traffic on ports 1510/1511
- **Connection timeout**: Verify server IP address and that both computers are on the same network
- **No rigid bodies found**: Make sure rigid bodies are created and enabled in Motive
- **Multiple interfaces error**: Specify `local_ip_address` explicitly if auto-detection finds multiple valid interfaces

### NatNet Version Issues

- **Garbled or missing data**: Protocol version mismatch. Check `client.server_info.nat_net_major` and `nat_net_minor` after connection. If the version is unexpected, ensure Motive is updated or check if the unpacker supports your version.

- **`asset_data` is None**: Assets are only supported in NatNet 4.1+. If using Motive 3.0 or earlier, asset data will not be available.

- **Marker names missing in rigid body descriptions**: Marker names in descriptors require NatNet 4.0+. With NatNet 3.0, marker names will be empty strings.

- **Precision timestamps are None**: Precision timestamps (`precision_timestamp_sec`, `precision_timestamp_frac_sec`) are only available in NatNet 4.1+. Earlier versions will have these fields as `None`.

- **Frame parsing errors or struct unpack failures**: This typically indicates a version mismatch where the unpacker expects a different frame format than what the server sends. Common causes:
  - Server reports incorrect version
  - Custom or modified NatNet implementation
  - Network packet corruption

  **Workaround**: You can manually inspect the raw server info to debug:
  ```python
  with NatNetClient(params) as client:
      if client:
          info = client.server_info
          print(f"App: {info.application_name}")
          print(f"App Version: {info.version}")
          print(f"NatNet: {info.nat_net_major}.{info.nat_net_minor}")
  ```

- **Data appears correct but asset/skeleton counts are wrong**: NatNet 4.1 changed the frame data order - assets come before labeled markers. If you're seeing incorrect counts, verify the NatNet version matches expectations.

### Version-Specific Workarounds

If you encounter persistent version issues:

1. **Verify Motive version**: Check Help > About in Motive to confirm the expected NatNet version
2. **Check streaming settings**: In Motive's Data Streaming pane, some versions allow selecting the NatNet version
3. **Update Motive**: Newer versions of Motive generally have better NatNet compatibility
4. **Test with OptiTrack samples**: Use OptiTrack's official NatNet SDK samples to verify your setup works before debugging this library

## License

MIT License
