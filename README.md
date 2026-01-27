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

- **No data received**: Ensure Motive is streaming and firewall allows UDP traffic on ports 1510/1511
- **Connection timeout**: Verify server IP address and that both computers are on the same network
- **No rigid bodies found**: Make sure rigid bodies are created and enabled in Motive
- **Multiple interfaces error**: Specify `local_ip_address` explicitly if auto-detection finds multiple valid interfaces

## License

MIT License
