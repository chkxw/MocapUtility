"""
OptiTrack Motion Capture Utility
Simple layer for reading mocap data from OptiTrack server using new_natnet_client.
Provides interfaces for position, quaternion, linear velocity, and angular velocity.
"""

from .Client import NatNetClient, NatNetParams
from .NatNetTypes import Position, Quaternion, NAT_Messages
from typing import Optional, Dict, Tuple, List
import threading
import time
import math
import socket
from dataclasses import dataclass, field
from collections import deque


@dataclass
class RigidBodyState:
    """Store rigid body state including position, rotation, tracking status, and calculated velocities"""

    timestamp: float  # Current timestamp (always updated)
    last_tracked_timestamp: float  # Timestamp when position was last updated with tracking=True
    position: Position
    quaternion: Quaternion
    tracking: bool  # True if currently tracked, False if track lost
    linear_velocity: Optional[Position] = None
    angular_velocity: Optional[Position] = (
        None  # Angular velocity as roll, pitch, yaw rates
    )


class MoCapUtility:
    """
    Motion Capture utility for OptiTrack server.
    Provides simple interface to get robot position, quaternion, and velocities.
    """

    def __init__(
        self, server_address: str, local_ip_address: Optional[str] = None, velocity_window_size: int = 5
    ):
        """
        Initialize MoCap utility.

        Args:
            server_address: IP address of the OptiTrack server
            local_ip_address: Local IP address for receiving data. If None, will auto-detect
            velocity_window_size: Number of samples to use for velocity calculation (smoothing)
        """
        # Auto-detect local IP if not provided
        if local_ip_address is None:
            local_ip_address = self._detect_local_ip(server_address)
            print(f"Auto-detected local IP address: {local_ip_address}")
        
        self.params = NatNetParams(
            server_address=server_address, local_ip_address=local_ip_address
        )
        self.client: Optional[NatNetClient] = None
        self.running = False
        self.data_thread: Optional[threading.Thread] = None

        # Store robot data: robot_name/id -> deque of RigidBodyState
        self.robot_states: Dict[str, deque] = {}
        self.robot_id_to_name: Dict[int, str] = {}
        self.velocity_window_size = velocity_window_size
        self._lock = threading.Lock()

    def _detect_local_ip(self, server_address: str, command_port: int = 1510) -> str:
        """
        Auto-detect the local IP address that can reach the server.
        
        Args:
            server_address: IP address of the OptiTrack server
            command_port: Port to test connectivity (default 1510)
            
        Returns:
            Local IP address that can reach the server
            
        Raises:
            RuntimeError: If no interface can reach the server or multiple interfaces can
        """
        import socket
        import ipaddress
        
        # Get all network interfaces and their IPs
        hostname = socket.gethostname()
        all_ips = []
        
        # Try to get all IPs for this host
        try:
            # Get all addresses for the hostname
            addr_info = socket.getaddrinfo(hostname, None, socket.AF_INET)
            for info in addr_info:
                ip = info[4][0]
                if ip not in all_ips and ip != '127.0.0.1':
                    all_ips.append(ip)
        except:
            pass
        
        # Also try to get IPs by creating test sockets
        # This method often finds more interfaces
        test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Connect to a public DNS server to find our default interface
            test_sock.connect(("8.8.8.8", 80))
            default_ip = test_sock.getsockname()[0]
            if default_ip not in all_ips and default_ip != '127.0.0.1':
                all_ips.append(default_ip)
        except:
            pass
        finally:
            test_sock.close()
        
        # Try to get the IP that's on the same subnet as the server
        try:
            server_ip = ipaddress.ip_address(server_address)
            # Check each local IP to see if it's on the same subnet
            for local_ip_str in all_ips:
                local_ip = ipaddress.ip_address(local_ip_str)
                # Common subnet masks to check
                for prefix_len in [24, 16, 8]:  # /24, /16, /8
                    local_net = ipaddress.ip_network(f"{local_ip_str}/{prefix_len}", strict=False)
                    if server_ip in local_net:
                        # Found an IP on the same subnet, try to verify connectivity
                        if self._test_connectivity(local_ip_str, server_address, command_port):
                            return local_ip_str
        except:
            pass
        
        # If no same-subnet IP found, test all IPs for connectivity
        valid_ips = []
        for ip in all_ips:
            if self._test_connectivity(ip, server_address, command_port):
                valid_ips.append(ip)
        
        if len(valid_ips) == 0:
            # Last resort: try to create a connection and see what local IP is used
            try:
                test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                test_sock.settimeout(1.0)
                # Try to connect to the server (won't actually send data for UDP)
                test_sock.connect((server_address, command_port))
                local_ip = test_sock.getsockname()[0]
                test_sock.close()
                if local_ip and local_ip != '0.0.0.0':
                    return local_ip
            except:
                pass
            
            raise RuntimeError(
                f"Could not find any network interface that can reach the OptiTrack server at {server_address}. "
                f"Please check your network connection and ensure the server is reachable."
            )
        elif len(valid_ips) == 1:
            return valid_ips[0]
        else:
            raise RuntimeError(
                f"Multiple network interfaces can reach the server: {valid_ips}. "
                f"Please specify local_ip_address explicitly."
            )
    
    def _test_connectivity(self, local_ip: str, server_address: str, port: int) -> bool:
        """
        Test if a local IP can reach the server.
        
        Args:
            local_ip: Local IP address to test
            server_address: Server IP address
            port: Port to test
            
        Returns:
            True if the local IP can reach the server
        """
        try:
            # Create a UDP socket (same as what NatNet uses)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(0.5)
            # Bind to the specific local IP
            sock.bind((local_ip, 0))
            # Try to "connect" (for UDP, this just sets the default destination)
            sock.connect((server_address, port))
            # Get the local address that was actually used
            actual_local = sock.getsockname()[0]
            sock.close()
            # Check if we successfully bound to the requested IP
            return actual_local == local_ip
        except Exception:
            return False
    
    def _request_model_definitions(self, max_attempts: int = 3, poll_interval: float = 0.2, timeout_per_attempt: float = 1.0) -> bool:
        """
        Request model definitions (rigid body descriptors) from the OptiTrack server.
        Retries up to max_attempts times, polling for a response within each attempt.

        Returns:
            True if descriptors were received and rigid bodies found, False otherwise.
        """
        for attempt in range(1, max_attempts + 1):
            print(f"Requesting model definitions (attempt {attempt}/{max_attempts})...")
            self.client.send_request(NAT_Messages.REQUEST_MODEL_DEF, "")

            # Poll for descriptors within the timeout
            elapsed = 0.0
            while elapsed < timeout_per_attempt:
                time.sleep(poll_interval)
                elapsed += poll_interval
                if hasattr(self.client, "descriptors") and self.client.descriptors:
                    descs = self.client.descriptors
                    if hasattr(descs, "rigid_body_description") and descs.rigid_body_description:
                        for rb_id, rb_desc in descs.rigid_body_description.items():
                            self.robot_id_to_name[rb_id] = rb_desc.name
                            print(f"Found rigid body: ID={rb_id}, Name='{rb_desc.name}'")
                            self.robot_states[rb_desc.name] = deque(
                                maxlen=self.velocity_window_size
                            )
                            self.robot_states[str(rb_id)] = self.robot_states[rb_desc.name]
                        return True

        print(
            "Warning: No descriptors received from server after "
            f"{max_attempts} attempts. Will use numeric IDs only."
        )
        print("Make sure rigid bodies are enabled/active in Motive.")
        return False

    def connect(self) -> bool:
        """
        Connect to the OptiTrack server.

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.client = NatNetClient(self.params)
            self.client.__enter__()

            if self.client is None:
                return False

            self._request_model_definitions()

            # Start data collection thread
            self.running = True
            self.data_thread = threading.Thread(
                target=self._data_collection_loop, daemon=True
            )
            self.data_thread.start()

            return True

        except Exception as e:
            print(f"Failed to connect to OptiTrack server: {e}")
            return False

    def disconnect(self):
        """Disconnect from the OptiTrack server."""
        self.running = False
        if self.data_thread:
            self.data_thread.join(timeout=2.0)
        if self.client:
            self.client.__exit__(None, None, None)
            self.client = None

    def _data_collection_loop(self):
        """Background thread to continuously collect MoCap data."""
        if not self.client:
            return

        try:
            for frame_data in self.client.MoCap():
                if not self.running:
                    break

                current_time = time.time()

                # Process rigid body data
                if (
                    hasattr(frame_data, "rigid_body_data")
                    and frame_data.rigid_body_data
                ):
                    for (
                        rb_id,
                        rigid_body,
                    ) in frame_data.rigid_body_data.rigid_bodies_d.items():
                        # Get robot name or use ID as string
                        robot_key = self.robot_id_to_name.get(rb_id, str(rb_id))

                        with self._lock:
                            # Initialize deque if needed
                            if robot_key not in self.robot_states:
                                self.robot_states[robot_key] = deque(
                                    maxlen=self.velocity_window_size
                                )
                                self.robot_states[str(rb_id)] = self.robot_states[
                                    robot_key
                                ]
                                # Log if we're using ID because no name was found
                                if robot_key == str(rb_id):
                                    print(f"Tracking rigid body with ID {rb_id} (no name available yet)")

                            states = self.robot_states[robot_key]
                            
                            # Determine last_tracked_timestamp
                            if rigid_body.tracking:
                                # Currently tracked - update last_tracked_timestamp
                                last_tracked_ts = current_time
                            else:
                                # Not tracked - use previous last_tracked_timestamp if available
                                if len(states) > 0:
                                    last_tracked_ts = states[-1].last_tracked_timestamp
                                else:
                                    # First state and not tracked - use current time as fallback
                                    last_tracked_ts = current_time

                            # Create new state
                            new_state = RigidBodyState(
                                timestamp=current_time,
                                last_tracked_timestamp=last_tracked_ts,
                                position=rigid_body.pos,
                                quaternion=rigid_body.rot,
                                tracking=rigid_body.tracking,
                            )

                            # Calculate velocities if we have previous states
                            if len(states) > 0:
                                self._calculate_velocities(new_state, states)

                            # Add new state
                            states.append(new_state)

        except Exception as e:
            print(f"Error in data collection loop: {e}")

    def _calculate_velocities(self, new_state: RigidBodyState, previous_states: deque):
        """
        Calculate linear and angular velocities from state history.

        Args:
            new_state: Current rigid body state
            previous_states: Deque of previous states
        """
        if len(previous_states) == 0:
            return

        # Use multiple samples for smoothing if available
        num_samples = min(len(previous_states), self.velocity_window_size - 1)

        # Use regular variables for accumulation since Position is immutable
        total_lin_vel_x = 0.0
        total_lin_vel_y = 0.0
        total_lin_vel_z = 0.0
        total_ang_vel_roll = 0.0
        total_ang_vel_pitch = 0.0
        total_ang_vel_yaw = 0.0
        valid_samples = 0

        for i in range(num_samples):
            prev_state = previous_states[-(i + 1)]
            dt = new_state.timestamp - prev_state.timestamp

            if dt > 0:
                # Linear velocity
                lin_vel_x = (new_state.position.x - prev_state.position.x) / dt
                lin_vel_y = (new_state.position.y - prev_state.position.y) / dt
                lin_vel_z = (new_state.position.z - prev_state.position.z) / dt

                # Angular velocity (using Euler angle rates - simplified)
                # Note: This is a simplified calculation. For more accuracy,
                # quaternion derivatives should be used
                roll_rate = (
                    self._angle_diff(
                        new_state.quaternion.roll, prev_state.quaternion.roll
                    )
                    / dt
                )
                pitch_rate = (
                    self._angle_diff(
                        new_state.quaternion.pitch, prev_state.quaternion.pitch
                    )
                    / dt
                )
                yaw_rate = (
                    self._angle_diff(
                        new_state.quaternion.yaw, prev_state.quaternion.yaw
                    )
                    / dt
                )

                # Weight more recent samples higher
                weight = 1.0 / (i + 1)
                total_lin_vel_x += lin_vel_x * weight
                total_lin_vel_y += lin_vel_y * weight
                total_lin_vel_z += lin_vel_z * weight

                total_ang_vel_roll += roll_rate * weight
                total_ang_vel_pitch += pitch_rate * weight
                total_ang_vel_yaw += yaw_rate * weight

                valid_samples += weight

        if valid_samples > 0:
            # Normalize by total weight and create new Position objects
            new_state.linear_velocity = Position(
                x=total_lin_vel_x / valid_samples,
                y=total_lin_vel_y / valid_samples,
                z=total_lin_vel_z / valid_samples,
            )
            new_state.angular_velocity = Position(
                x=total_ang_vel_roll / valid_samples,
                y=total_ang_vel_pitch / valid_samples,
                z=total_ang_vel_yaw / valid_samples,
            )

    def _angle_diff(self, angle1: float, angle2: float) -> float:
        """
        Calculate the shortest angular difference between two angles.

        Args:
            angle1: First angle in radians
            angle2: Second angle in radians

        Returns:
            Shortest angular difference in radians
        """
        diff = angle1 - angle2
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def get_robot_names(self) -> List[str]:
        """
        Get list of available robot names.

        Returns:
            List of robot names
        """
        with self._lock:
            # Filter out ID-only keys
            return [name for name in self.robot_states.keys() if not name.isdigit()]

    def get_pos(self, robot_name: str) -> Optional[Tuple[float, float, float]]:
        """
        Get position of a robot.

        Args:
            robot_name: Name or ID of the robot

        Returns:
            Tuple of (x, y, z) position or None if not available
        """
        with self._lock:
            if (
                robot_name in self.robot_states
                and len(self.robot_states[robot_name]) > 0
            ):
                latest_state = self.robot_states[robot_name][-1]
                return (
                    latest_state.position.x,
                    latest_state.position.y,
                    latest_state.position.z,
                )
        return None

    def get_quat(self, robot_name: str) -> Optional[Tuple[float, float, float, float]]:
        """
        Get quaternion orientation of a robot.

        Args:
            robot_name: Name or ID of the robot

        Returns:
            Tuple of (x, y, z, w) quaternion or None if not available
        """
        with self._lock:
            if (
                robot_name in self.robot_states
                and len(self.robot_states[robot_name]) > 0
            ):
                latest_state = self.robot_states[robot_name][-1]
                return (
                    latest_state.quaternion.x,
                    latest_state.quaternion.y,
                    latest_state.quaternion.z,
                    latest_state.quaternion.w,
                )
        return None

    def get_lin_vel(self, robot_name: str) -> Optional[Tuple[float, float, float]]:
        """
        Get linear velocity of a robot.

        Args:
            robot_name: Name or ID of the robot

        Returns:
            Tuple of (vx, vy, vz) linear velocity or None if not available
        """
        with self._lock:
            if (
                robot_name in self.robot_states
                and len(self.robot_states[robot_name]) > 0
            ):
                latest_state = self.robot_states[robot_name][-1]
                if latest_state.linear_velocity:
                    return (
                        latest_state.linear_velocity.x,
                        latest_state.linear_velocity.y,
                        latest_state.linear_velocity.z,
                    )
        return None

    def get_ang_vel(self, robot_name: str) -> Optional[Tuple[float, float, float]]:
        """
        Get angular velocity of a robot.

        Args:
            robot_name: Name or ID of the robot

        Returns:
            Tuple of (roll_rate, pitch_rate, yaw_rate) angular velocity or None if not available
        """
        with self._lock:
            if (
                robot_name in self.robot_states
                and len(self.robot_states[robot_name]) > 0
            ):
                latest_state = self.robot_states[robot_name][-1]
                if latest_state.angular_velocity:
                    return (
                        latest_state.angular_velocity.x,
                        latest_state.angular_velocity.y,
                        latest_state.angular_velocity.z,
                    )
        return None

    def get_tracking_status(self, robot_name: str) -> Optional[bool]:
        """
        Get tracking status of a robot.

        Args:
            robot_name: Name or ID of the robot

        Returns:
            True if currently tracked, False if track lost, None if robot not available
        """
        with self._lock:
            if (
                robot_name in self.robot_states
                and len(self.robot_states[robot_name]) > 0
            ):
                latest_state = self.robot_states[robot_name][-1]
                return latest_state.tracking
        return None

    def get_position_staleness(self, robot_name: str) -> Optional[float]:
        """
        Get time elapsed since position was last tracked.

        Args:
            robot_name: Name or ID of the robot

        Returns:
            Time in seconds since last tracked update, or None if robot not available
        """
        with self._lock:
            if (
                robot_name in self.robot_states
                and len(self.robot_states[robot_name]) > 0
            ):
                latest_state = self.robot_states[robot_name][-1]
                return latest_state.timestamp - latest_state.last_tracked_timestamp
        return None

    def __enter__(self):
        """Context manager entry."""
        if self.connect():
            return self
        else:
            raise ConnectionError("Failed to connect to OptiTrack server")

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


# Example usage
if __name__ == "__main__":
    # Example 1: Auto-detect local IP (NEW!)
    mocap = MoCapUtility(
        server_address="192.168.0.232"
        # local_ip_address will be auto-detected
    )
    
    # Example 2: Manually specify local IP (traditional)
    # mocap = MoCapUtility(
    #     server_address="192.168.0.232", 
    #     local_ip_address="192.168.0.136"
    # )

    # Connect and use
    with mocap:
        # Wait a bit for data to start coming in
        time.sleep(1)

        # Get available robots
        robots = mocap.get_robot_names()
        print(f"Available robots: {robots}")

        # Monitor robot data for a few seconds
        for _ in range(10):
            for robot in robots:
                pos = mocap.get_pos(robot)
                quat = mocap.get_quat(robot)
                lin_vel = mocap.get_lin_vel(robot)
                ang_vel = mocap.get_ang_vel(robot)

                print(f"\nRobot: {robot}")
                if pos:
                    print(f"  Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
                if quat:
                    print(
                        f"  Quaternion: x={quat[0]:.3f}, y={quat[1]:.3f}, z={quat[2]:.3f}, w={quat[3]:.3f}"
                    )
                if lin_vel:
                    print(
                        f"  Linear Velocity: vx={lin_vel[0]:.3f}, vy={lin_vel[1]:.3f}, vz={lin_vel[2]:.3f}"
                    )
                if ang_vel:
                    print(
                        f"  Angular Velocity: roll_rate={ang_vel[0]:.3f}, pitch_rate={ang_vel[1]:.3f}, yaw_rate={ang_vel[2]:.3f}"
                    )

            time.sleep(0.5)
