#!/usr/bin/env python3
"""
Single-arm follower-side teleoperation script using PubNub for internet communication.

This script:
1. Connects to 1 ARX R5 follower robot via CAN interface  # REMOVED: dual functionality for 2 followers
2. Subscribes to position data from PubNub
3. Applies received positions to the ARX follower robot with safety checks

Usage:
    python teleop_single_arx_follower.py
"""

import os
import logging
import zmq 
import json

# Disable PubNub logging via environment variable
os.environ['PUBNUB_LOG_LEVEL'] = 'NONE'

# Configure logging BEFORE importing other modules
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
logger = logging.getLogger(__name__)

# Suppress verbose HTTP logs from various libraries
logging.getLogger('urllib3').setLevel(logging.ERROR)
logging.getLogger('requests').setLevel(logging.ERROR)
logging.getLogger('httpx').setLevel(logging.ERROR)
logging.getLogger('httpcore').setLevel(logging.ERROR)
logging.getLogger('pubnub').setLevel(logging.WARNING)
# Disable all INFO logs from modules starting with 'http'
for name in logging.root.manager.loggerDict:
    if name.startswith('http'):
        logging.getLogger(name).setLevel(logging.ERROR)

import argparse
# import json  # Already imported above
import platform
import signal
import subprocess
import sys
import time
import threading
from typing import Dict, List, Optional
import numpy as np

# try:
#     from pubnub.pnconfiguration import PNConfiguration
#     from pubnub.pubnub import PubNub
#     from pubnub.exceptions import PubNubException
#     from pubnub.callbacks import SubscribeCallback
#     from pubnub.enums import PNStatusCategory
# except ImportError:
#     print("PubNub not installed. Please install with: pip install pubnub")
#     sys.exit(1)

try:
    from colorama import init, Fore, Style
    init()
except ImportError:
    # Fallback if colorama not installed
    class Fore:
        RED = GREEN = YELLOW = CYAN = MAGENTA = ""
    class Style:
        RESET_ALL = BRIGHT = ""

# Import our modules
# import pubnub_config
from arx_control import ARXArm  # Use ARX arm instead of SO101Controller

# Global flag for graceful shutdown
shutdown_requested = False


def signal_handler(signum, frame):
    """Handle SIGINT (Ctrl+C) for graceful shutdown."""
    global shutdown_requested
    logger.info("\n\n⚠️  Shutdown requested. Cleaning up...")
    shutdown_requested = True


class ARXPositionSmoother:
    """Smooth position changes for ARX arm to prevent jerky movements."""
    
    def __init__(self, smoothing_factor: float = 0.8):
        self.smoothing_factor = smoothing_factor
        self.current_positions = np.zeros(6)  # 6 arm joints for ARX R5 (gripper handled separately)
        self.initialized = False
        
    def smooth(self, target_positions: np.ndarray) -> np.ndarray:
        """Apply exponential smoothing to position changes."""
        if not self.initialized:
            self.current_positions = target_positions.copy()
            self.initialized = True
            return target_positions
            
        # Apply smoothing
        smoothed = (self.current_positions * self.smoothing_factor + 
                   target_positions * (1 - self.smoothing_factor))
        
        # Enforce maximum change limit per joint
        max_change = 0.1  # Maximum change per joint in radians
        for i in range(len(smoothed)):
            change = abs(smoothed[i] - self.current_positions[i])
            if change > max_change:
                # Limit the change
                direction = 1 if smoothed[i] > self.current_positions[i] else -1
                smoothed[i] = self.current_positions[i] + (direction * max_change)
                
        self.current_positions = smoothed
        return smoothed


class ARXArmWrapper:
    """Wrapper around ARXArm to provide SO101-style interface for teleoperation."""
    
    def __init__(self, can_port: str = "can0", robot_type: int = 1, calibration_file: str = "arx_leader_calibration.json"):
        """Initialize ARX arm wrapper.
        
        Args:
            can_port: CAN interface port (e.g., "can0")
            robot_type: 1 for R5, 0 for X5lite
            calibration_file: Path to calibration file for leader-follower position mapping
        """
        self.arm_config = {
            "can_port": can_port,
            "type": robot_type,
            "num_joints": 6,  # 6 arm joints for ARX R5 (gripper handled separately)
        }
        self.arm = None
        self.connected = False
        self.robot_id = "ARXFollower"
        self.calibration_file = calibration_file
        
        # Position conversion constants
        # Convert from SO101 servo positions (0-4095) to ARX joint positions (radians)
        self.servo_to_radian_scale = (2 * np.pi) / 4095.0  # Full rotation scale
        
        # Load calibration data or use defaults
        self.servo_centers, self.invert_motors = self._load_calibration()
        
    def _load_calibration(self) -> tuple[Dict[int, int], List[int]]:
        """Load servo center positions and motor inversion list from calibration file."""
        default_centers = {i: 2048 for i in range(1, 8)}  # Default center for 7 joints
        default_invert = []  # No motors inverted by default
        
        if not os.path.exists(self.calibration_file):
            logger.warning(f"Calibration file not found: {self.calibration_file}")
            logger.warning("Using default servo centers (2048). Consider running calibration.")
            return default_centers, default_invert
            
        try:
            with open(self.calibration_file, 'r') as f:
                calibration_data = json.load(f)
                
            home_positions = calibration_data.get("home_positions", {})
            motor_ids = calibration_data.get("motor_ids", list(range(1, 8)))
            invert_motors = calibration_data.get("invert_motors", [])
            
            # Convert string keys to int and validate
            servo_centers = {}
            for motor_id in motor_ids:
                key = str(motor_id)  # JSON keys are strings
                if key in home_positions:
                    servo_centers[motor_id] = int(home_positions[key])
                else:
                    logger.warning(f"Missing calibration for motor {motor_id}, using default")
                    servo_centers[motor_id] = 2048
                    
            logger.info(f"✓ Loaded calibration from {self.calibration_file}")
            logger.info(f"  Created: {calibration_data.get('timestamp_str', 'Unknown')}")
            logger.info(f"  Servo centers: {servo_centers}")
            if invert_motors:
                logger.info(f"  Inverted motors: {invert_motors}")
            
            return servo_centers, invert_motors
            
        except Exception as e:
            logger.error(f"Failed to load calibration: {e}")
            logger.warning("Using default servo centers")
            return default_centers, default_invert
        
    def connect(self):
        """Connect to the ARX arm."""
        try:
            self.arm = ARXArm(self.arm_config)
            self.connected = True
            logger.info(f"{Fore.GREEN}✓ Connected to ARX R5 arm{Style.RESET_ALL}")
            
            # Initialize to home position
            self.arm.go_home()
            time.sleep(1)  # Give time for initialization
            
        except Exception as e:
            logger.error(f"Failed to connect to ARX arm: {e}")
            raise
            
    def disconnect(self):
        """Disconnect from the ARX arm."""
        if self.connected and self.arm:
            try:
                # Return to safe position before disconnect
                self.arm.go_home()
                time.sleep(0.5)
            except:
                pass
        self.connected = False
        self.arm = None
        
    def read_joint_tics(self) -> Dict[int, int]:
        """Read current joint positions as tic values (SO101-style interface).
        
        Returns:
            Dict mapping motor ID to position in tics
            Motors 1-6: Arm joint positions
            Motor 7: Gripper position
        """
        if not self.connected or not self.arm:
            return {}
            
        try:
            # Read arm joint positions (6 joints)
            joint_positions = self.arm.get_joint_positions()  # Returns radians for 6 joints
            tics = {}
            
            # Convert arm joints (1-6) back to tics
            for i, pos_rad in enumerate(joint_positions[:6]):  # 6 arm joints
                motor_id = i + 1  # Motor IDs are 1-indexed
                servo_center = self.servo_centers.get(motor_id, 2048)
                
                # Check if this motor should be inverted (reverse the inversion for reading)
                if motor_id in self.invert_motors:
                    # Inverted: reverse the inversion when reading back
                    tic_pos = int(servo_center - pos_rad / self.servo_to_radian_scale)
                else:
                    # Normal: standard conversion
                    tic_pos = int(pos_rad / self.servo_to_radian_scale + servo_center)
                    
                tics[motor_id] = tic_pos
                
            # Note: ARX SDK doesn't provide a way to read gripper position
            # So we can't include motor 7 in the return dict for now
            # If needed, we could track the last sent gripper position
            
            return tics
        except Exception as e:
            logger.error(f"Error reading joint positions: {e}")
            return {}
            
    def write_joint_tics(self, positions: Dict[int, int]):
        """Write joint positions from tic values (SO101-style interface).
        
        Args:
            positions: Dict mapping motor ID to position in tics
                      Motors 1-6: Arm joints (sent to set_joint_positions)
                      Motor 7: Gripper (sent to set_catch_pos)
        """
        if not self.connected or not self.arm:
            return
            
        try:
            # Separate arm joints (1-6) from gripper (7)
            arm_positions = np.zeros(6)  # 6 joints for ARX R5 arm
            gripper_position = None
            
            for motor_id, tic_pos in positions.items():
                if 1 <= motor_id <= 6:  # Arm joints
                    # Get calibrated center for this motor
                    servo_center = self.servo_centers.get(motor_id, 2048)
                    
                    # Check if this motor should be inverted
                    if motor_id in self.invert_motors:
                        # Inverted: flip motion around center point
                        rad_pos = (servo_center - tic_pos) * self.servo_to_radian_scale
                        logger.debug(f"Motor {motor_id} inverted: {tic_pos} -> {rad_pos:.3f}")
                    else:
                        # Normal: standard conversion
                        rad_pos = (tic_pos - servo_center) * self.servo_to_radian_scale
                        
                    arm_positions[motor_id - 1] = rad_pos  # Convert to 0-indexed
                    
                elif motor_id == 7:  # Gripper
                    gripper_position = tic_pos
                    
            # Set arm joint positions (6 joints)
            if len(arm_positions) == 6:
                self.arm.set_joint_positions(arm_positions)
                
            # Set gripper position if present
            if gripper_position is not None:
                gripper_cmd = self._convert_gripper_tics_to_cmd(gripper_position)
                logger.debug(f"Gripper: tics={gripper_position} -> cmd={gripper_cmd:.3f}")
                self.arm.set_catch_pos(gripper_cmd)
            
        except Exception as e:
            logger.error(f"Error writing joint positions: {e}")
            
    def write_joint_tics_smoothed(self, positions: Dict[int, int], smoother):
        """Write joint positions with smoothing applied to arm joints."""
        if not self.connected or not self.arm:
            return
            
        try:
            # Separate arm joints (1-6) from gripper (7)
            arm_positions = np.zeros(6)  # 6 joints for ARX R5 arm
            gripper_position = None
            
            for motor_id, tic_pos in positions.items():
                if 1 <= motor_id <= 6:  # Arm joints
                    # Get calibrated center for this motor
                    servo_center = self.servo_centers.get(motor_id, 2048)
                    
                    # Check if this motor should be inverted
                    if motor_id in self.invert_motors:
                        # Inverted: flip motion around center point
                        rad_pos = (servo_center - tic_pos) * self.servo_to_radian_scale
                        logger.debug(f"Motor {motor_id} inverted: {tic_pos} -> {rad_pos:.3f}")
                    else:
                        # Normal: standard conversion
                        rad_pos = (tic_pos - servo_center) * self.servo_to_radian_scale
                        
                    arm_positions[motor_id - 1] = rad_pos  # Convert to 0-indexed
                    
                elif motor_id == 7:  # Gripper
                    gripper_position = tic_pos
                    
            # Apply smoothing to arm joint positions (in radians)
            if len(arm_positions) == 6:
                smoothed_positions = smoother.smooth(arm_positions)
                self.arm.set_joint_positions(smoothed_positions)
                
            # Set gripper position if present
            if gripper_position is not None:
                gripper_cmd = self._convert_gripper_tics_to_cmd(gripper_position)
                logger.debug(f"Gripper: tics={gripper_position} -> cmd={gripper_cmd:.3f}")
                self.arm.set_catch_pos(gripper_cmd)
            
        except Exception as e:
            logger.error(f"Error writing smoothed joint positions: {e}")
            
    def _convert_gripper_tics_to_cmd(self, tic_pos: int) -> float:
        """Convert gripper servo tics to ARX gripper command (-1.0 to 1.0).
        
        Args:
            tic_pos: Servo position in tics
            
        Returns:
            float: Gripper command (-1.0 = fully closed, 0.0 = neutral, 1.0 = fully open)
        """
        # Get calibrated center position for gripper (motor 7)
        servo_center = self.servo_centers.get(7, 2048)
        
        # Define gripper range in tics (adjust this based on your gripper's actual range)
        # This assumes ±1000 tics from center gives full gripper range
        max_gripper_range = 1000.0
        
        # Calculate offset from center
        offset = tic_pos - servo_center
        
        # Map to -1.0 to 1.0 range, with clamping
        gripper_cmd = offset / max_gripper_range
        gripper_cmd = max(-1.0, min(1.0, gripper_cmd))  # Clamp to valid range
        
        return gripper_cmd


# class TelemetryListener(SubscribeCallback):
#     """Listen for telemetry data from leaders."""
    
#     def __init__(self):
#         self.latest_data = None
#         self.last_sequence = 0
#         self.received_count = 0
#         self.dropped_count = 0
#         self.last_receive_time = 0
        
#     def status(self, pubnub, status):
#         """Handle connection status changes."""
#         if status.category == PNStatusCategory.PNConnectedCategory:
#             logger.info(f"{Fore.GREEN}✓ Connected to PubNub channels{Style.RESET_ALL}")
#         elif status.category == PNStatusCategory.PNReconnectedCategory:
#             logger.info(f"{Fore.YELLOW}Reconnected to PubNub{Style.RESET_ALL}")
#         elif status.category == PNStatusCategory.PNDisconnectedCategory:
#             logger.warning(f"{Fore.RED}Disconnected from PubNub{Style.RESET_ALL}")
            
#     def message(self, pubnub, message):
#         """Handle incoming telemetry messages."""
#         try:
#             # Get the message data
#             data = message.message
                
#             if isinstance(data, dict) and data.get("type") == "telemetry":
#                 sequence = data.get("sequence", 0)
                
#                 # Only count as dropped if we actually missed messages (not just overwrites)
#                 if self.last_sequence > 0 and sequence > self.last_sequence + 1:
#                     missed = sequence - self.last_sequence - 1
#                     # Only count as dropped if we have pending data that would be overwritten
#                     if self.latest_data is not None:
#                         self.dropped_count += missed
                
#                 self.latest_data = data
#                 self.last_receive_time = time.time()
#                 self.received_count += 1
#                 self.last_sequence = sequence
                
#         except Exception as e:
#             logger.error(f"Error processing message: {e}")


class SingleFollowerTeleop:
    """Main teleoperation class for single ARX follower."""
    
    def __init__(self, can_port: str = "can0", robot_type: int = 1, calibration_file: str = "arx_leader_calibration.json"):
        self.can_port = can_port
        self.robot_type = robot_type
        self.calibration_file = calibration_file
        self.follower: Optional[ARXArmWrapper] = None  # SIMPLIFIED: Single follower instead of list
        self.running = False
        
        # Network components
        # self.pubnub: Optional[PubNub] = None
        # self.telemetry_listener = TelemetryListener()
        
        # Position smoothing
        # self.smoother = ARXPositionSmoother(pubnub_config.POSITION_SMOOTHING)
        
        # Update tracking
        self.last_update_time = 0
        self.update_times = []
        self.latencies = []
        self.s = zmq.Context().socket(zmq.PULL)
        self.s.bind("tcp://0.0.0.0:5000")
        print("Follower set up to ZMQ")
        
    # def setup_pubnub(self):
    #     """Initialize PubNub connection."""
    #     logger.info("Setting up PubNub connection...")
    #     
    #     pnconfig = PNConfiguration()
    #     pnconfig.subscribe_key = pubnub_config.SUBSCRIBE_KEY
    #     pnconfig.publish_key = pubnub_config.PUBLISH_KEY
    #     pnconfig.user_id = f"follower-{platform.node()}"
    #     pnconfig.ssl = True
    #     pnconfig.enable_subscribe = True
    #     # Disable PubNub's internal logging
    #     pnconfig.log_verbosity = False
    #     pnconfig.enable_logging = False
    #     
    #     self.pubnub = PubNub(pnconfig)
    #     # self.pubnub.add_listener(self.telemetry_listener)
    #     
    #     # Subscribe to telemetry channel
    #     self.pubnub.subscribe().channels([pubnub_config.TELEMETRY_CHANNEL]).execute()
    #     
    #     logger.info(f"{Fore.GREEN}✓ PubNub connected as {pnconfig.user_id}{Style.RESET_ALL}")
        
    def connect_follower(self):
        """Connect to the ARX follower robot."""
        # SIMPLIFIED: Single follower object instead of list
        self.follower = ARXArmWrapper(self.can_port, self.robot_type, self.calibration_file)
        self.follower.connect()
        
        logger.info(f"{Fore.GREEN}✓ Connected to ARX follower robot{Style.RESET_ALL}")
        
    # def send_acknowledgment(self, sequence: int, timestamp: float):
    #     """Send acknowledgment back to leader."""
    #     try:
    #         ack_msg = {
    #             "type": "ack",
    #             "sequence": sequence,
    #             "timestamp": timestamp,
    #             "follower_id": f"follower-{platform.node()}"
    #         }
    #         self.pubnub.publish().channel(pubnub_config.STATUS_CHANNEL).message(ack_msg).pn_async(lambda result, status: None)
    #     except:
    #         pass  # Don't fail on ack errors
            
    # def send_status(self):
    #     """Send periodic status updates."""
    #     try:
    #         status_msg = {
    #             "type": "status",
    #             "timestamp": time.time(),
    #             "follower_id": f"follower-{platform.node()}",
    #             "motors_active": 7,  # 6 arm joints + 1 gripper for ARX R5
    #             "followers_connected": 1  # Single follower
    #         }
    #         self.pubnub.publish().channel(pubnub_config.STATUS_CHANNEL).message(status_msg).sync()
    #     except:
    #         pass
            
    def apply_positions(self, telemetry_data: Dict):
        """Apply received positions to ARX follower robot."""
        timestamp = telemetry_data.get("timestamp", 0)
        sequence = telemetry_data.get("sequence", 0)
        positions_data = telemetry_data.get("positions", {})
        
        # Debug logging
        logger.debug(f"Received positions: {positions_data}")
        
        # Calculate latency
        latency = (time.time() - timestamp) * 1000  # ms
        self.latencies.append(latency)
        if len(self.latencies) > 100:
            self.latencies.pop(0)
            
        # Safety check: reject if latency too high
        # if latency > pubnub_config.MAX_LATENCY_MS:
        #     logger.warning(f"{Fore.RED}Rejecting data: latency {latency:.1f}ms > {pubnub_config.MAX_LATENCY_MS}ms{Style.RESET_ALL}")
        #     return
            
        # Send acknowledgment (only every 5th message to reduce traffic)
        # if sequence % 5 == 0:
        #     self.send_acknowledgment(sequence, timestamp)
        
        # SIMPLIFIED: Direct position application for single arm
        if not self.follower or not self.follower.connected:
            logger.warning("No connected follower to apply positions to")
            return
            
        try:
            # Convert string motor IDs back to integers and create position dict
            motor_positions = {}
            for motor_id_str, position in positions_data.items():
                motor_id = int(motor_id_str)
                motor_positions[motor_id] = position
                
            logger.debug(f"Writing positions to ARX arm: {motor_positions}")
            
            # Apply positions to ARX arm with smoothing
            self.follower.write_joint_tics(motor_positions)
            
        except Exception as e:
            logger.error(f"Error applying positions: {e}")
            
    def display_status(self):
        """Display current status and statistics."""
        # Clear screen and move cursor to top
        print("\033[2J\033[H", end="")
        
        print(f"{Style.BRIGHT}=== SINGLE ARM ARX FOLLOWER TELEOPERATION ==={Style.RESET_ALL}")
        
        # Connected follower
        print(f"{Style.BRIGHT}Connected Follower:{Style.RESET_ALL}")
        if self.follower:
            print(f"  ARX R5 - {'Connected' if self.follower.connected else 'Disconnected'}")
            print(f"  Motors: 6 arm joints + 1 gripper")  # ARX R5 architecture
        print()
        
        # # Network stats
        # stats = {
        #     'received': self.telemetry_listener.received_count,
        #     'dropped': self.telemetry_listener.dropped_count,
        #     'latency': self.latencies
        # }
        # print(f"{Style.BRIGHT}Network Statistics:{Style.RESET_ALL}")
        # if stats['latency']:
        #     avg_latency = sum(stats['latency']) / len(stats['latency'])
        #     max_latency = max(stats['latency'])
        #     print(f"  Average Latency: {avg_latency:.1f}ms")
        #     print(f"  Max Latency:     {max_latency:.1f}ms")
        # else:
        #     print(f"  Latency: No data yet")
            
        # print(f"  Received:        {stats['received']}")
        # print(f"  Dropped:         {stats['dropped']}")
        
        # Update rate
        if self.update_times:
            avg_interval = sum(self.update_times) / len(self.update_times)
            actual_fps = 1.0 / avg_interval if avg_interval > 0 else 0
            print(f"  Update Rate:     {actual_fps:.1f} Hz")
            
        # # Connection status
        # if self.telemetry_listener.last_receive_time > 0:
        #     age = time.time() - self.telemetry_listener.last_receive_time
        #     if age < 1:
        #         status = f"{Fore.GREEN}Connected{Style.RESET_ALL}"
        #     elif age < 5:
        #         status = f"{Fore.YELLOW}Slow{Style.RESET_ALL}"
        #     else:
        #         status = f"{Fore.RED}Disconnected{Style.RESET_ALL}"
        #     print(f"  Status:          {status} (last data {age:.1f}s ago)")
            
        print()
        print(f"{Fore.CYAN}Press Ctrl+C to stop{Style.RESET_ALL}")
        
    def teleoperation_loop(self):
        """Main loop processing received positions."""
        self.running = True
        
        # Start display thread
        display_thread = threading.Thread(target=self.display_loop, daemon=True)
        display_thread.start()
        
        # Start status thread
        status_thread = threading.Thread(target=self.status_loop, daemon=True)
        status_thread.start()
        
        logger.info("Starting ARX follower teleoperation...")
        
        try:
            while self.running and not shutdown_requested:
                # Check for new telemetry data (blocking receive)
                try:
                    message = self.s.recv_string(flags=zmq.NOBLOCK)  # Non-blocking receive
                    # Process the latest data
                    self.apply_positions(json.loads(message))
                    
                    # Track update rate
                    now = time.time()
                    if self.last_update_time > 0:
                        self.update_times.append(now - self.last_update_time)
                        if len(self.update_times) > 100:
                            self.update_times.pop(0)
                    self.last_update_time = now
                    
                    # Clear to prevent reprocessing
                    # self.telemetry_listener.latest_data = None
                except zmq.Again:
                    # No message available, continue
                    pass
                except Exception as e:
                    logger.error(f"Error receiving ZMQ message: {e}")
                    
                # Small sleep to prevent CPU spinning
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            logger.info("\nStopping teleoperation...")
        finally:
            self.running = False
            
    def display_loop(self):
        """Separate thread for updating display."""
        while self.running and not shutdown_requested:
            self.display_status()
            time.sleep(0.5)  # Update display at 2Hz
            
    def status_loop(self):
        """Send periodic status updates."""
        while self.running and not shutdown_requested:
            # self.send_status()
            time.sleep(2)  # Send status every 2 seconds
            
    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        
        # Unsubscribe from channels
        # if self.pubnub:
        #     self.pubnub.unsubscribe_all()
            
        # Return to home position and disconnect robot
        logger.info("Returning ARX arm to home position...")
        if self.follower and self.follower.connected:
            try:
                self.follower.arm.go_home()
                time.sleep(1)  # Give time for movement
            except Exception as e:
                logger.warning(f"Failed to return to home position: {e}")
                
        logger.info("Disconnecting robot...")
        if self.follower:
            try:
                self.follower.disconnect()
            except Exception as e:
                logger.warning(f"Failed to disconnect follower: {e}")
                
        logger.info("Shutdown complete")


def main():
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # SETUP ZMQ RECEIVING:
    
    
    
    parser = argparse.ArgumentParser(description="Single-arm ARX follower-side teleoperation via PubNub")
    parser.add_argument("--can_port", type=str, default="can0",
                       help="CAN interface port")
    parser.add_argument("--robot_type", type=int, default=1,
                       help="Robot type (0 for X5lite, 1 for R5)")
    parser.add_argument("--calibration_file", type=str, default="arx_leader_calibration.json",
                       help="Path to calibration file for servo-to-ARX position mapping")
    parser.add_argument("--debug", action="store_true",
                       help="Enable debug logging")
    
    args = parser.parse_args()
    
    # Set logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.debug("Debug logging enabled")
    
    # Create and run teleoperation
    teleop = SingleFollowerTeleop(args.can_port, args.robot_type, args.calibration_file)
    
    try:
        # Setup
        # teleop.setup_pubnub()
        teleop.connect_follower()
        
        # Run main loop
        teleop.teleoperation_loop()
        
    except Exception as e:
        logger.error(f"Error: {e}")
        return 1
    finally:
        teleop.shutdown()
        
    return 0


if __name__ == "__main__":
    sys.exit(main())