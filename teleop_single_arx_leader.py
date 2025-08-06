#!/usr/bin/env python3
"""
Single-arm leader-side teleoperation script using PubNub for internet communication.

This script:
1. Connects to 1 leader robot (<9V) via USB  # REMOVED: dual functionality for 2 leaders
2. Reads its positions at high frequency
3. Publishes position data to PubNub for a single follower robot

Usage:
    python teleop_single_arx_leader.py
"""

import os
# import logging
import zmq
import json

# Disable PubNub logging via environment variable
os.environ['PUBNUB_LOG_LEVEL'] = 'NONE'

# Configure logging BEFORE importing other modules
# logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
# logger = logging.getLogger(__name__)

# Suppress verbose HTTP logs from various libraries
# logging.getLogger('urllib3').setLevel(logging.ERROR)
# logging.getLogger('requests').setLevel(logging.ERROR)
# logging.getLogger('httpx').setLevel(logging.ERROR)
# logging.getLogger('httpcore').setLevel(logging.ERROR)
# logging.getLogger('pubnub').setLevel(logging.WARNING)
# Disable all INFO logs from modules starting with 'http'
# for name in logging.root.manager.loggerDict:
#     if name.startswith('http'):
#         logging.getLogger(name).setLevel(logging.ERROR)

import argparse
import json
import platform
import signal
import sys
import time
import threading
from typing import Dict, List, Optional

# Import select for Unix systems
try:
    import select
except ImportError:
    # Windows doesn't have select for stdin
    select = None

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
from servo_controller import SO101Controller

# Global flag for graceful shutdown
shutdown_requested = False


def signal_handler(signum, frame):
    """Handle SIGINT (Ctrl+C) for graceful shutdown."""
    global shutdown_requested
    print("\n\n⚠️  Shutdown requested. Cleaning up...")
    shutdown_requested = True


class NetworkMonitor:
    """Monitor network statistics and latency."""
    
    def __init__(self):
        self.sent_count = 0
        self.ack_count = 0
        self.last_sent_time = {}
        self.latencies = []
        self.max_latency_samples = 100
        
    def message_sent(self, sequence: int):
        """Record when a message was sent."""
        self.sent_count += 1
        self.last_sent_time[sequence] = time.time()
        
    def message_acknowledged(self, sequence: int, timestamp: float):
        """Calculate latency when acknowledgment received."""
        if sequence in self.last_sent_time:
            latency = (time.time() - self.last_sent_time[sequence]) * 1000  # ms
            self.latencies.append(latency)
            if len(self.latencies) > self.max_latency_samples:
                self.latencies.pop(0)
            self.ack_count += 1
            del self.last_sent_time[sequence]
            return latency
        return None
        
    def get_stats(self) -> Dict:
        """Get current network statistics."""
        if not self.latencies:
            return {
                "avg_latency": 0, 
                "max_latency": 0, 
                "packet_loss": 0,
                "sent": self.sent_count,
                "acked": self.ack_count
            }
            
        avg_latency = sum(self.latencies) / len(self.latencies)
        max_latency = max(self.latencies)
        expected_acks = self.sent_count // 5  # Only every 5th packet expects ack
        packet_loss = 1 - (self.ack_count / expected_acks) if expected_acks > 0 else 0
        
        return {
            "avg_latency": avg_latency,
            "max_latency": max_latency,
            "packet_loss": packet_loss,
            "sent": self.sent_count,
            "acked": self.ack_count
        }


# class StatusListener(SubscribeCallback):
#     """Listen for status updates from follower."""
#     
#     def __init__(self, monitor: NetworkMonitor):
#         self.monitor = monitor
#         self.follower_status = {}
#         
#     def message(self, pubnub, message):
#         """Handle status messages from follower."""
#         data = message.message
#         if isinstance(data, dict) and data.get("type") == "ack":
#             # Acknowledgment from follower
#             sequence = data.get("sequence")
#             timestamp = data.get("timestamp")
#             if sequence is not None and timestamp is not None:
#                 latency = self.monitor.message_acknowledged(sequence, timestamp)
#                 if latency is not None:
#                     # print(f"Received ack for seq {sequence}, latency: {latency:.1f}ms")
#                     pass
#                 # if latency and latency > pubnub_config.LATENCY_WARNING_MS:
#                 #     print(f"{Fore.YELLOW}High latency: {latency:.1f}ms{Style.RESET_ALL}")
#                 
#         elif isinstance(data, dict) and data.get("type") == "status":
#             # Status update from follower
#             self.follower_status[data.get("follower_id")] = data


class SingleLeaderTeleop:
    """Main teleoperation class for single leader arm."""
    
    # HARDCODED PORT - Change this to switch ports easily
    LEADER_PORT = "/dev/tty.usbmodem5A460813891"
    
    def __init__(self, motor_ids: List[int], baudrate: int = 1000000):
        self.motor_ids = motor_ids
        self.baudrate = baudrate
        self.leader: Optional[SO101Controller] = None  # SIMPLIFIED: Single leader instead of list
        self.running = False
        self.sequence = 0
        
        # Network components
        # self.pubnub: Optional[PubNub] = None
        self.zmq_socket = None
        self.monitor = NetworkMonitor()
        # self.status_listener = StatusListener(self.monitor)
        
        # Performance tracking
        self.last_publish_time = 0
        self.publish_times = []
        
    def setup_pubnub(self):
        """Initialize PubNub connection."""
        print("Setting up PubNub connection...")
        
        pnconfig = PNConfiguration()
        pnconfig.subscribe_key = pubnub_config.SUBSCRIBE_KEY
        pnconfig.publish_key = pubnub_config.PUBLISH_KEY
        pnconfig.user_id = f"leader-{platform.node()}"
        pnconfig.ssl = True
        pnconfig.enable_subscribe = True
        # Disable PubNub's internal logging
        pnconfig.log_verbosity = False
        pnconfig.enable_logging = False
        
        self.pubnub = PubNub(pnconfig)
        self.pubnub.add_listener(self.status_listener)
        
        # Subscribe to status channel for follower feedback
        self.pubnub.subscribe().channels([pubnub_config.STATUS_CHANNEL]).execute()
        
        print(f"{Fore.GREEN}✓ PubNub connected as {pnconfig.user_id}{Style.RESET_ALL}")
        
    def connect_leader(self):
        """Connect to the single leader robot."""
        # Use hardcoded port instead of auto-detection
        leader_port = self.LEADER_PORT
        print(f"Using hardcoded leader port: {leader_port}")
        
        # SIMPLIFIED: Single leader object instead of list
        self.leader = SO101Controller(leader_port, self.motor_ids, self.baudrate, "Leader")
        self.leader.connect()
            
        print(f"{Fore.GREEN}✓ Connected to leader robot at {leader_port}{Style.RESET_ALL}")
        
    def publish_positions(self, positions: Dict[int, int]):
        """Publish position data via ZMQ."""
        self.sequence += 1
        
        # SIMPLIFIED: No mapping needed for single arm - directly use positions
        # Convert motor IDs to strings for JSON serialization
        position_data = {str(motor_id): int(pos) for motor_id, pos in positions.items()}
        
        message = {
            "type": "telemetry",
            "timestamp": time.time(),
            "sequence": self.sequence,
            "positions": position_data  # Single arm positions
        }
        
        try:
            # SETUP STREAMING WITHOUT PUBNUB:
            self.zmq_socket.send_string(json.dumps(message))
            # self.monitor.message_sent(self.sequence)
            
            # Track publish rate
            now = time.time()
            if self.last_publish_time > 0:
                self.publish_times.append(now - self.last_publish_time)
                if len(self.publish_times) > 100:
                    self.publish_times.pop(0)
            self.last_publish_time = now
            
        except Exception as e:
            print(f"Failed to publish: {e}")
            
    def display_status(self):
        """Display current status and statistics - compact version."""
        stats = self.monitor.get_stats()
        
        # Build compact status line
        leader_status = "✓" if self.leader and self.leader.connected else "❌"
        
        # Network info
        if stats['avg_latency'] > 0:
            net_info = f"Latency: {stats['avg_latency']:.1f}ms | Loss: {stats['packet_loss']*100:.1f}%"
        else:
            net_info = "Network: Disconnected"
        
        # Publish rate
        if self.publish_times:
            avg_interval = sum(self.publish_times) / len(self.publish_times)
            actual_fps = 1.0 / avg_interval if avg_interval > 0 else 0
            rate_info = f"Rate: {actual_fps:.1f}Hz"
        else:
            rate_info = "Rate: --"
        
        # Follower count
        # active_followers = sum(1 for fid, status in self.status_listener.follower_status.items() 
        #                       if time.time() - status.get("timestamp", 0) < 5)
        follower_info = f"Followers: 0"
        
        # Single compact line
        status_line = f"LEADER {leader_status} | {net_info} | {rate_info} | {follower_info} | Sent: {stats['sent']}"
        print(f"\r{status_line:<80}", end="", flush=True)
        
    def teleoperation_loop(self):
        """Main loop reading positions and publishing."""
        self.running = True
        target_interval = 1.0 / 20  # Default 20 FPS
        
        # Start display thread
        display_thread = threading.Thread(target=self.display_loop, daemon=True)
        display_thread.start()
        
        # print(f"Starting single arm teleoperation at {pubnub_config.TARGET_FPS} Hz...")
        print("Status updates every 2 seconds on single line. Press Ctrl+C to stop.")
        
        try:
            while self.running and not shutdown_requested:
                loop_start = time.time()
                
                # Read positions from the leader
                if self.leader and self.leader.connected:
                    positions = self.leader.read_positions()
                    if positions:
                        self.publish_positions(positions)
                    
                # Maintain target rate
                elapsed = time.time() - loop_start
                if elapsed < target_interval:
                    time.sleep(target_interval - elapsed)
                    
        except KeyboardInterrupt:
            print()  # New line after status display
            print("Stopping teleoperation...")
        finally:
            self.running = False
            
    def display_loop(self):
        """Separate thread for updating display."""
        while self.running and not shutdown_requested:
            self.display_status()
            time.sleep(2.0)  # Update display at 0.5Hz (every 2 seconds) - much less frequent
            
    def shutdown(self):
        """Clean shutdown."""
        self.running = False
        print()  # New line after status display
        
        # Publish disconnect message
        if self.pubnub:
            try:
                disconnect_msg = {
                    "type": "disconnect",
                    "timestamp": time.time(),
                    "leader_id": f"leader-{platform.node()}"
                }
                self.pubnub.publish().channel(pubnub_config.STATUS_CHANNEL).message(disconnect_msg).sync()
            except:
                pass
                
            self.pubnub.unsubscribe_all()
            
        # Disconnect robot
        print("Disconnecting robot...")
        if self.leader:
            try:
                self.leader.disconnect()
            except Exception as e:
                print(f"Failed to disconnect leader: {e}")
                
        print("Shutdown complete")


def main():
    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    parser = argparse.ArgumentParser(description="Single-arm leader-side teleoperation via PubNub")
    parser.add_argument("--motor_ids", type=str, default="1,2,3,4,5,6,7",  # All 7 motors: 1-6 for arm joints, 7 for gripper
                       help="Comma-separated motor IDs")
    parser.add_argument("--baudrate", type=int, default=1000000,
                       help="Serial baudrate")
    parser.add_argument("--fps", type=int, default=20,
                       help="Target update rate (Hz)")
    
    args = parser.parse_args()
    
    # Parse motor IDs
    motor_ids = [int(id.strip()) for id in args.motor_ids.split(",")]
    
    # Override config FPS if specified
    target_fps = args.fps if args.fps else 20
    
    # Create and run teleoperation
    teleop = SingleLeaderTeleop(motor_ids, args.baudrate)
    
    try:
        # SETTING UP NON_PUBNUB STREAMING:
        context = zmq.Context()
        teleop.zmq_socket = context.socket(zmq.PUSH)
        teleop.zmq_socket.connect("tcp://10.1.10.85:5000")
        print("Successfully connected to zmq")
        
        # Setup
        # teleop.setup_pubnub()
        teleop.connect_leader()
        
        # Run main loop
        teleop.teleoperation_loop()
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        teleop.shutdown()
        
    return 0


if __name__ == "__main__":
    sys.exit(main())