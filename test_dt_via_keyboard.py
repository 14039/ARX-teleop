#!/usr/bin/env python3
"""
DT Tank Drive Keyboard Control Script

This script combines motor finding and tank drive control functionality.
It first scans for available motors on the CAN bus, then provides
keyboard-based control for the RS03 tank drive system with Z-axis control.

Controls an RS03 tank drive system using keyboard input with:
- WASD for tank drive movement
- Q/E for Z-axis up/down movement

Requirements:
- RS03 motors connected via CAN adapter
- CAN interface configured (e.g., can0)
- PyGame for keyboard input
- CANopen library

Usage:
    python test_dt_via_keyboard.py [can_interface]
    
Example:
    python test_dt_via_keyboard.py can0
"""

import socket
import canopen
import os
import time
import pygame
import sys

# CAN bus configuration (may be overridden via environment variables)
CAN_CHANNEL: str = os.getenv("CAN_CHANNEL", "can0")
CAN_BITRATE: int = int(os.getenv("CAN_BITRATE", "1000000"))  # 1 Mbps

# Motor configuration
LEFT_MOTOR_ID = 126
RIGHT_MOTOR_ID = 127
Z_MOTOR_ID = 1  # Z-axis motor ID

# CANopen object dictionary indices
CONTROLWORD = 0x6040
STATUSWORD = 0x6041
MODES_OF_OPERATION = 0x6060
TARGET_TORQUE = 0x6071
TARGET_VELOCITY = 0x60FF
VELOCITY_ACTUAL = 0x606C
TARGET_POSITION = 0x607A
PROFILE_VELOCITY = 0x6081
PROFILE_ACCELERATION = 0x6083
POSITION_ACTUAL = 0x6064

# Control modes
VELOCITY_MODE = 3
POSITION_MODE_PP = 1  # Profile Position mode

# Motor states
SWITCH_ON_DISABLED = 0x40
READY_TO_SWITCH_ON = 0x21
SWITCHED_ON = 0x23
OPERATION_ENABLE = 0x27

# Speed configuration (in RPM)
MAX_SPEED_RPM = 100  # Maximum speed in RPM
TURN_SPEED_FACTOR = 0.7  # Reduce speed when turning

# Z-axis configuration
Z_PROFILE_VELOCITY_RPM = 50  # Speed for Z-axis movements
Z_PROFILE_ACCELERATION_RPM_S = 200  # Acceleration for Z-axis
Z_POSITION_INCREMENT = 8192  # Position increment per key press (0.5 revolution)


def scan_for_motors(can_channel=None, can_bitrate=None):
    """
    Scans the CAN bus for available nodes and prints their IDs.
    Returns True if scan was successful, False otherwise.
    """
    if can_channel is None:
        can_channel = CAN_CHANNEL
    if can_bitrate is None:
        can_bitrate = CAN_BITRATE
        
    network = canopen.Network()
    print(f"Connecting to CAN bus ({can_channel}, {can_bitrate} bit/s)…")
    
    try:
        network.connect(
            bustype="socketcan",
            channel=can_channel,
            bitrate=can_bitrate,
        )

        print("Scanning for nodes…")
        network.scanner.search(limit=127)

        # Give nodes some time to respond to the search request
        time.sleep(2)

        if not network.scanner.nodes:
            print("No nodes found on the network.")
            return False
        else:
            print("Found the following node IDs:")
            for node_id in sorted(network.scanner.nodes):
                print(f"- {node_id}")
            return True

    except Exception as e:
        print(f"An error occurred during motor scan: {e}")
        print(
            "Please ensure the CAN interface is configured correctly "
            f"(e.g., `sudo ip link set {can_channel} up type can bitrate {can_bitrate}`)"
        )
        return False
    finally:
        network.disconnect()
        print("Motor scan complete.")


class TankDrive:
    def __init__(self, can_interface='can0', bitrate=1000000):
        """Initialize tank drive controller"""
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 320))
        pygame.display.set_caption("RS03 Tank Drive + Z-Axis Control")
        self.clock = pygame.time.Clock()
        
        # Initialize CANopen network
        self.network = canopen.Network()
        
        try:
            # Connect to CAN interface
            self.network.connect(interface='socketcan', channel=can_interface, bitrate=bitrate)
            print(f"Connected to CAN interface: {can_interface}")
        except Exception as e:
            print(f"Failed to connect to CAN interface: {e}")
            print("Make sure the CAN interface is configured and up:")
            print(f"  sudo ip link set {can_interface} type can bitrate {bitrate}")
            print(f"  sudo ip link set {can_interface} up")
            sys.exit(1)
        
        # Add motor nodes - need to access .eds file from chassis_control_example directory
        eds_path = os.path.join(os.path.dirname(__file__), 'chassis_control', 'rs03.eds')
        self.left_motor = self.network.add_node(LEFT_MOTOR_ID, eds_path)
        self.right_motor = self.network.add_node(RIGHT_MOTOR_ID, eds_path)
        self.z_motor = self.network.add_node(Z_MOTOR_ID, eds_path)
        
        # Initialize motors
        self.init_motors()
        
        # Control state
        self.running = True
        self.left_speed = 0
        self.right_speed = 0
        self.z_position = 0  # Track current Z position
        
    def init_motors(self):
        """Initialize motors - tank drive motors to velocity mode, Z motor to position mode"""
        # Initialize tank drive motors in velocity mode
        tank_motors = [
            (self.left_motor, "Left"),
            (self.right_motor, "Right")
        ]
        
        for motor, name in tank_motors:
            try:
                print(f"Initializing {name} motor (ID: {motor.id})...")
                
                # First disable motor (set to SWITCH_ON_DISABLED state)
                motor.sdo[CONTROLWORD].raw = 0
                time.sleep(0.1)
                
                # Set velocity mode
                motor.sdo[MODES_OF_OPERATION].raw = VELOCITY_MODE
                
                # Set max torque (1000 = 100% = 20 N·m for RS03)
                motor.sdo[TARGET_TORQUE].raw = 1000
                
                # Enable motor operation
                motor.sdo[CONTROLWORD].raw = 15
                
                # Check status
                status = motor.sdo[STATUSWORD].raw
                if status & 0x6F == OPERATION_ENABLE:
                    print(f"{name} motor enabled successfully")
                else:
                    print(f"{name} motor status: 0x{status:04X}")
                    
            except Exception as e:
                print(f"Error initializing {name} motor: {e}")
                
        # Initialize Z motor in position mode
        try:
            print(f"Initializing Z motor (ID: {self.z_motor.id})...")
            
            # First disable motor
            self.z_motor.sdo[CONTROLWORD].raw = 0
            time.sleep(0.1)
            
            # Set position mode (Profile Position)
            self.z_motor.sdo[MODES_OF_OPERATION].raw = POSITION_MODE_PP
            
            # Set motion parameters
            self.z_motor.sdo[TARGET_TORQUE].raw = 1000  # Max torque
            self.z_motor.sdo[PROFILE_VELOCITY].raw = int(Z_PROFILE_VELOCITY_RPM * 10)  # Convert to 0.1 RPM units
            self.z_motor.sdo[PROFILE_ACCELERATION].raw = int(Z_PROFILE_ACCELERATION_RPM_S * 10)  # Convert to 0.1 RPM/s units
            
            # Enable motor operation
            self.z_motor.sdo[CONTROLWORD].raw = 15
            
            # Get current position
            self.z_position = self.z_motor.sdo[POSITION_ACTUAL].raw
            print(f"Z motor initialized at position: {self.z_position}")
            
            # Check status
            status = self.z_motor.sdo[STATUSWORD].raw
            if status & 0x6F == OPERATION_ENABLE:
                print("Z motor enabled successfully")
            else:
                print(f"Z motor status: 0x{status:04X}")
                
        except Exception as e:
            print(f"Error initializing Z motor: {e}")
                
    def stop_motors(self):
        """Stop all motors"""
        try:
            # Set velocity to 0 for tank drive motors
            self.left_motor.sdo[TARGET_VELOCITY].raw = 0
            self.right_motor.sdo[TARGET_VELOCITY].raw = 0
            
            # Disable all motors
            self.left_motor.sdo[CONTROLWORD].raw = 0
            self.right_motor.sdo[CONTROLWORD].raw = 0
            self.z_motor.sdo[CONTROLWORD].raw = 0
            
        except Exception as e:
            print(f"Error stopping motors: {e}")
            
    def set_motor_speeds(self, left_rpm, right_rpm):
        """Set motor speeds in RPM"""
        try:
            # Convert RPM to 0.1 RPM units (as per manual)
            left_value = int(left_rpm * 10)
            right_value = int(right_rpm * 10)
            
            # Set target velocities
            self.left_motor.sdo[TARGET_VELOCITY].raw = left_value
            self.right_motor.sdo[TARGET_VELOCITY].raw = right_value
            
        except Exception as e:
            print(f"Error setting motor speeds: {e}")
            
    def move_z_axis(self, direction):
        """Move Z axis up or down"""
        try:
            # Calculate new position
            if direction == "up":
                self.z_position += Z_POSITION_INCREMENT
            elif direction == "down":
                self.z_position -= Z_POSITION_INCREMENT
                
            # Set target position
            self.z_motor.sdo[TARGET_POSITION].raw = self.z_position
            
        except Exception as e:
            print(f"Error moving Z axis: {e}")
            
    def handle_input(self, events):
        """Handle WASD keyboard input for tank drive and QE for Z-axis"""
        keys = pygame.key.get_pressed()
        
        # Calculate base speeds for tank drive
        forward = 0
        turn = 0
        
        if keys[pygame.K_w]:
            forward = MAX_SPEED_RPM
        elif keys[pygame.K_s]:
            forward = -MAX_SPEED_RPM
            
        if keys[pygame.K_a]:
            turn = -MAX_SPEED_RPM * TURN_SPEED_FACTOR
        elif keys[pygame.K_d]:
            turn = MAX_SPEED_RPM * TURN_SPEED_FACTOR
            
        # Calculate individual motor speeds for tank drive
        # Left motor: forward + turn
        # Right motor: forward - turn
        self.left_speed = forward + turn
        self.right_speed = forward - turn
        
        # Apply speeds
        self.set_motor_speeds(self.left_speed, self.right_speed)
        
        # Handle Z-axis movement (single press detection)
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.move_z_axis("up")
                elif event.key == pygame.K_e:
                    self.move_z_axis("down")
        
    def draw_status(self):
        """Draw status information on screen"""
        self.screen.fill((0, 0, 0))
        
        font = pygame.font.Font(None, 36)
        
        # Title
        title = font.render("RS03 Tank Drive + Z", True, (255, 255, 255))
        self.screen.blit(title, (80, 20))
        
        # Instructions
        font_small = pygame.font.Font(None, 24)
        instructions = [
            "W - Forward",
            "S - Backward", 
            "A - Turn Left",
            "D - Turn Right",
            "Q - Z-axis Up",
            "E - Z-axis Down",
            "ESC - Exit"
        ]
        
        y = 80
        for instruction in instructions:
            text = font_small.render(instruction, True, (200, 200, 200))
            self.screen.blit(text, (20, y))
            y += 25
            
        # Motor speeds
        left_text = font_small.render(f"Left Motor: {self.left_speed:.1f} RPM", True, (0, 255, 0))
        right_text = font_small.render(f"Right Motor: {self.right_speed:.1f} RPM", True, (0, 255, 0))
        self.screen.blit(left_text, (20, 250))
        self.screen.blit(right_text, (200, 250))
        
        # Z-axis position
        z_text = font_small.render(f"Z Position: {self.z_position} ({self.z_position/16384:.2f} rev)", True, (0, 255, 255))
        self.screen.blit(z_text, (20, 275))
        
        pygame.display.flip()
        
    def run(self):
        """Main control loop"""
        print("Tank drive control started.")
        print("Use WASD keys for tank drive, Q/E for Z-axis up/down.")
        print("Press ESC to exit.")
        
        try:
            while self.running:
                # Get all events
                events = pygame.event.get()
                
                # Handle system events
                for event in events:
                    if event.type == pygame.QUIT:
                        self.running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            self.running = False
                            
                # Handle continuous input and pass events for single-press detection
                self.handle_input(events)
                
                # Update display
                self.draw_status()
                
                # Control loop rate (50 Hz)
                self.clock.tick(50)
                
        except KeyboardInterrupt:
            print("\nControl interrupted by user")
            
        finally:
            # Clean shutdown
            print("Stopping motors...")
            self.stop_motors()
            
            # Disconnect from CAN
            self.network.disconnect()
            
            # Quit pygame
            pygame.quit()
            print("Tank drive control stopped.")


def main():
    """Main function - scans for motors then starts keyboard control"""
    
    # Check if CAN interface is specified
    can_interface = 'can1'
    if len(sys.argv) > 1:
        can_interface = sys.argv[1]
    
    print("=== DT Tank Drive Keyboard Control ===")
    print(f"Using CAN interface: {can_interface}")
    print()
    
    # First, scan for available motors
    print("Step 1: Scanning for motors...")
    scan_success = scan_for_motors(can_interface, CAN_BITRATE)
    
    if not scan_success:
        print("Motor scan failed. Please check your CAN interface configuration.")
        print("Exiting...")
        return
    
    print()
    print("Step 2: Starting tank drive control...")
    print("Make sure motors with IDs 126 (left), 127 (right), and 1 (Z-axis) are connected.")
    
    # Ask user if they want to continue
    try:
        response = input("Continue with tank drive control? (y/N): ").strip().lower()
        if response not in ['y', 'yes']:
            print("Exiting...")
            return
    except KeyboardInterrupt:
        print("\nExiting...")
        return
    
    # Create and run tank drive controller
    try:
        controller = TankDrive(can_interface=can_interface, bitrate=CAN_BITRATE)
        controller.run()
    except Exception as e:
        print(f"Error starting tank drive controller: {e}")
        print("Make sure:")
        print("1. RS03 motors are connected and powered on")
        print("2. CAN interface is properly configured")
        print("3. You have necessary permissions for CAN access")
        print("4. The rs03.eds file exists in chassis_control/")


if __name__ == "__main__":
    main()