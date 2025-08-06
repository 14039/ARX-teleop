# ARX Teleoperation Instructions

## Agora Setup
Current Agora credentials are based on nate@vassarrobotics.com agora account:
- **Project Name:** teleop-stream-1
- **App ID:** d1b381fe495547cc867a343c1eceef5d
- **Primary Certificate:** db2813337e8b46bcb271cd544f19bd63
- **Temp Token Channel Name:** nate-rnd-250804
- **Temp Token:** 007eJxTYJD7UxMYwmq62WDvmX+iQpkHJW5veC207X3kJd5VXd1Zp+IUGFIMk4wtDNNSTSxNTU3Mk5MtzMwTjU2Mkw1Tk1NT00xTrB5NyGgIZGRYv9+ZkZEBAkF8foa8xJJU3aK8FF0jUwMLAxMGBgDySyNg

⚠️ **Note:** Tokens expire - regenerate if you get "CAN_NOT_GET_GATEWAY_SERVER" errors

## ARX Keyboard Control

### Prerequisites
- ARX R5 arm connected via CAN adapter and powered on
- Python 3 with numpy installed
- CAN interface tools installed (`sudo apt install can-utils`)
- ARX libraries available (included in arx_control/lib/)

### Quick Start

#### 1. Set up CAN interface
```bash
# Find your CAN adapter device (usually /dev/ttyACM0 or /dev/ttyACM1)
ls /dev/ttyACM*

# Set up CAN interface (replace /dev/ttyACM0 with your device)
sudo slcand -o -f -s8 /dev/ttyACM1 can0
sudo ip link set can0 up
```

#### 2. Run keyboard control
```bash
# Use the provided launcher script (handles all environment setup)
./run_keyboard_control.sh
```

### Keyboard Controls
- `w/s` - Move forward/backward (X axis)
- `a/d` - Move left/right (Y axis)
- `↑/↓` arrows - Move up/down (Z axis)
- `m/n` - Rotate roll
- `l/.` - Rotate pitch  
- `,//` - Rotate yaw
- `c` - Close gripper, `o` - Open gripper
- `i` - Enable gravity compensation mode
- `r` - Return to home position
- `q` - Quit program

## ARX Teleoperation

### Leader Calibration

#### Prerequisites
- Leader arm (<9V power supply) connected via USB
- Python 3 with pyserial and scservo_sdk installed

#### Quick Calibration
1. **Connect leader arm** via USB
2. **Update port** in `single_arx_leader_calib.py` if needed (default: `/dev/tty.usbmodem5A460813891`)
3. **Run calibration:**
   ```bash
   python3 single_arx_leader_calib.py
   ```
4. **Position the arm** in desired home pose when prompted
5. **Save calibration** - creates `arx_leader_calibration.json`

The calibration file contains:
- Home positions for each servo motor
- Motor inversion settings (motors 3 & 4 are inverted by default)
- Voltage readings for leader identification

#### Technical Details
- Servo resolution: 4096 (0-4095 range)
- 7 motors total: 6 arm joints + 1 gripper
- Position mapping: `(tic_pos - servo_center) * (2π / 4095)` converts to radians
- Gripper mapping: servo tics → [-1.0, 1.0] range for ARX gripper command

### Running Teleoperation

#### Leader Side
```bash
# Option 1: Use script
./run_leader.sh

# Option 2: Direct
python3 teleop_single_arx_leader.py
```

#### Follower Side
```bash
# Option 1: Use script  
./run_follower.sh

# Option 2: Direct
python3 teleop_single_arx_follower.py --calibration_file arx_leader_calibration.json
```

### Network Configuration
PubNub configuration is in `pubnub_config.py`. Update subscribe/publish keys as needed.

### Motor Mapping
- **Motors 1-6:** Arm joints (sent to `set_joint_positions()`)
- **Motor 7:** Gripper (sent to `set_catch_pos()`)
- **Motors 3 & 4:** Inverted in calibration file for proper follower mapping