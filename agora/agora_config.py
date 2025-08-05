"""
Agora Video Streaming Configuration
"""

# Agora Credentials (Get these from https://console.agora.io)
APP_ID = "d1b381fe495547cc867a343c1eceef5d"  # User's Agora App ID
APP_CERTIFICATE = "db2813337e8b46bcb271cd544f19bd63"  # Primary certificate

# Token Configuration for secure channels
USE_TOKEN = True  # Set to True when using tokens
TOKEN = "007eJxTYPirH82jb9d4J0vSb3/RvtjCkjm7RdtXOD/Iv/c5beex3xsVGFIMk4wtDNNSTSxNTU3Mk5MtzMwTjU2Mkw1Tk1NT00xTVmRPymgIZGSwXBLOzMgAgSA+P0NeYkmqblFeiq6RqYGFgSkDAwAjaCPP"
TOKEN_CHANNEL = "nate-rnd-250805"  # Channel name for the token

# Channel Configuration
CHANNEL_PREFIX = "robot_cam_"  # Prefix for video channels
VIDEO_CHANNELS = {
    "camera1": f"{CHANNEL_PREFIX}1",
    "camera2": f"{CHANNEL_PREFIX}2", 
    "camera3": f"{CHANNEL_PREFIX}3"
}

# When using token, use the same channel but different UIDs for each camera
if USE_TOKEN:
    # For token-based auth, we'll use the same channel but different UIDs
    VIDEO_CHANNELS = {
        "camera1": TOKEN_CHANNEL,
        "camera2": TOKEN_CHANNEL,
        "camera3": TOKEN_CHANNEL
    }
    # UIDs for each camera (must be unique per channel)
    CAMERA_UIDS = {
        "camera1": 1001,
        "camera2": 1002,
        "camera3": 1003
    }
else:
    CAMERA_UIDS = {
        "camera1": None,
        "camera2": None,
        "camera3": None
    }

# Video Configuration - 480p @ 30fps
VIDEO_PROFILE = {
    "width": 640,   # 480p width
    "height": 480,  # 480p height
    "frameRate": 30,
    "bitrate": 800  # Reduced bitrate for 480p
}

# Audio Configuration (disabled for video-only streaming)
ENABLE_AUDIO = False

# Network Configuration
ENABLE_DUAL_STREAM = True  # Enable dual stream for bandwidth adaptation
LOW_STREAM_PARAMETER = {
    "width": 320,
    "height": 240,
    "frameRate": 15,
    "bitrate": 200
}

# Recording Configuration
ENABLE_CLOUD_RECORDING = False  # Set to True to enable cloud recording
RECORDING_BUCKET = ""  # S3 bucket name if using cloud recording
RECORDING_REGION = ""  # AWS region for recording

# UI Configuration
WINDOW_TITLE_LEADER = "Robot Video Feeds - Leader"
WINDOW_TITLE_FOLLOWER = "Robot Video Feeds - Follower"
GRID_ROWS = 2
GRID_COLS = 2  # 2x2 grid for 3 cameras + 1 empty slot 