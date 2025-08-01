#!/usr/bin/env python3
"""
Headless Agora Streamer using Selenium WebDriver
Streams video from cameras to Agora channels without display
Designed for Orange Pi and other headless Linux systems
"""

import os
import sys
import time
import signal
import logging
import json
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.chrome.service import Service
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

# Add parent directory to import agora_config
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import agora_config

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class HeadlessAgoraStreamer:
    """Manages headless browser streaming to Agora channels"""
    
    def __init__(self):
        self.driver = None
        self.is_running = False
        self.html_path = os.path.join(os.path.dirname(__file__), 'streaming_page.html')
        
    def create_streaming_page(self, camera_index=0):
        """Create HTML page with Agora Web SDK for a specific camera"""
        
        # Get channel configuration
        channel = "robot-video-1"  # Default channel
        if hasattr(agora_config, 'VIDEO_CHANNELS'):
            channels = list(agora_config.VIDEO_CHANNELS.values())
            if camera_index < len(channels):
                channel = channels[camera_index]
            else:
                channel = f"robot-video-{camera_index + 1}"
                
        # Get token if configured
        token = "null"
        if hasattr(agora_config, 'USE_TOKEN') and agora_config.USE_TOKEN:
            if hasattr(agora_config, 'TOKEN'):
                token = f'"{agora_config.TOKEN}"'
                
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Headless Agora Streaming - Camera {camera_index}</title>
    <script src="https://download.agora.io/sdk/release/AgoraRTC_N-4.20.0.js"></script>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 20px;
            background-color: #f0f0f0;
        }}
        #status {{
            background-color: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        .success {{ color: #4CAF50; }}
        .error {{ color: #f44336; }}
        .info {{ color: #2196F3; }}
    </style>
</head>
<body>
    <h1>Headless Agora Streaming</h1>
    <div id="status">
        <h2>Status</h2>
        <div id="app-id">App ID: {agora_config.APP_ID[:8]}...</div>
        <div id="channel-name">Channel: {channel}</div>
        <div id="camera-status">Camera: Initializing...</div>
        <div id="stream-status">Stream: Not started</div>
        <div id="error-message"></div>
    </div>
    
    <script>
        // Configuration
        const APP_ID = '{agora_config.APP_ID}';
        const CHANNEL = '{channel}';
        const TOKEN = {token};
        const CAMERA_INDEX = {camera_index};
        
        // Initialize Agora
        console.log('Initializing Agora SDK...');
        AgoraRTC.setLogLevel(1);
        const client = AgoraRTC.createClient({{ mode: 'rtc', codec: 'vp8' }});
        
        // Update status in UI
        function updateStatus(elementId, message, className = 'info') {{
            const element = document.getElementById(elementId);
            if (element) {{
                element.textContent = message;
                element.className = className;
            }}
            console.log(`[${{elementId}}] ${{message}}`);
        }}
        
        async function startStreaming() {{
            try {{
                updateStatus('stream-status', 'Joining channel...', 'info');
                
                // Join channel
                await client.join(APP_ID, CHANNEL, TOKEN, null);
                updateStatus('stream-status', 'Joined channel successfully', 'success');
                
                // Get available cameras
                const devices = await AgoraRTC.getDevices();
                const cameras = devices.filter(device => device.kind === 'videoinput');
                
                console.log('Available cameras:', cameras);
                updateStatus('camera-status', `Found ${{cameras.length}} camera(s)`, 'info');
                
                if (cameras.length === 0) {{
                    throw new Error('No cameras detected!');
                }}
                
                // Select camera based on index
                let selectedCamera = cameras[0];  // Default to first camera
                if (CAMERA_INDEX < cameras.length) {{
                    selectedCamera = cameras[CAMERA_INDEX];
                }}
                
                updateStatus('camera-status', `Using: ${{selectedCamera.label || 'Camera ' + CAMERA_INDEX}}`, 'info');
                
                // Create video track with selected camera
                const videoTrack = await AgoraRTC.createCameraVideoTrack({{
                    cameraId: selectedCamera.deviceId,
                    encoderConfig: {{
                        width: 640,
                        height: 480,
                        frameRate: 30,
                        bitrateMax: 1000,
                        bitrateMin: 600
                    }}
                }});
                
                // Publish video track
                await client.publish([videoTrack]);
                updateStatus('stream-status', 'Streaming active', 'success');
                
                // Monitor connection state
                client.on('connection-state-change', (curState, prevState) => {{
                    console.log(`Connection state: ${{prevState}} -> ${{curState}}`);
                    updateStatus('stream-status', `Connection: ${{curState}}`, 
                                curState === 'CONNECTED' ? 'success' : 'info');
                }});
                
                // Monitor stream stats
                setInterval(async () => {{
                    const stats = client.getRTCStats();
                    console.log('Stream stats:', {{
                        SendBitrate: stats.SendBitrate,
                        RecvBitrate: stats.RecvBitrate,
                        OutgoingAvailableBandwidth: stats.OutgoingAvailableBandwidth,
                        RTT: stats.RTT
                    }});
                }}, 5000);
                
            }} catch (error) {{
                console.error('Streaming error:', error);
                updateStatus('error-message', `Error: ${{error.message}}`, 'error');
                updateStatus('stream-status', 'Stream failed', 'error');
            }}
        }}
        
        // Start streaming when page loads
        window.addEventListener('load', () => {{
            console.log('Page loaded, starting stream...');
            startStreaming();
        }});
        
        // Cleanup on page unload
        window.addEventListener('beforeunload', async () => {{
            if (client) {{
                await client.leave();
            }}
        }});
    </script>
</body>
</html>
"""
        
        # Write HTML to file
        with open(self.html_path, 'w') as f:
            f.write(html_content)
            
        logger.info(f"Created streaming page for camera {camera_index} on channel {channel}")
        
    def setup_chrome_options(self):
        """Configure Chrome options for headless operation"""
        options = Options()
        
        # Headless mode
        options.add_argument('--headless')
        options.add_argument('--disable-gpu')
        
        # Security and sandboxing
        options.add_argument('--no-sandbox')
        options.add_argument('--disable-setuid-sandbox')
        options.add_argument('--disable-dev-shm-usage')
        
        # Media permissions
        options.add_argument('--use-fake-ui-for-media-stream')
        options.add_argument('--use-fake-device-for-media-stream')
        
        # Performance optimizations
        options.add_argument('--disable-software-rasterizer')
        options.add_argument('--disable-extensions')
        options.add_argument('--disable-plugins')
        options.add_argument('--disable-images')
        options.add_argument('--disable-javascript-harmony-shipping')
        
        # Additional ARM/resource optimizations
        options.add_argument('--disable-web-security')
        options.add_argument('--disable-features=VizDisplayCompositor')
        options.add_argument('--disable-breakpad')
        options.add_argument('--disable-features=TranslateUI')
        options.add_argument('--disable-ipc-flooding-protection')
        options.add_argument('--disable-renderer-backgrounding')
        options.add_argument('--disable-field-trial-config')
        options.add_argument('--disable-backgrounding-occluded-windows')
        options.add_argument('--disable-features=site-per-process')
        
        # Memory optimization
        options.add_argument('--memory-pressure-off')
        options.add_argument('--js-flags=--max-old-space-size=512')
        
        # Crash/hang prevention
        options.add_argument('--disable-hang-monitor')
        options.add_argument('--disable-prompt-on-repost')
        options.add_argument('--disable-sync')
        options.add_argument('--disable-domain-reliability')
        options.add_argument('--disable-client-side-phishing-detection')
        
        # Set window size
        options.add_argument('--window-size=1280,720')
        
        # Media permissions preferences
        prefs = {
            "profile.default_content_setting_values.media_stream_camera": 1,
            "profile.default_content_setting_values.media_stream_mic": 1,
            "profile.default_content_setting_values.notifications": 2
        }
        options.add_experimental_option("prefs", prefs)
        
        # Suppress logging
        options.add_experimental_option('excludeSwitches', ['enable-logging'])
        options.add_argument('--log-level=3')
        
        # Enable verbose logging for debugging
        options.add_argument('--enable-logging=stderr')
        options.add_argument('--v=1')
        
        return options
        
    def find_chrome_binary(self):
        """Find Chrome/Chromium binary on the system"""
        possible_paths = [
            '/usr/bin/chromium-browser',
            '/usr/bin/chromium',
            '/usr/bin/chromium-bsu',
            '/usr/bin/google-chrome',
            '/usr/bin/google-chrome-stable',
            '/snap/bin/chromium',
            '/usr/local/bin/chromium',
            '/opt/chromium/chromium',
            # Add common ARM paths
            '/usr/lib/chromium-browser/chromium-browser',
            '/usr/lib/chromium/chromium'
        ]
        
        # Also check PATH
        for cmd in ['chromium-browser', 'chromium', 'chromium-bsu', 'google-chrome']:
            try:
                import shutil
                path = shutil.which(cmd)
                if path:
                    logger.info(f"Found Chrome binary in PATH: {path}")
                    return path
            except:
                pass
        
        # Check hardcoded paths
        for path in possible_paths:
            if os.path.exists(path):
                logger.info(f"Found Chrome binary at: {path}")
                return path
                
        # Provide more helpful error message
        error_msg = """Chrome/Chromium binary not found. Please install it using one of:
        - sudo apt-get install chromium
        - sudo apt-get install chromium-bsu
        - sudo snap install chromium
        - Or check the installation guide in README_HEADLESS.md"""
        raise Exception(error_msg)
        
    def find_chromedriver(self):
        """Find ChromeDriver binary on the system"""
        possible_paths = [
            '/usr/bin/chromedriver',
            '/usr/local/bin/chromedriver',
            '/usr/lib/chromium-browser/chromedriver',
            '/usr/lib/chromium/chromedriver',
            '/snap/bin/chromium.chromedriver',
            # Common ARM paths
            '/usr/lib/chromium-browser/chromedriver',
            '/usr/lib/aarch64-linux-gnu/chromium-browser/chromedriver'
        ]
        
        # Check PATH first
        try:
            import shutil
            path = shutil.which('chromedriver')
            if path:
                logger.info(f"Found ChromeDriver in PATH: {path}")
                return path
        except:
            pass
            
        # Check hardcoded paths
        for path in possible_paths:
            if os.path.exists(path):
                logger.info(f"Found ChromeDriver at: {path}")
                return path
                
        return None
        
    def start_streaming(self, camera_index=0):
        """Start streaming from specified camera"""
        try:
            logger.info(f"Starting headless streaming for camera {camera_index}...")
            
            # Create streaming page
            self.create_streaming_page(camera_index)
            
            # Setup Chrome options
            options = self.setup_chrome_options()
            
            # Find Chrome binary
            chrome_binary = self.find_chrome_binary()
            options.binary_location = chrome_binary
            
            # Find ChromeDriver
            chromedriver_path = self.find_chromedriver()
            
            # Create driver with better error handling
            logger.info("Launching headless Chrome...")
            logger.info(f"Chrome binary: {chrome_binary}")
            if chromedriver_path:
                logger.info(f"ChromeDriver path: {chromedriver_path}")
            
            try:
                if chromedriver_path:
                    # Use specific ChromeDriver path
                    from selenium.webdriver.chrome.service import Service
                    service = Service(chromedriver_path)
                    service.log_path = '/tmp/chromedriver.log'  # Enable ChromeDriver logging
                    self.driver = webdriver.Chrome(service=service, options=options)
                else:
                    # Try default (relies on PATH or selenium-manager)
                    self.driver = webdriver.Chrome(options=options)
                    
                logger.info("Chrome driver created successfully")
                
            except Exception as e:
                # Log the full exception details
                import traceback
                logger.error(f"Failed to create Chrome driver: {str(e)}")
                logger.error(f"Exception type: {type(e).__name__}")
                logger.error(f"Traceback:\n{traceback.format_exc()}")
                
                # Check if ChromeDriver log exists
                if os.path.exists('/tmp/chromedriver.log'):
                    with open('/tmp/chromedriver.log', 'r') as f:
                        logger.error(f"ChromeDriver log:\n{f.read()}")
                
                # Provide detailed error message
                error_msg = f"""Failed to create Chrome driver: {str(e)}
                    
Please check:
1. ChromeDriver and Chrome versions match
   - Run: chromium --version
   - Run: chromedriver --version
   
2. Missing dependencies (common on ARM):
   - sudo apt-get install libnss3 libnspr4 libatk1.0-0 libatk-bridge2.0-0
   - sudo apt-get install libcups2 libdrm2 libxkbcommon0 libxcomposite1
   - sudo apt-get install libxdamage1 libxfixes3 libxrandr2 libgbm1 libasound2
   
3. Try running Chrome manually:
   - {chrome_binary} --headless --no-sandbox --disable-gpu --dump-dom https://google.com
   
4. Check system resources:
   - free -h (ensure enough memory)
   - df -h (ensure enough disk space in /tmp)"""
                raise Exception(error_msg)
            
            # Set timeouts
            self.driver.set_page_load_timeout(30)
            self.driver.implicitly_wait(10)
            
            # Load streaming page
            logger.info(f"Loading streaming page: file://{self.html_path}")
            self.driver.get(f'file://{self.html_path}')
            
            # Wait for streaming to start
            WebDriverWait(self.driver, 20).until(
                EC.text_to_be_present_in_element((By.ID, "stream-status"), "Streaming active")
            )
            
            logger.info("Streaming started successfully!")
            self.is_running = True
            
            # Monitor streaming
            while self.is_running:
                try:
                    # Get browser console logs
                    logs = self.driver.get_log('browser')
                    for log in logs:
                        if log['level'] == 'SEVERE':
                            logger.error(f"Browser: {log['message']}")
                        elif 'stats:' in log['message'].lower():
                            logger.info(f"Browser: {log['message']}")
                            
                    # Check if page is still responsive
                    self.driver.execute_script("return document.readyState")
                    
                except Exception as e:
                    logger.error(f"Monitoring error: {e}")
                    break
                    
                time.sleep(5)
                
        except Exception as e:
            logger.error(f"Failed to start streaming: {e}")
            raise
            
    def stop_streaming(self):
        """Stop streaming and cleanup"""
        logger.info("Stopping streaming...")
        self.is_running = False
        
        if self.driver:
            try:
                self.driver.quit()
            except:
                pass
                
        # Cleanup HTML file
        if os.path.exists(self.html_path):
            os.remove(self.html_path)
            
        logger.info("Streaming stopped")
        
def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    logger.info("Received shutdown signal")
    if 'streamer' in globals():
        streamer.stop_streaming()
    sys.exit(0)
    
def main():
    """Main entry point"""
    global streamer
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info("=== Headless Agora Streamer ===")
    logger.info(f"App ID: {agora_config.APP_ID[:8]}...")
    
    # Check dependencies
    try:
        import selenium
        logger.info(f"Selenium version: {selenium.__version__}")
    except ImportError:
        logger.error("Selenium not installed! Run: ./install_headless_deps.sh")
        sys.exit(1)
        
    # Create streamer instance
    streamer = HeadlessAgoraStreamer()
    
    try:
        # Start streaming from camera 0
        # You can modify this to stream from multiple cameras
        streamer.start_streaming(camera_index=0)
        
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    except Exception as e:
        logger.error(f"Streaming failed: {e}")
    finally:
        streamer.stop_streaming()
        
if __name__ == "__main__":
    main() 