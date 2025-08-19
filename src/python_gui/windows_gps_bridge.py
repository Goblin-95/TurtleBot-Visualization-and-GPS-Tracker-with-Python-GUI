#!/usr/bin/env python3
"""
FIXED GPS Bridge - Properly scales small GPS coordinate changes
"""

import socket
import time
import json
import os

# === Configuration ===
GPS_DATA_PATH = r"C:\Users\georg\OneDrive\Desktop\TurtleBotGPS\gps_data.json"
HOST = '127.0.0.1'
PORT = 65432

print("🐢 FIXED GPS Bridge Starting...")
print(f"📁 GPS file: {GPS_DATA_PATH}")

class FixedGPSBridge:
    def __init__(self):
        # Set a fixed reference point (first GPS reading will be center)
        self.reference_lat = None
        self.reference_lon = None
        self.first_reading = True
        
        # Scale factor - how much to amplify GPS coordinate differences
        # This converts tiny GPS changes into visible GUI movement
        self.scale_factor = 5000  # Amplifies small GPS changes
        
    def convert_gps_to_gui(self, lat, lon):
        """Convert GPS to GUI coordinates with fixed scaling"""
        
        # Set reference point from first reading
        if self.reference_lat is None or self.reference_lon is None:
            self.reference_lat = lat
            self.reference_lon = lon
            print(f"📍 Reference GPS set: ({lat:.8f}, {lon:.8f})")
            return 50.0, 50.0  # Start at center
        
        # Calculate differences from reference point
        lat_diff = lat - self.reference_lat
        lon_diff = lon - self.reference_lon
        
        print(f"🔢 GPS diffs: lat={lat_diff:.8f}, lon={lon_diff:.8f}")
        
        # Convert to GUI coordinates (50,50 is center)
        gui_x = 50 + (lat_diff * self.scale_factor)
        gui_y = 50 + (lon_diff * self.scale_factor)
        
        # Keep within GUI bounds (10-90 to stay visible)
        gui_x = max(10, min(90, gui_x))
        gui_y = max(10, min(90, gui_y))
        
        return round(gui_x, 2), round(gui_y, 2)

def read_gps_file():
    """Read GPS JSON file"""
    try:
        if not os.path.exists(GPS_DATA_PATH):
            print(f"❌ GPS file not found: {GPS_DATA_PATH}")
            return None
            
        with open(GPS_DATA_PATH, 'r') as file:
            content = file.read().strip()
            
        if content.startswith('['):
            gps_data = json.loads(content)
        else:
            lines = [line.strip().rstrip(',') for line in content.split('\n') if line.strip()]
            gps_data = []
            for line in lines:
                try:
                    if line:
                        gps_data.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        
        return gps_data
        
    except Exception as e:
        print(f"❌ Error reading GPS file: {e}")
        return None

def connect_to_gui():
    """Connect to GUI"""
    print("🔗 Connecting to GUI...")
    
    attempts = 0
    while attempts < 10:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            print("✅ Connected to GUI!")
            return sock
        except ConnectionRefusedError:
            attempts += 1
            print(f"⏳ GUI not ready (attempt {attempts}/10)...")
            time.sleep(1)
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return None
    
    print("❌ Failed to connect to GUI!")
    return None

def send_position(sock, x, y):
    """Send position to GUI"""
    try:
        message = f"{x},{y}"
        sock.sendall(message.encode())
        return True
    except Exception as e:
        print(f"❌ Error sending: {e}")
        return False

def monitor_live_gps(sock):
    """Monitor GPS file for changes with FIXED scaling"""
    print("\n📡 FIXED GPS MONITORING")
    print("=" * 50)
    
    bridge = FixedGPSBridge()
    last_size = 0
    last_position = None
    
    print("🔄 Monitoring GPS file for changes...")
    print("🤖 Move your TurtleBot with teleop to see movement!")
    print("⏹️  Press Ctrl+C to stop")
    print()
    
    while True:
        try:
            if os.path.exists(GPS_DATA_PATH):
                current_size = os.path.getsize(GPS_DATA_PATH)
                
                if current_size > last_size:
                    gps_data = read_gps_file()
                    
                    if gps_data:
                        # Get latest reading
                        latest = gps_data[-1]
                        lat = latest.get('latitude', 0)
                        lon = latest.get('longitude', 0)
                        
                        # Check if position changed
                        current_position = (lat, lon)
                        if (last_position is None or 
                            abs(lat - last_position[0]) > 1e-10 or 
                            abs(lon - last_position[1]) > 1e-10):
                            
                            gui_x, gui_y = bridge.convert_gps_to_gui(lat, lon)
                            
                            if send_position(sock, gui_x, gui_y):
                                print(f"📍 GPS({lat:.8f}, {lon:.8f}) → GUI({gui_x}, {gui_y}) [Points: {len(gps_data)}]")
                                last_position = current_position
                            else:
                                print("💔 Connection lost!")
                                break
                    
                    last_size = current_size
            
            time.sleep(0.1)  # Check frequently
            
        except KeyboardInterrupt:
            print("\n⏹️  Monitoring stopped")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            time.sleep(1)

def main():
    print("🚀 FIXED GPS Bridge")
    print("=" * 30)
    
    # Connect to GUI
    sock = connect_to_gui()
    if not sock:
        return
    
    print("\n🎮 Make sure your GUI is in GPS Map mode!")
    print("🤖 Start moving your TurtleBot with teleop!")
    print("📍 First GPS reading will be center (50,50)")
    print("🎯 Robot movement will show as movement from center")
    
    try:
        monitor_live_gps(sock)
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if sock:
            sock.close()
        print("👋 Done!")

if __name__ == "__main__":
    main()