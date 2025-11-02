"""
Dual Crutch HMI Data Logger - Serial Port Version
==================================================
Connects to two Bluetooth crutches via COM ports, waits for MC commands, 
then logs sensor data until AN command is received.

Requirements:
- Windows 10/11
- Python 3.7+
- pyserial library

Install: pip install pyserial

SETUP:
1. Pair both Bluetooth devices in Windows Settings
2. Find COM port numbers in Device Manager > Ports (COM & LPT)
3. Update COM_PORT_CRUTCH_1 and COM_PORT_CRUTCH_2 below
"""

import serial
import serial.tools.list_ports
import time
import csv
import os
import sys
from datetime import datetime
from threading import Thread, Event
from typing import Optional, Dict, List


# ============= CONFIGURATION =============
# Update these COM port numbers after pairing devices
COM_PORT_CRUTCH_1 = "COM10"  # Change to your actual COM port for CrutchHMI-BT-7036
COM_PORT_CRUTCH_2 = "COM12"  # Change to your actual COM port for CrutchHMI-BT-74d3
BAUD_RATE = 921600
WAIT_AFTER_MC_SECONDS = 5
LOG_FOLDER = "crutch_logs"

CRUTCH_1_NAME = "CrutchHMI-BT-7036"
CRUTCH_2_NAME = "CrutchHMI-BT-74d3"
# ==========================================


class CrutchDevice:
    """Represents a single crutch device"""
    
    def __init__(self, name: str, port: str):
        self.name = name
        self.port = port
        self.serial: Optional[serial.Serial] = None
        self.mc_received = False
        self.mc_timestamp: Optional[datetime] = None
        self.data_log: List[Dict] = []
        self.connected = False
        self.thread: Optional[Thread] = None
        self.stop_event = Event()
        self.buffer = ""
        
    def __str__(self):
        return f"{self.name} ({self.port})"


class DualCrutchLogger:
    """Main logging application"""
    
    def __init__(self):
        self.crutch_1: Optional[CrutchDevice] = None
        self.crutch_2: Optional[CrutchDevice] = None
        self.logging_started = False
        self.logging_stopped = False
        self.log_start_time: Optional[datetime] = None
        
    def list_com_ports(self):
        """List all available COM ports"""
        print("\n📡 Available COM ports:")
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            print("   No COM ports found")
            return
        
        for port in ports:
            print(f"   {port.device}: {port.description}")
        print()
    
    def connect_device(self, device: CrutchDevice) -> bool:
        """Connect to a single crutch device"""
        print(f"📡 Connecting to {device.name} on {device.port}...")
        
        try:
            device.serial = serial.Serial(
                port=device.port,
                baudrate=BAUD_RATE,
                timeout=1,
                write_timeout=1
            )
            
            time.sleep(1)  # Give connection time to stabilize
            
            if device.serial.is_open:
                device.connected = True
                print(f"✅ Connected to {device.name}")
                return True
            else:
                print(f"❌ Failed to open {device.port}")
                return False
                
        except serial.SerialException as e:
            print(f"❌ Serial connection error for {device.name}: {e}")
            print(f"   Make sure {device.port} is correct and the device is paired")
            return False
        except Exception as e:
            print(f"❌ Unexpected error connecting to {device.name}: {e}")
            return False
    
    def parse_data_line(self, device: CrutchDevice, line: str) -> Optional[Dict]:
        """Parse a data line from the crutch
        
        Expected format (new readable format):
        timestamp,CMD,force,qw,qx,qy,qz
        Example: 519683,S,894.88,0.575,-0.418,0.570,0.408
        """
        line = line.strip()
        if not line:
            return None
        
        # Check for MC command
        if ",MC," in line or ",MC" in line or line.endswith(",MC"):
            if not device.mc_received:
                device.mc_received = True
                device.mc_timestamp = datetime.now()
                print(f"✅ MC command received from {device.name} at {device.mc_timestamp.strftime('%H:%M:%S.%f')[:-3]}")
            return None
        
        # Check for AN command (stop logging)
        if ",AN:" in line and self.logging_started:
            print(f"\n🛑 AN command received from {device.name}")
            self.logging_stopped = True
            return None
        
        # Only parse data lines if logging has started
        if not self.logging_started:
            return None
        
        # Parse data line
        try:
            parts = line.split(',')
            if len(parts) >= 7:
                timestamp = int(parts[0])
                command = parts[1]
                force = float(parts[2])
                qw = float(parts[3])
                qx = float(parts[4])
                qy = float(parts[5])
                qz = float(parts[6])
                
                data = {
                    'device': device.name,
                    'timestamp': timestamp,
                    'system_time': datetime.now(),
                    'command': command,
                    'force': force,
                    'qw': qw,
                    'qx': qx,
                    'qy': qy,
                    'qz': qz
                }
                
                return data
                
        except (ValueError, IndexError):
            # Skip malformed lines
            pass
        
        return None
    
    def read_thread(self, device: CrutchDevice):
        """Thread function to continuously read from a device"""
        print(f"🔄 Started read thread for {device.name}")
        
        while not device.stop_event.is_set():
            try:
                if device.serial and device.serial.in_waiting > 0:
                    # Read available data
                    data = device.serial.read(device.serial.in_waiting)
                    text = data.decode('utf-8', errors='ignore')
                    device.buffer += text
                    
                    # Process complete lines
                    while '\n' in device.buffer:
                        line, device.buffer = device.buffer.split('\n', 1)
                        
                        # Parse the line
                        parsed_data = self.parse_data_line(device, line)
                        
                        if parsed_data:
                            device.data_log.append(parsed_data)
                
                time.sleep(0.01)  # Small delay to prevent CPU spinning
                
            except serial.SerialException as e:
                print(f"⚠️  Serial error for {device.name}: {e}")
                break
            except Exception as e:
                print(f"⚠️  Read thread error for {device.name}: {e}")
        
        print(f"🔄 Read thread stopped for {device.name}")
    
    def start_reading(self, device: CrutchDevice):
        """Start the read thread for a device"""
        device.thread = Thread(target=self.read_thread, args=(device,), daemon=True)
        device.thread.start()
    
    def wait_for_mc_commands(self) -> bool:
        """Wait for MC commands from both devices"""
        print(f"\n⏳ Waiting for MC commands from both crutches...")
        print(f"   (Press B1 button on each crutch to send MC command)")
        print(f"   (Press Ctrl+C to cancel)\n")
        
        start_time = time.time()
        
        try:
            while not (self.crutch_1.mc_received and self.crutch_2.mc_received):
                time.sleep(0.1)
                
                # Print waiting status every 5 seconds
                elapsed = time.time() - start_time
                if int(elapsed) % 5 == 0 and int(elapsed * 10) % 10 == 0:  # Every 5 seconds
                    status_1 = "✓" if self.crutch_1.mc_received else "⏳"
                    status_2 = "✓" if self.crutch_2.mc_received else "⏳"
                    print(f"   {status_1} {self.crutch_1.name}  |  {status_2} {self.crutch_2.name}")
                
                # Timeout after 5 minutes
                if elapsed > 300:
                    print("\n⏱️  Timeout waiting for MC commands (5 minutes)")
                    return False
            
            print(f"\n✅ Both MC commands received!")
            
            # Determine which was received last
            if self.crutch_1.mc_timestamp > self.crutch_2.mc_timestamp:
                last_mc_time = self.crutch_1.mc_timestamp
                last_device = self.crutch_1.name
                time_diff = (self.crutch_1.mc_timestamp - self.crutch_2.mc_timestamp).total_seconds()
            else:
                last_mc_time = self.crutch_2.mc_timestamp
                last_device = self.crutch_2.name
                time_diff = (self.crutch_2.mc_timestamp - self.crutch_1.mc_timestamp).total_seconds()
            
            print(f"⏰ Last MC command from {last_device} at {last_mc_time.strftime('%H:%M:%S.%f')[:-3]}")
            print(f"   (Time difference: {time_diff:.3f} seconds)")
            print(f"\n⏳ Waiting {WAIT_AFTER_MC_SECONDS} seconds before starting data logging...")
            
            for i in range(WAIT_AFTER_MC_SECONDS, 0, -1):
                print(f"   {i}...", end='\r')
                time.sleep(1)
            
            self.logging_started = True
            self.log_start_time = datetime.now()
            print(f"\n📝 Data logging STARTED at {self.log_start_time.strftime('%H:%M:%S.%f')[:-3]}")
            print(f"   (Logging will stop when AN command is received from either crutch)")
            print(f"   (Or press Ctrl+C to stop manually)\n")
            
            return True
            
        except KeyboardInterrupt:
            print("\n\n⚠️  User cancelled waiting for MC commands")
            return False
    
    def monitor_logging(self):
        """Monitor logging until AN command or user interrupt"""
        last_status_time = time.time()
        
        try:
            while not self.logging_stopped:
                time.sleep(0.1)
                
                # Print status every 5 seconds
                current_time = time.time()
                if current_time - last_status_time >= 5.0:
                    elapsed = (datetime.now() - self.log_start_time).total_seconds()
                    print(f"⏱️  {int(elapsed)}s | "
                          f"{self.crutch_1.name}: {len(self.crutch_1.data_log)} samples | "
                          f"{self.crutch_2.name}: {len(self.crutch_2.data_log)} samples")
                    last_status_time = current_time
                    
        except KeyboardInterrupt:
            print("\n\n⚠️  User interrupted logging")
            self.logging_stopped = True
    
    def save_logs(self):
        """Save logged data to CSV files"""
        print(f"\n💾 Saving logs to '{LOG_FOLDER}' folder...")
        
        # Create log folder if it doesn't exist
        os.makedirs(LOG_FOLDER, exist_ok=True)
        
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Save data for each crutch
        for device in [self.crutch_1, self.crutch_2]:
            if not device.data_log:
                print(f"⚠️  No data to save for {device.name}")
                continue
            
            # Generate filename
            device_id = device.name.split('-')[-1]  # Extract ID (7036 or 74d3)
            filename = f"crutch_{device_id}_{timestamp_str}.csv"
            filepath = os.path.join(LOG_FOLDER, filename)
            
            # Write CSV
            try:
                with open(filepath, 'w', newline='') as csvfile:
                    fieldnames = ['system_time', 'device_timestamp', 'device', 'command',
                                  'force', 'qw', 'qx', 'qy', 'qz']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    
                    writer.writeheader()
                    for data in device.data_log:
                        writer.writerow({
                            'system_time': data['system_time'].strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                            'device_timestamp': data['timestamp'],
                            'device': data['device'],
                            'command': data['command'],
                            'force': f"{data['force']:.2f}",
                            'qw': f"{data['qw']:.3f}",
                            'qx': f"{data['qx']:.3f}",
                            'qy': f"{data['qy']:.3f}",
                            'qz': f"{data['qz']:.3f}"
                        })
                
                print(f"✅ Saved {len(device.data_log)} samples from {device.name} to {filename}")
                
            except Exception as e:
                print(f"❌ Error saving {device.name} data: {e}")
        
        # Create combined log
        combined_filename = f"combined_log_{timestamp_str}.csv"
        combined_filepath = os.path.join(LOG_FOLDER, combined_filename)
        
        try:
            all_data = sorted(
                self.crutch_1.data_log + self.crutch_2.data_log,
                key=lambda x: x['system_time']
            )
            
            with open(combined_filepath, 'w', newline='') as csvfile:
                fieldnames = ['system_time', 'device_timestamp', 'device', 'command',
                              'force', 'qw', 'qx', 'qy', 'qz']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                writer.writeheader()
                for data in all_data:
                    writer.writerow({
                        'system_time': data['system_time'].strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        'device_timestamp': data['timestamp'],
                        'device': data['device'],
                        'command': data['command'],
                        'force': f"{data['force']:.2f}",
                        'qw': f"{data['qw']:.3f}",
                        'qx': f"{data['qx']:.3f}",
                        'qy': f"{data['qy']:.3f}",
                        'qz': f"{data['qz']:.3f}"
                    })
            
            print(f"✅ Saved combined log with {len(all_data)} samples to {combined_filename}")
            
        except Exception as e:
            print(f"❌ Error saving combined log: {e}")
        
        print(f"\n📁 All logs saved in: {os.path.abspath(LOG_FOLDER)}")
    
    def cleanup(self):
        """Disconnect from devices"""
        print("\n🔌 Disconnecting from devices...")
        
        for device in [self.crutch_1, self.crutch_2]:
            if device:
                # Stop read thread
                if device.thread and device.thread.is_alive():
                    device.stop_event.set()
                    device.thread.join(timeout=2)
                
                # Close serial port
                if device.serial and device.serial.is_open:
                    try:
                        device.serial.close()
                        print(f"✅ Disconnected from {device.name}")
                    except Exception as e:
                        print(f"⚠️  Error disconnecting from {device.name}: {e}")
    
    def run(self):
        """Main execution flow"""
        print("=" * 70)
        print("  DUAL CRUTCH HMI DATA LOGGER (Serial Port Version)")
        print("=" * 70)
        
        try:
            # Show available COM ports
            self.list_com_ports()
            
            # Step 1: Create device objects
            self.crutch_1 = CrutchDevice(CRUTCH_1_NAME, COM_PORT_CRUTCH_1)
            self.crutch_2 = CrutchDevice(CRUTCH_2_NAME, COM_PORT_CRUTCH_2)
            
            # Step 2: Connect to both devices
            if not self.connect_device(self.crutch_1):
                print(f"\n❌ Failed to connect to {CRUTCH_1_NAME}")
                print(f"   Check that {COM_PORT_CRUTCH_1} is correct")
                return
            
            if not self.connect_device(self.crutch_2):
                print(f"\n❌ Failed to connect to {CRUTCH_2_NAME}")
                print(f"   Check that {COM_PORT_CRUTCH_2} is correct")
                self.cleanup()
                return
            
            print(f"\n✅ Both crutches connected successfully!")
            
            # Step 3: Start read threads
            print(f"\n🔄 Starting data read threads...")
            self.start_reading(self.crutch_1)
            self.start_reading(self.crutch_2)
            
            time.sleep(1)  # Give threads time to start
            
            # Step 4: Wait for MC commands
            if not self.wait_for_mc_commands():
                print("\n❌ Failed to receive MC commands from both devices")
                self.cleanup()
                return
            
            # Step 5: Monitor logging until AN command
            self.monitor_logging()
            
            # Step 6: Save logs
            print(f"\n✅ Logging stopped")
            log_duration = (datetime.now() - self.log_start_time).total_seconds()
            print(f"📊 Total logging duration: {log_duration:.1f} seconds")
            print(f"📊 {self.crutch_1.name}: {len(self.crutch_1.data_log)} samples")
            print(f"📊 {self.crutch_2.name}: {len(self.crutch_2.data_log)} samples")
            
            if len(self.crutch_1.data_log) > 0 or len(self.crutch_2.data_log) > 0:
                self.save_logs()
            else:
                print("\n⚠️  No data was logged")
            
        except KeyboardInterrupt:
            print("\n\n⚠️  Interrupted by user")
            if self.logging_started and (len(self.crutch_1.data_log) > 0 or len(self.crutch_2.data_log) > 0):
                self.save_logs()
        
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.cleanup()
            print("\n👋 Logger shutdown complete")


def main():
    """Entry point"""
    
    # Check configuration
    if COM_PORT_CRUTCH_1 == "COM3" or COM_PORT_CRUTCH_2 == "COM4":
        print("\n⚠️  WARNING: COM ports are set to default values (COM3, COM4)")
        print("   Please update COM_PORT_CRUTCH_1 and COM_PORT_CRUTCH_2 in the script")
        print("   with the correct ports from Device Manager\n")
        
        response = input("Continue anyway? (y/n): ")
        if response.lower() != 'y':
            print("Exiting...")
            return
    
    logger = DualCrutchLogger()
    logger.run()


if __name__ == "__main__":
    print("\nStarting Dual Crutch Logger...")
    print("Make sure both crutches are powered on and paired!\n")
    
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 Goodbye!")