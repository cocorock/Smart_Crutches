"""
Dual Crutch HMI Data Logger - Serial Port Version with Real-Time Plotting
==========================================================================
Connects to two Bluetooth crutches via COM ports, waits for MC commands, 
then logs sensor data and displays real-time plots until AN command is received.

Requirements:
- Windows 10/11
- Python 3.7+
- pyserial, numpy, matplotlib

Install: pip install pyserial numpy matplotlib

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
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# ============= CONFIGURATION =============
# Update these COM port numbers after pairing devices
COM_PORT_CRUTCH_1 = "COM10"  # CrutchHMI-BT-7036 (has joystick)
COM_PORT_CRUTCH_2 = "COM12"  # CrutchHMI-BT-74d3 (NO joystick - analog disabled)
BAUD_RATE = 921600
WAIT_AFTER_MC_SECONDS = 5
LOG_FOLDER = "crutch_logs"

CRUTCH_1_NAME = "CrutchHMI-BT-7036"  # Right crutch (with joystick)
CRUTCH_2_NAME = "CrutchHMI-BT-74d3"  # Left crutch (NO joystick)

# Device capabilities
CRUTCH_1_HAS_ANALOG = True   # 7036 has joystick
CRUTCH_2_HAS_ANALOG = False  # 74d3 does NOT have joystick

# Plotting configuration
PLOT_BUFFER_SIZE = 200  # Number of samples to show in plots
PLOT_UPDATE_INTERVAL = 50  # milliseconds
# ==========================================


def quaternion_to_euler(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.degrees(np.arctan2(sinr_cosp, cosr_cosp))
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.degrees(np.copysign(np.pi / 2, sinp))
    else:
        pitch = np.degrees(np.arcsin(sinp))
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.degrees(np.arctan2(siny_cosp, cosy_cosp))
    
    return roll, pitch, yaw


class CrutchDevice:
    """Represents a single crutch device"""
    
    def __init__(self, name: str, port: str, has_analog: bool = True):
        self.name = name
        self.port = port
        self.has_analog = has_analog  # Whether this crutch has joystick
        self.serial: Optional[serial.Serial] = None
        self.mc_received = False
        self.mc_timestamp: Optional[datetime] = None
        self.data_log: List[Dict] = []
        self.connected = False
        self.thread: Optional[Thread] = None
        self.stop_event = Event()
        self.buffer = ""
        
        # Plotting buffers
        self.plot_time = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_analog = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_force = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_qw = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_qx = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_qy = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_qz = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_roll = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_pitch = deque(maxlen=PLOT_BUFFER_SIZE)
        self.plot_yaw = deque(maxlen=PLOT_BUFFER_SIZE)
        
    def __str__(self):
        joystick_status = "with joystick" if self.has_analog else "NO joystick"
        return f"{self.name} ({self.port}) [{joystick_status}]"


class RealTimePlotter:
    """Handles real-time plotting of crutch data"""
    
    def __init__(self, crutch1: CrutchDevice, crutch2: CrutchDevice):
        self.crutch1 = crutch1
        self.crutch2 = crutch2
        self.start_time = None
        
        # Create figure with 4 vertical subplots
        self.fig, self.axes = plt.subplots(4, 1, figsize=(12, 10))
        self.fig.suptitle('Dual Crutch HMI System - Real-Time Data', fontsize=14, fontweight='bold')
        
        # Plot 1: Analog values (joystick) - Only for crutch with joystick
        self.ax_analog = self.axes[0]
        if self.crutch1.has_analog and self.crutch2.has_analog:
            title = 'Analog Input (Single-Axis Joystick) - Both Crutches'
        elif self.crutch1.has_analog:
            title = f'Analog Input (Single-Axis Joystick) - {CRUTCH_1_NAME} Only'
        elif self.crutch2.has_analog:
            title = f'Analog Input (Single-Axis Joystick) - {CRUTCH_2_NAME} Only'
        else:
            title = 'Analog Input (No Joysticks Available)'
        
        self.ax_analog.set_title(title)
        self.ax_analog.set_ylabel('Value')
        self.ax_analog.set_ylim(0, 4096)
        self.ax_analog.grid(True, alpha=0.3)
        
        # Only create lines for crutches that have analog input
        if self.crutch1.has_analog:
            self.line_analog_1, = self.ax_analog.plot([], [], 'b-', label=f'{CRUTCH_1_NAME} (Right)', linewidth=2)
        else:
            self.line_analog_1 = None
            
        if self.crutch2.has_analog:
            self.line_analog_2, = self.ax_analog.plot([], [], 'r-', label=f'{CRUTCH_2_NAME} (Left)', linewidth=2)
        else:
            self.line_analog_2 = None
        
        # Add threshold lines
        self.ax_analog.axhline(y=819, color='g', linestyle='--', alpha=0.5, label='W threshold')
        self.ax_analog.axhline(y=3276, color='orange', linestyle='--', alpha=0.5, label='ES threshold')
        self.ax_analog.axhline(y=2048, color='gray', linestyle=':', alpha=0.3, label='Center')
        
        # Add note if one crutch doesn't have joystick
        if not (self.crutch1.has_analog and self.crutch2.has_analog):
            note = f"Note: {CRUTCH_2_NAME} (left) has NO joystick" if not self.crutch2.has_analog else f"Note: {CRUTCH_1_NAME} (right) has NO joystick"
            self.ax_analog.text(0.02, 0.98, note, transform=self.ax_analog.transAxes,
                              fontsize=9, verticalalignment='top',
                              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        self.ax_analog.legend(loc='upper right', fontsize=8)
        
        # Plot 2: Force measurements
        self.ax_force = self.axes[1]
        self.ax_force.set_title('Force Measurement')
        self.ax_force.set_ylabel('Force (g)')
        self.ax_force.grid(True, alpha=0.3)
        self.line_force_1, = self.ax_force.plot([], [], 'b-', label=CRUTCH_1_NAME, linewidth=2)
        self.line_force_2, = self.ax_force.plot([], [], 'r-', label=CRUTCH_2_NAME, linewidth=2)
        self.ax_force.legend(loc='upper right', fontsize=8)
        
        # Plot 3: Quaternions (Crutch 1 only for clarity)
        self.ax_quat = self.axes[2]
        self.ax_quat.set_title('Orientation - Quaternions (Crutch 1)')
        self.ax_quat.set_ylabel('Value')
        self.ax_quat.set_ylim(-1.1, 1.1)
        self.ax_quat.grid(True, alpha=0.3)
        self.line_qw, = self.ax_quat.plot([], [], 'b-', label='W', linewidth=1.5)
        self.line_qx, = self.ax_quat.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_qy, = self.ax_quat.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_qz, = self.ax_quat.plot([], [], 'm-', label='Z', linewidth=1.5)
        self.ax_quat.legend(loc='upper right', fontsize=8)
        
        # Plot 4: Euler angles (both crutches)
        self.ax_euler = self.axes[3]
        self.ax_euler.set_title('Orientation - Euler Angles')
        self.ax_euler.set_xlabel('Time (s)')
        self.ax_euler.set_ylabel('Angle (deg)')
        self.ax_euler.grid(True, alpha=0.3)
        self.line_roll_1, = self.ax_euler.plot([], [], 'b-', label='Roll C1', linewidth=2)
        self.line_pitch_1, = self.ax_euler.plot([], [], 'r-', label='Pitch C1', linewidth=2)
        self.line_yaw_1, = self.ax_euler.plot([], [], 'g-', label='Yaw C1', linewidth=2)
        self.line_roll_2, = self.ax_euler.plot([], [], 'b--', label='Roll C2', linewidth=1.5, alpha=0.7)
        self.line_pitch_2, = self.ax_euler.plot([], [], 'r--', label='Pitch C2', linewidth=1.5, alpha=0.7)
        self.line_yaw_2, = self.ax_euler.plot([], [], 'g--', label='Yaw C2', linewidth=1.5, alpha=0.7)
        self.ax_euler.legend(loc='upper right', ncol=2, fontsize=8)
        
        plt.tight_layout()
    
    def update_plot(self, frame):
        """Update all plots (called by FuncAnimation)"""
        # Get data from both crutches
        c1_time = list(self.crutch1.plot_time)
        c2_time = list(self.crutch2.plot_time)
        
        # Update analog plot - only for crutches with joystick
        if self.line_analog_1 is not None and len(c1_time) > 0:
            self.line_analog_1.set_data(c1_time, list(self.crutch1.plot_analog))
        if self.line_analog_2 is not None and len(c2_time) > 0:
            self.line_analog_2.set_data(c2_time, list(self.crutch2.plot_analog))
        
        # Update force plot
        if len(c1_time) > 0:
            self.line_force_1.set_data(c1_time, list(self.crutch1.plot_force))
        if len(c2_time) > 0:
            self.line_force_2.set_data(c2_time, list(self.crutch2.plot_force))
        
        # Update quaternion plot (Crutch 1 only)
        if len(c1_time) > 0:
            self.line_qw.set_data(c1_time, list(self.crutch1.plot_qw))
            self.line_qx.set_data(c1_time, list(self.crutch1.plot_qx))
            self.line_qy.set_data(c1_time, list(self.crutch1.plot_qy))
            self.line_qz.set_data(c1_time, list(self.crutch1.plot_qz))
        
        # Update euler plot (both crutches)
        if len(c1_time) > 0:
            self.line_roll_1.set_data(c1_time, list(self.crutch1.plot_roll))
            self.line_pitch_1.set_data(c1_time, list(self.crutch1.plot_pitch))
            self.line_yaw_1.set_data(c1_time, list(self.crutch1.plot_yaw))
        if len(c2_time) > 0:
            self.line_roll_2.set_data(c2_time, list(self.crutch2.plot_roll))
            self.line_pitch_2.set_data(c2_time, list(self.crutch2.plot_pitch))
            self.line_yaw_2.set_data(c2_time, list(self.crutch2.plot_yaw))
        
        # Auto-scale x-axis for all plots
        all_times = c1_time + c2_time
        if len(all_times) > 0:
            xmin = min(all_times)
            xmax = max(all_times)
            margin = (xmax - xmin) * 0.05 if xmax > xmin else 1
            
            for ax in self.axes:
                ax.set_xlim(xmin - margin, xmax + margin)
        
        # Auto-scale y-axis for force
        if len(self.crutch1.plot_force) > 0 or len(self.crutch2.plot_force) > 0:
            all_forces = list(self.crutch1.plot_force) + list(self.crutch2.plot_force)
            if len(all_forces) > 0:
                fmin = min(all_forces)
                fmax = max(all_forces)
                margin = (fmax - fmin) * 0.1 if fmax > fmin else 10
                self.ax_force.set_ylim(fmin - margin, fmax + margin)
        
        # Auto-scale y-axis for euler angles
        if len(self.crutch1.plot_roll) > 0 or len(self.crutch2.plot_roll) > 0:
            all_angles = (list(self.crutch1.plot_roll) + list(self.crutch1.plot_pitch) + 
                         list(self.crutch1.plot_yaw) + list(self.crutch2.plot_roll) + 
                         list(self.crutch2.plot_pitch) + list(self.crutch2.plot_yaw))
            if len(all_angles) > 0:
                amin = min(all_angles)
                amax = max(all_angles)
                margin = (amax - amin) * 0.1 if amax > amin else 10
                self.ax_euler.set_ylim(amin - margin, amax + margin)
        
        # Return all line objects (including None for disabled analog lines)
        return_lines = [self.line_force_1, self.line_force_2,
                       self.line_qw, self.line_qx, self.line_qy, self.line_qz,
                       self.line_roll_1, self.line_pitch_1, self.line_yaw_1,
                       self.line_roll_2, self.line_pitch_2, self.line_yaw_2]
        
        if self.line_analog_1 is not None:
            return_lines.append(self.line_analog_1)
        if self.line_analog_2 is not None:
            return_lines.append(self.line_analog_2)
            
        return tuple(return_lines)
    
    def start(self):
        """Start the animation"""
        self.anim = FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=PLOT_UPDATE_INTERVAL, 
            blit=False,
            cache_frame_data=False
        )
        
        # Try to maximize window for better visibility
        try:
            manager = plt.get_current_fig_manager()
            # Try different methods depending on backend
            try:
                manager.window.state('zoomed')  # Windows
            except:
                try:
                    manager.window.showMaximized()  # Qt backend
                except:
                    try:
                        manager.frame.Maximize(True)  # wxPython
                    except:
                        pass  # If all fail, just use default size
        except:
            pass  # Ignore if maximize fails
        
        plt.show(block=False)
        plt.pause(0.1)  # Give window time to render
    
    def close(self):
        """Close the plot window"""
        plt.close(self.fig)


class DualCrutchLogger:
    """Main logging application"""
    
    def __init__(self):
        self.crutch_1: Optional[CrutchDevice] = None
        self.crutch_2: Optional[CrutchDevice] = None
        self.logging_started = False
        self.logging_stopped = False
        self.log_start_time: Optional[datetime] = None
        self.plotter: Optional[RealTimePlotter] = None
        
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
        
        # Check for ES command (Emergency Stop - also stops logging)
        if ",ES," in line and self.logging_started:
            print(f"\n🛑 ES (Emergency Stop) received from {device.name}")
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
                
                # Calculate Euler angles
                roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)
                
                # Extract analog value from command or use default
                # Analog value might be in the data stream as part of command
                # For now, we'll use a placeholder unless we parse it separately
                analog_value = 2048  # Default center value
                
                data = {
                    'device': device.name,
                    'timestamp': timestamp,
                    'system_time': datetime.now(),
                    'command': command,
                    'force': force,
                    'qw': qw,
                    'qx': qx,
                    'qy': qy,
                    'qz': qz,
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'analog': analog_value
                }
                
                # Update plotting buffers
                if self.log_start_time:
                    rel_time = (datetime.now() - self.log_start_time).total_seconds()
                    device.plot_time.append(rel_time)
                    device.plot_analog.append(analog_value)
                    device.plot_force.append(force)
                    device.plot_qw.append(qw)
                    device.plot_qx.append(qx)
                    device.plot_qy.append(qy)
                    device.plot_qz.append(qz)
                    device.plot_roll.append(roll)
                    device.plot_pitch.append(pitch)
                    device.plot_yaw.append(yaw)
                
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
                if int(elapsed) % 5 == 0 and int(elapsed * 10) % 10 == 0:
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
            print(f"   (Logging will stop when AN/ES command is received from either crutch)")
            print(f"   (Or close the plot window to stop)\n")
            
            return True
            
        except KeyboardInterrupt:
            print("\n\n⚠️  User cancelled waiting for MC commands")
            return False
    
    def monitor_logging(self):
        """Monitor logging until AN command or user interrupt"""
        last_status_time = time.time()
        
        # Start real-time plotter
        print("📊 Starting real-time plots...")
        self.plotter = RealTimePlotter(self.crutch_1, self.crutch_2)
        self.plotter.start()
        
        try:
            while not self.logging_stopped:
                time.sleep(0.1)
                
                # Check if plot window was closed
                if not plt.fignum_exists(self.plotter.fig.number):
                    print("\n📊 Plot window closed by user")
                    self.logging_stopped = True
                    break
                
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
        
        # Keep plot window open after logging stops
        if self.plotter and plt.fignum_exists(self.plotter.fig.number):
            print("\n📊 Logging complete - Plot window will remain open")
            print("   Close the plot window when you're done viewing")
            
            # Stop animation but keep window open
            if hasattr(self.plotter, 'anim'):
                self.plotter.anim.event_source.stop()
            
            # Block here until user closes the window
            try:
                plt.show(block=True)
            except:
                pass
    
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
                                  'force', 'qw', 'qx', 'qy', 'qz', 
                                  'roll', 'pitch', 'yaw', 'analog']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    
                    writer.writeheader()
                    for data in device.data_log:
                        writer.writerow({
                            'system_time': data['system_time'].strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                            'device_timestamp': data['timestamp'],
                            'device': data['device'],
                            'command': data['command'],
                            'force': f"{data['force']:.2f}",
                            'qw': f"{data['qw']:.4f}",
                            'qx': f"{data['qx']:.4f}",
                            'qy': f"{data['qy']:.4f}",
                            'qz': f"{data['qz']:.4f}",
                            'roll': f"{data['roll']:.2f}",
                            'pitch': f"{data['pitch']:.2f}",
                            'yaw': f"{data['yaw']:.2f}",
                            'analog': data['analog']
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
                              'force', 'qw', 'qx', 'qy', 'qz',
                              'roll', 'pitch', 'yaw', 'analog']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                writer.writeheader()
                for data in all_data:
                    writer.writerow({
                        'system_time': data['system_time'].strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                        'device_timestamp': data['timestamp'],
                        'device': data['device'],
                        'command': data['command'],
                        'force': f"{data['force']:.2f}",
                        'qw': f"{data['qw']:.4f}",
                        'qx': f"{data['qx']:.4f}",
                        'qy': f"{data['qy']:.4f}",
                        'qz': f"{data['qz']:.4f}",
                        'roll': f"{data['roll']:.2f}",
                        'pitch': f"{data['pitch']:.2f}",
                        'yaw': f"{data['yaw']:.2f}",
                        'analog': data['analog']
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
        
        # Only close plot if it still exists (user hasn't closed it yet)
        if self.plotter and plt.fignum_exists(self.plotter.fig.number):
            print("📊 Closing plot window...")
            plt.close(self.plotter.fig)
    
    def run(self):
        """Main execution flow"""
        print("=" * 70)
        print("  DUAL CRUTCH HMI DATA LOGGER (with Real-Time Plotting)")
        print("=" * 70)
        
        try:
            # Show available COM ports
            self.list_com_ports()
            
            # Step 1: Create device objects
            self.crutch_1 = CrutchDevice(CRUTCH_1_NAME, COM_PORT_CRUTCH_1, CRUTCH_1_HAS_ANALOG)
            self.crutch_2 = CrutchDevice(CRUTCH_2_NAME, COM_PORT_CRUTCH_2, CRUTCH_2_HAS_ANALOG)
            
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
    print("\nStarting Dual Crutch Logger with Real-Time Plotting...")
    print("Make sure both crutches are powered on and paired!\n")
    
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n👋 Goodbye!")