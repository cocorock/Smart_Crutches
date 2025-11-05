"""
Dual Crutch Data Logger - Windows 11
Logs data from two ESP32-based instrumented crutches via Bluetooth Serial
Author: Created for Crutch HMI System
Date: 2025
"""

import serial
import threading
import time
import csv
import os
from datetime import datetime
from collections import deque
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

# Configuration
COM_PORTS = {
    'crutch1': 'COM10',
    'crutch2': 'COM12'
}
BAUD_RATE = 921600  # Match ESP32 configuration
TIMEOUT = 0.1  # Short timeout for efficiency
LOG_FOLDER = "LOGS"

# Data structure for each crutch
class CrutchData:
    def __init__(self, name):
        self.name = name
        self.timestamp = []
        self.force = []
        self.qw = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.analog = []
        self.commands = []
        self.lock = threading.Lock()
        self.mc_received = False
        self.an_received = False
        self.has_analog = False  # Will be determined from first data
        self.first_timestamp = 0  # For time alignment
        
    def add_data(self, data_dict):
        """Thread-safe data addition with force filtering"""
        with self.lock:
            self.timestamp.append(data_dict.get('timestamp', 0))
            
            # Filter force values to remove anomalies
            force_val = data_dict.get('force', 0.0)
            if force_val > 40000 or force_val < -2000:
                # Replace anomalous values with the last valid value or 0
                force_val = self.force[-1] if len(self.force) > 0 else 0.0
            self.force.append(force_val)
            
            self.qw.append(data_dict.get('qw', 0.0))
            self.qx.append(data_dict.get('qx', 0.0))
            self.qy.append(data_dict.get('qy', 0.0))
            self.qz.append(data_dict.get('qz', 0.0))
            
            # Handle analog value
            analog_val = data_dict.get('analog', None)
            if analog_val is not None and analog_val != 0:
                self.has_analog = True
            self.analog.append(analog_val if analog_val is not None else 0)
            
            self.commands.append(data_dict.get('command', ''))
    
    def get_data_copy(self):
        """Thread-safe data retrieval"""
        with self.lock:
            return {
                'timestamp': self.timestamp.copy(),
                'force': self.force.copy(),
                'qw': self.qw.copy(),
                'qx': self.qx.copy(),
                'qy': self.qy.copy(),
                'qz': self.qz.copy(),
                'analog': self.analog.copy(),
                'commands': self.commands.copy()
            }

# Global variables
crutch_data = {
    'crutch1': CrutchData('Crutch 1 (COM10)'),
    'crutch2': CrutchData('Crutch 2 (COM12)')
}
stop_logging = threading.Event()
start_logging = threading.Event()

def parse_message(line):
    """
    Parse incoming message from ESP32
    Actual format from Arduino: timestamp,command,force*100,qw*1000,qx*1000,qy*1000,qz*1000
    Examples:
    - Normal data: 50234,S,15634,987,23,-145,56
    - Motor Calibration: 12345,MC,AN:2048
    - Analog/Emergency: 50123,ES,AN:3500 or 50123,AN,AN:3500
    - Performance: PERF: IMU:8 Force:1 ... (ignored)
    """
    try:
        # Ignore performance monitoring and filter messages
        if line.startswith('PERF:') or line.startswith('FILTER:') or line.startswith('Calibration') or line.startswith('Test reading'):
            return None
            
        parts = line.strip().split(',')
        if len(parts) < 2:
            return None
        
        # Try to parse timestamp (first part should be a number)
        try:
            data = {'timestamp': int(parts[0])}
        except ValueError:
            # Not a valid data message (could be debug output)
            return None
        
        # Second field is the command/state (S, W, ES, MC, AN, etc.)
        command_field = parts[1].strip()
        
        # Check for MC command
        if command_field == 'MC' or 'MC' in command_field:
            data['command'] = 'MC'
            # MC format: timestamp,MC,AN:value
            if len(parts) > 2 and parts[2].startswith('AN:'):
                data['analog'] = int(parts[2][3:])
            return data
        
        # Check for AN command (CRITICAL for stopping)
        if command_field == 'AN':
            data['command'] = 'AN'
            # AN format: timestamp,AN,AN:value
            if len(parts) > 2 and parts[2].startswith('AN:'):
                data['analog'] = int(parts[2][3:])
            return data
        
        # Check for ES command (Emergency Stop)
        if command_field == 'ES':
            data['command'] = 'AN'  # Treat ES as AN for stopping
            if len(parts) > 2 and parts[2].startswith('AN:'):
                data['analog'] = int(parts[2][3:])
            return data
        
        # Normal data message: timestamp,command,force*100,qw*1000,qx*1000,qy*1000,qz*1000
        data['command'] = command_field  # S, W, or other state
        
        if len(parts) >= 7:
            try:
                data['force'] = int(parts[2]) / 100.0  # Convert back from *100
                data['qw'] = int(parts[3]) / 1000.0     # Convert back from *1000
                data['qx'] = int(parts[4]) / 1000.0
                data['qy'] = int(parts[5]) / 1000.0
                data['qz'] = int(parts[6]) / 1000.0
            except ValueError:
                # Invalid numeric data
                return None
        
        # Check if there's an analog value (8th field)
        if len(parts) >= 8 and parts[7].startswith('AN:'):
            data['analog'] = int(parts[7][3:])
        
        return data
    except Exception as e:
        # Only print errors for actual data lines
        if not (line.startswith('PERF:') or line.startswith('FILTER:') or line.startswith('Test') or line.startswith('Calibration')):
            print(f"Parse error: {e} - Line: {line[:80]}")
        return None

def read_serial(port_name, crutch_key):
    """
    Thread function to read from serial port
    Optimized for high-speed data acquisition
    """
    crutch = crutch_data[crutch_key]
    buffer = ""
    
    try:
        # Open serial port with optimal settings for ESP32
        ser = serial.Serial(
            port=port_name,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT,
            # ESP32-specific optimizations
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )
        
        # Flush any old data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print(f"✓ Connected to {crutch.name} on {port_name}")
        
        # Wait for MC command before starting
        print(f"Waiting for MC command from {crutch.name}...")
        while not stop_logging.is_set() and not crutch.mc_received:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += chunk
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    data = parse_message(line)
                    
                    if data and data.get('command') == 'MC':
                        crutch.mc_received = True
                        print(f"✓ MC received from {crutch.name}")
                        break
            time.sleep(0.001)  # Minimal sleep for CPU efficiency
        
        # Wait for both crutches to receive MC
        while not stop_logging.is_set() and not (crutch_data['crutch1'].mc_received and crutch_data['crutch2'].mc_received):
            time.sleep(0.01)
        
        if stop_logging.is_set():
            return
        
        # Both received MC, wait 5 seconds
        if crutch_key == 'crutch1':  # Only print once
            print("\n✓ Both crutches ready. Waiting 5 seconds before logging...")
            time.sleep(10)
            start_logging.set()
            print("✓ Logging started!\n")
            print("=" * 60)
            print("FORCE MONITORING (every 20 messages)")
            print("=" * 60)
            print("DeviceID, MessageNumber, ForceValue")
            print("-" * 60)
        else:
            start_logging.wait()
        
        # Main logging loop
        message_count = 0
        buffer = ""  # Reset buffer
        current_force = 0.0  # Track last force value
        first_timestamp = None  # For time alignment
        
        while not stop_logging.is_set():
            if ser.in_waiting > 0:
                # Read available data efficiently
                chunk = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                buffer += chunk
                
                # Process complete lines
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    data = parse_message(line)
                    
                    if data:
                        # Check for AN command (CRITICAL - stop immediately!)
                        if data.get('command') == 'AN':
                            print(f"\n✓ AN (stop) received from {crutch.name}")
                            crutch.an_received = True
                            stop_logging.set()  # IMMEDIATE STOP - don't wait for other crutch
                            break
                        
                        # Add data if logging started
                        if start_logging.is_set():
                            # Capture first timestamp for alignment
                            if first_timestamp is None:
                                first_timestamp = data.get('timestamp', 0)
                                crutch.first_timestamp = first_timestamp
                                print(f"{crutch.name} first timestamp: {first_timestamp}ms")
                            
                            crutch.add_data(data)
                            message_count += 1
                            
                            # Update current force
                            current_force = data.get('force', 0.0)
                            
                            # Print force value every 20 messages
                            if message_count % 20 == 0:
                                print(f"{port_name}, {message_count}, {current_force:.2f}")
            else:
                time.sleep(0.001)  # Minimal sleep when no data
        
        print(f"\n{'='*60}")
        print(f"✓ {crutch.name} logging stopped")
        print(f"  Total messages: {message_count}")
        if message_count > 0:
            print(f"  Final force: {current_force:.2f}")
        print(f"{'='*60}")
        ser.close()
        
    except serial.SerialException as e:
        print(f"✗ Error connecting to {port_name}: {e}")
        stop_logging.set()
    except Exception as e:
        print(f"✗ Unexpected error on {port_name}: {e}")
        import traceback
        traceback.print_exc()
        stop_logging.set()

def save_csv_files():
    """Save logged data to CSV files"""
    # Create LOGS folder if it doesn't exist
    Path(LOG_FOLDER).mkdir(exist_ok=True)
    
    # Generate timestamp for filenames
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Get data copies
    data1 = crutch_data['crutch1'].get_data_copy()
    data2 = crutch_data['crutch2'].get_data_copy()
    
    # Determine which crutch has analog
    crutch1_has_analog = crutch_data['crutch1'].has_analog
    crutch2_has_analog = crutch_data['crutch2'].has_analog
    
    # Save individual crutch files
    files_saved = []
    
    # Crutch 1
    filename1 = os.path.join(LOG_FOLDER, f"crutch1_COM10_{timestamp_str}.csv")
    first_ts1 = crutch_data['crutch1'].first_timestamp
    with open(filename1, 'w', newline='') as f:
        writer = csv.writer(f)
        if crutch1_has_analog:
            writer.writerow(['Timestamp(ms)', 'AlignedTime(s)', 'Force', 'Qw', 'Qx', 'Qy', 'Qz', 'Analog', 'Command'])
            for i in range(len(data1['timestamp'])):
                aligned_time = (data1['timestamp'][i] - first_ts1) / 1000.0
                writer.writerow([
                    data1['timestamp'][i], aligned_time, data1['force'][i],
                    data1['qw'][i], data1['qx'][i], data1['qy'][i], data1['qz'][i],
                    data1['analog'][i], data1['commands'][i]
                ])
        else:
            writer.writerow(['Timestamp(ms)', 'AlignedTime(s)', 'Force', 'Qw', 'Qx', 'Qy', 'Qz', 'Command'])
            for i in range(len(data1['timestamp'])):
                aligned_time = (data1['timestamp'][i] - first_ts1) / 1000.0
                writer.writerow([
                    data1['timestamp'][i], aligned_time, data1['force'][i],
                    data1['qw'][i], data1['qx'][i], data1['qy'][i], data1['qz'][i],
                    data1['commands'][i]
                ])
    files_saved.append(filename1)
    print(f"✓ Saved: {filename1}")
    
    # Crutch 2
    filename2 = os.path.join(LOG_FOLDER, f"crutch2_COM12_{timestamp_str}.csv")
    first_ts2 = crutch_data['crutch2'].first_timestamp
    with open(filename2, 'w', newline='') as f:
        writer = csv.writer(f)
        if crutch2_has_analog:
            writer.writerow(['Timestamp(ms)', 'AlignedTime(s)', 'Force', 'Qw', 'Qx', 'Qy', 'Qz', 'Analog', 'Command'])
            for i in range(len(data2['timestamp'])):
                aligned_time = (data2['timestamp'][i] - first_ts2) / 1000.0
                writer.writerow([
                    data2['timestamp'][i], aligned_time, data2['force'][i],
                    data2['qw'][i], data2['qx'][i], data2['qy'][i], data2['qz'][i],
                    data2['analog'][i], data2['commands'][i]
                ])
        else:
            writer.writerow(['Timestamp(ms)', 'AlignedTime(s)', 'Force', 'Qw', 'Qx', 'Qy', 'Qz', 'Command'])
            for i in range(len(data2['timestamp'])):
                aligned_time = (data2['timestamp'][i] - first_ts2) / 1000.0
                writer.writerow([
                    data2['timestamp'][i], aligned_time, data2['force'][i],
                    data2['qw'][i], data2['qx'][i], data2['qy'][i], data2['qz'][i],
                    data2['commands'][i]
                ])
    files_saved.append(filename2)
    print(f"✓ Saved: {filename2}")
    
    # Combined file
    filename_combined = os.path.join(LOG_FOLDER, f"combined_both_crutches_{timestamp_str}.csv")
    with open(filename_combined, 'w', newline='') as f:
        writer = csv.writer(f)
        # Write header with both crutches including aligned time
        header = [
            'AlignedTime(s)', 'Timestamp1(ms)', 'Crutch1_Force', 'Crutch1_Qw', 'Crutch1_Qx', 'Crutch1_Qy', 'Crutch1_Qz',
            'Timestamp2(ms)', 'Crutch2_Force', 'Crutch2_Qw', 'Crutch2_Qx', 'Crutch2_Qy', 'Crutch2_Qz'
        ]
        if crutch1_has_analog:
            header.insert(7, 'Crutch1_Analog')
        if crutch2_has_analog:
            header.append('Crutch2_Analog')
        writer.writerow(header)
        
        # Write data (align by row index)
        max_len = max(len(data1['timestamp']), len(data2['timestamp']))
        for i in range(max_len):
            row = []
            
            # Calculate aligned time (use average of both for combined file)
            aligned_time = i * 0.05  # Assuming ~20Hz, approximate aligned time
            if i < len(data1['timestamp']) and i < len(data2['timestamp']):
                t1_aligned = (data1['timestamp'][i] - first_ts1) / 1000.0
                t2_aligned = (data2['timestamp'][i] - first_ts2) / 1000.0
                aligned_time = (t1_aligned + t2_aligned) / 2.0
            elif i < len(data1['timestamp']):
                aligned_time = (data1['timestamp'][i] - first_ts1) / 1000.0
            elif i < len(data2['timestamp']):
                aligned_time = (data2['timestamp'][i] - first_ts2) / 1000.0
            
            row.append(aligned_time)
            
            # Crutch 1 data
            if i < len(data1['timestamp']):
                row.extend([
                    data1['timestamp'][i], data1['force'][i],
                    data1['qw'][i], data1['qx'][i], data1['qy'][i], data1['qz'][i]
                ])
                if crutch1_has_analog:
                    row.append(data1['analog'][i])
            else:
                row.extend(['', '', '', '', '', ''])
                if crutch1_has_analog:
                    row.append('')
            
            # Crutch 2 data
            if i < len(data2['timestamp']):
                row.extend([
                    data2['timestamp'][i], data2['force'][i],
                    data2['qw'][i], data2['qx'][i], data2['qy'][i], data2['qz'][i]
                ])
                if crutch2_has_analog:
                    row.append(data2['analog'][i])
            else:
                row.extend(['', '', '', '', '', ''])
                if crutch2_has_analog:
                    row.append('')
            
            writer.writerow(row)
    
    files_saved.append(filename_combined)
    print(f"✓ Saved: {filename_combined}")
    
    return files_saved

def plot_data():
    """Generate plots for both crutches with time alignment"""
    print("\nGenerating plots...")
    
    data1 = crutch_data['crutch1'].get_data_copy()
    data2 = crutch_data['crutch2'].get_data_copy()
    
    # Get first timestamps for alignment
    first_ts1 = crutch_data['crutch1'].first_timestamp
    first_ts2 = crutch_data['crutch2'].first_timestamp
    
    print(f"\nTime Alignment:")
    print(f"  Crutch 1 first timestamp: {first_ts1}ms")
    print(f"  Crutch 2 first timestamp: {first_ts2}ms")
    print(f"  Time offset: {abs(first_ts1 - first_ts2)}ms")
    
    # Convert timestamps to seconds and align to same starting point (0)
    if len(data1['timestamp']) > 0:
        t1 = (np.array(data1['timestamp']) - first_ts1) / 1000.0
    else:
        t1 = np.array([])
    
    if len(data2['timestamp']) > 0:
        t2 = (np.array(data2['timestamp']) - first_ts2) / 1000.0
    else:
        t2 = np.array([])
    
    # Create figure with 2 subplots (removed analog plot)
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Dual Crutch Data Analysis (Time Aligned)', fontsize=16, fontweight='bold')
    
    # Plot 1: Forces
    ax1 = axes[0]
    if len(t1) > 0:
        ax1.plot(t1, data1['force'], 'b-', label='Crutch 1 (COM10)', linewidth=1.5)
    if len(t2) > 0:
        ax1.plot(t2, data2['force'], 'r-', label='Crutch 2 (COM12)', linewidth=1.5)
    ax1.set_xlabel('Time (s) - Aligned at logging start')
    ax1.set_ylabel('Force (units)')
    ax1.set_title('Force Measurements (Filtered: -2000 to 40000)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Quaternions (all 4 components for both crutches)
    ax2 = axes[1]
    if len(t1) > 0:
        ax2.plot(t1, data1['qw'], 'b-', label='C1 Qw', alpha=0.7)
        ax2.plot(t1, data1['qx'], 'b--', label='C1 Qx', alpha=0.7)
        ax2.plot(t1, data1['qy'], 'b:', label='C1 Qy', alpha=0.7)
        ax2.plot(t1, data1['qz'], 'b-.', label='C1 Qz', alpha=0.7)
    if len(t2) > 0:
        ax2.plot(t2, data2['qw'], 'r-', label='C2 Qw', alpha=0.7)
        ax2.plot(t2, data2['qx'], 'r--', label='C2 Qx', alpha=0.7)
        ax2.plot(t2, data2['qy'], 'r:', label='C2 Qy', alpha=0.7)
        ax2.plot(t2, data2['qz'], 'r-.', label='C2 Qz', alpha=0.7)
    ax2.set_xlabel('Time (s) - Aligned at logging start')
    ax2.set_ylabel('Quaternion Value')
    ax2.set_title('Quaternion Components (Orientation)')
    ax2.legend(ncol=2, fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    print("✓ Plots displayed")

def main():
    """Main execution function"""
    print("="*60)
    print("  DUAL CRUTCH DATA LOGGER - Windows 11")
    print("="*60)
    print(f"  Crutch 1: {COM_PORTS['crutch1']}")
    print(f"  Crutch 2: {COM_PORTS['crutch2']}")
    print(f"  Baud Rate: {BAUD_RATE}")
    print(f"  Log Folder: {LOG_FOLDER}")
    print("="*60)
    print("\nStarting connection to crutches...")
    
    # Create threads for both crutches
    thread1 = threading.Thread(target=read_serial, args=(COM_PORTS['crutch1'], 'crutch1'))
    thread2 = threading.Thread(target=read_serial, args=(COM_PORTS['crutch2'], 'crutch2'))
    
    # Start threads
    thread1.start()
    thread2.start()
    
    # Wait for threads to complete
    thread1.join()
    thread2.join()
    
    print("\n" + "="*60)
    print("  LOGGING COMPLETED")
    print("="*60)
    
    # Save data to CSV
    print("\nSaving data to CSV files...")
    saved_files = save_csv_files()
    
    # Display statistics
    print("\n" + "="*60)
    print("  STATISTICS")
    print("="*60)
    data1 = crutch_data['crutch1'].get_data_copy()
    data2 = crutch_data['crutch2'].get_data_copy()
    
    print(f"Crutch 1 (COM10):")
    print(f"  - Messages logged: {len(data1['timestamp'])}")
    print(f"  - Has analog input: {'Yes' if crutch_data['crutch1'].has_analog else 'No'}")
    if len(data1['timestamp']) > 0:
        duration1 = (data1['timestamp'][-1] - data1['timestamp'][0]) / 1000.0
        print(f"  - Duration: {duration1:.2f} seconds")
        print(f"  - Average rate: {len(data1['timestamp'])/duration1:.2f} Hz")
    
    print(f"\nCrutch 2 (COM12):")
    print(f"  - Messages logged: {len(data2['timestamp'])}")
    print(f"  - Has analog input: {'Yes' if crutch_data['crutch2'].has_analog else 'No'}")
    if len(data2['timestamp']) > 0:
        duration2 = (data2['timestamp'][-1] - data2['timestamp'][0]) / 1000.0
        print(f"  - Duration: {duration2:.2f} seconds")
        print(f"  - Average rate: {len(data2['timestamp'])/duration2:.2f} Hz")
    
    print("\n" + "="*60)
    print("  FILES SAVED")
    print("="*60)
    for f in saved_files:
        print(f"  - {f}")
    
    # Generate plots
    print("\n" + "="*60)
    plot_data()
    
    print("\n✓ All operations completed successfully!")
    print("="*60)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n✗ Interrupted by user")
        stop_logging.set()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()