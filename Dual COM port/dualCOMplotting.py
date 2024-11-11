import serial
import threading
import time
import re
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import numpy as np
from collections import deque

class SerialPortPlotter:
  def __init__(self, max_points=500):
      # Initialize COM ports
      try:
          self.com_left = serial.Serial(
              port='COM3',
              baudrate=9600,
              parity=serial.PARITY_NONE,
              stopbits=serial.STOPBITS_ONE,
              bytesize=serial.EIGHTBITS,
              timeout=0.1
          )
          
          self.com_right = serial.Serial(
              port='COM5',
              baudrate=9600,
              parity=serial.PARITY_NONE,
              stopbits=serial.STOPBITS_ONE,
              bytesize=serial.EIGHTBITS,
              timeout=0.1
          )
      except serial.SerialException as e:
          print(f"Error opening COM ports: {e}")
          raise

      # Create events for thread control
      self.running = True
      
      # Initialize data storage with empty lists
      self.max_points = max_points
      self.left_data = []
      self.right_data = []
      self.timestamps_left = []
      self.timestamps_right = []
      self.start_time = time.time()

      # Initialize plot
      self.setup_plot()

  def setup_plot(self):
      """Initialize the plotting window"""
      self.app = QApplication([])
      
      # Create window
      self.win = pg.GraphicsLayoutWidget(show=True, title="Real-time COM Port Data")
      self.win.resize(1000, 600)
      
      # Create plot item
      self.plot = self.win.addPlot(title="Signal vs Time")
      self.plot.setLabel('left', "Signal Value")
      self.plot.setLabel('bottom', "Time", units='s')
      self.plot.addLegend()
      self.plot.showGrid(x=True, y=True)
      
      # Create curve items with specific styles for continuous lines
      pen_left = pg.mkPen(color='b', width=2)
      pen_right = pg.mkPen(color='r', width=2)
      self.left_curve = self.plot.plot(pen=pen_left, name='Left (COM3)')
      self.right_curve = self.plot.plot(pen=pen_right, name='Right (COM5)')
      
      # Set up the timer for updates
      self.timer = QTimer()
      self.timer.timeout.connect(self.update_plot)
      self.timer.start(50)  # Update every 50ms

  def update_plot(self):
      """Update the plot with new data"""
      # Update left data
      if len(self.timestamps_left) > self.max_points:
          self.timestamps_left = self.timestamps_left[-self.max_points:]
          self.left_data = self.left_data[-self.max_points:]
      
      # Update right data
      if len(self.timestamps_right) > self.max_points:
          self.timestamps_right = self.timestamps_right[-self.max_points:]
          self.right_data = self.right_data[-self.max_points:]

      # Update the curves if we have data
      if self.left_data:
          self.left_curve.setData(self.timestamps_left, self.left_data)
      if self.right_data:
          self.right_curve.setData(self.timestamps_right, self.right_data)

      # Auto-range if we have data
      if self.left_data or self.right_data:
          self.plot.enableAutoRange('xy', True)

  def read_com_port(self, com_port, identifier):
      """Read data from specified COM port and store it"""
      while self.running:
          if com_port.in_waiting:
              try:
                  received_data = com_port.readline().decode().strip()
                  if received_data:
                      # Extract numerical value using regex
                      match = re.search(r'-?\d+\.?\d*', received_data)
                      if match:
                          value = float(match.group())
                          current_time = time.time() - self.start_time
                          
                          print(f"->{identifier}:{value}")
                          
                          # Store data based on identifier
                          if identifier == 'L':
                              self.left_data.append(value)
                              self.timestamps_left.append(current_time)
                          else:
                              self.right_data.append(value)
                              self.timestamps_right.append(current_time)
                              
              except Exception as e:
                  print(f"Error reading from {identifier}: {e}")
          time.sleep(0.01)

  def process_user_input(self):
      """Process user input and send to appropriate COM port"""
      while self.running:
          try:
              user_input = input()
              
              # Check if input matches the expected format
              left_match = re.match(r'^L:(.*)', user_input)
              right_match = re.match(r'^R:(.*)', user_input)
              
              if left_match:
                  message = left_match.group(1)
                  self.com_left.write(f"{message}\n".encode())
              elif right_match:
                  message = right_match.group(1)
                  self.com_right.write(f"{message}\n".encode())
              else:
                  print("Invalid format. Use L:message or R:message")
                  
          except Exception as e:
              print(f"Error processing input: {e}")

  def start(self):
      """Start all communication threads"""
      # Create threads
      left_thread = threading.Thread(target=self.read_com_port, args=(self.com_left, 'L'))
      right_thread = threading.Thread(target=self.read_com_port, args=(self.com_right, 'R'))
      input_thread = threading.Thread(target=self.process_user_input)
      
      # Set threads as daemon
      left_thread.daemon = True
      right_thread.daemon = True
      input_thread.daemon = True
      
      # Start all threads
      left_thread.start()
      right_thread.start()
      input_thread.start()
      
      print("COM port communication started. Enter messages in format L:message or R:message")
      print("Press Ctrl+C to exit")
      
      # Start Qt event loop
      self.app.exec_()
      self.stop()

  def stop(self):
      """Stop all communications and close ports"""
      self.running = False
      time.sleep(0.5)  # Allow threads to finish
      
      if hasattr(self, 'com_left'):
          self.com_left.close()
      if hasattr(self, 'com_right'):
          self.com_right.close()
      
      print("\nCOM ports closed")

if __name__ == "__main__":
  try:
      plotter = SerialPortPlotter()
      plotter.start()
  except Exception as e:
      print(f"Error: {e}")