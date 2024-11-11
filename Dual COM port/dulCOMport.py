import serial
import threading
import time
import re

class SerialPortManager:
  def __init__(self):
      # Initialize COM ports
      try:
          self.com_left = serial.Serial(
              port='COM3',
              baudrate=115200,
              parity=serial.PARITY_NONE,
              stopbits=serial.STOPBITS_ONE,
              bytesize=serial.EIGHTBITS,
              timeout=0.1
          )
          
          self.com_right = serial.Serial(
              port='COM5',
              baudrate=115200,
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
      
  def read_com_port(self, com_port, identifier):
      """Read data from specified COM port and display with identifier"""
      while self.running:
          if com_port.in_waiting:
              try:
                  received_data = com_port.readline().decode().strip()
                  if received_data:
                      print(f"->{identifier}:{received_data}")
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
      # Create threads for reading from each COM port
      left_thread = threading.Thread(target=self.read_com_port, args=(self.com_left, 'L'))
      right_thread = threading.Thread(target=self.read_com_port, args=(self.com_right, 'R'))
      input_thread = threading.Thread(target=self.process_user_input)
      
      # Set threads as daemon so they'll end when the main program ends
      left_thread.daemon = True
      right_thread.daemon = True
      input_thread.daemon = True
      
      # Start all threads
      left_thread.start()
      right_thread.start()
      input_thread.start()
      
      print("COM port communication started. Enter messages in format L:message or R:message")
      print("Press Ctrl+C to exit")
      
      try:
          while True:
              time.sleep(0.1)
      except KeyboardInterrupt:
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
      manager = SerialPortManager()
      manager.start()
  except Exception as e:
      print(f"Error: {e}")


