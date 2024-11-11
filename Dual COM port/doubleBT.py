import serial
import threading

# Configuración del puerto serie
port = 'COM7'
baudrate = 115200  # Ajusta el baudrate según tus necesidades

# Conexión al puerto serie
ser = serial.Serial(port, baudrate, timeout=1)

def read_from_port(ser):
  """Función para leer datos del puerto serie y mostrarlos en consola."""
  while True:
      if ser.in_waiting > 0:
          response = ser.readline().decode('utf-8').strip()
          print(f"->R:{response}")

# Hilo para leer del puerto
thread = threading.Thread(target=read_from_port, args=(ser,))

# Iniciar el hilo
thread.start()

try:
  while True:
      # Leer entrada del usuario
      user_input = input("Ingrese el comando (R:{texto}): ").strip()
      
      if user_input.startswith('R:'):
          message = user_input[2:]
          ser.write(message.encode('utf-8'))
      else:
          print("Comando no reconocido. Use R:{texto}.")
except KeyboardInterrupt:
  print("Cerrando el programa...")
finally:
  # Cerrar el puerto serie
  ser.close()