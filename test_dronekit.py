from dronekit import connect, VehicleMode
import time

# Conectar al Pixhawk usando el puerto UART /dev/ttyAMA0
print("Conectando al vehículo...")
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

# Función para mostrar telemetría básica
def mostrar_telemetria():
    print("\n----- Telemetría Básica -----")
    print(f"Modo: {vehicle.mode.name}")
    print(f"Estado de armado: {vehicle.armed}")
    print(f"Altitud: {vehicle.location.global_relative_frame.alt} m")
    print(f"Batería: {vehicle.battery}")
    print(f"Velocidad: {vehicle.airspeed} m/s")
    print("-----------------------------\n")

# Cambiar a modo GUIDED
print("Cambiando a modo Stabilize...")
vehicle.mode = VehicleMode("STABILIZE")
while not vehicle.mode.name == "STABILIZE":
    print("Esperando cambio a modo STABILIZE...")
    time.sleep(1)

# Armar los motores
print("Armando motores...")
vehicle.armed = True
while not vehicle.armed:
    print("Esperando a que los motores se armen...")
    time.sleep(1)

print("Motores armados. Dron listo para operación.")

# Mostrar telemetría básica
mostrar_telemetria()

# Mantener el script corriendo para mostrar telemetría periódicamente
try:
    while True:
        mostrar_telemetria()
        time.sleep(5)  # Actualiza cada 5 segundos
except KeyboardInterrupt:
    print("Finalizando script...")

# Desarmar los motores antes de salir
print("Desarmando motores...")
vehicle.armed = False
while vehicle.armed:
    print("Esperando a que los motores se desarmen...")
    time.sleep(1)

print("Motores desarmados. Conexión terminada.")
vehicle.close()
