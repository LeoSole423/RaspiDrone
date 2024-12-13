from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

# Conectar al Pixhawk usando el puerto UART /dev/ttyAMA0
print("Conectando al vehículo...")
vehicle = connect('/dev/ttyAMA0', baud=57600, wait_ready=True)

# Función para despegar
def despegar(altura_objetivo):
    print(f"Despegando a {altura_objetivo} metros...")
    vehicle.simple_takeoff(altura_objetivo)

    # Esperar hasta alcanzar la altitud objetivo
    while True:
        print(f"Altitud actual: {vehicle.location.global_relative_frame.alt:.2f} m")
        if vehicle.location.global_relative_frame.alt >= altura_objetivo * 0.95:  # Alcanzar el 95% de la altitud objetivo
            print("Altitud alcanzada.")
            break
        time.sleep(1)

# Asegurarse de que el dron esté en modo GUIDED
print("Cambiando a modo GUIDED...")
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name == "GUIDED":
    print("Esperando cambio a modo GUIDED...")
    time.sleep(1)

# Armar los motores
print("Armando motores...")
vehicle.armed = True
while not vehicle.armed:
    print("Esperando a que los motores se armen...")
    time.sleep(1)

print("Motores armados. Listo para despegar.")

# Realizar despegue
despegar(5)  # Despegar a 5 metros

# Mantener la altitud durante 5 segundos
print("Manteniendo altitud...")
time.sleep(5)

# Iniciar aterrizaje
print("Iniciando aterrizaje...")
vehicle.mode = VehicleMode("LAND")
while not vehicle.mode.name == "LAND":
    print("Esperando cambio a modo LAND...")
    time.sleep(1)

# Esperar hasta que el dron aterrice
while vehicle.armed:
    print("Aterrizando...")
    time.sleep(1)

print("Dron aterrizado y motores desarmados.")

# Cerrar la conexión
vehicle.close()
print("Conexión terminada.")
