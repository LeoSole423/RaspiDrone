from dronekit import connect, VehicleMode
import time

# Configura el puerto de conexión
connection_string = 'COM3'

# Conectar al dron
print(f"Conectando al vehículo en: {connection_string}...")
vehicle = connect(connection_string, baud=57600, wait_ready=False, timeout=120)

def check_system_status(vehicle):
    """
    Imprime el estado de los sistemas críticos del dron.
    """
    print("### Diagnóstico del sistema ###")
    print(f"- ¿Dron armable?: {'Sí' if vehicle.is_armable else 'No'}")
    print(f"- GPS: {vehicle.gps_0.fix_type} (Sats: {vehicle.gps_0.satellites_visible})")
    print(f"- Batería: {vehicle.battery.voltage:.2f} V, Nivel: {vehicle.battery.level}%")
    print(f"- Sistema armado: {'Sí' if vehicle.armed else 'No'}")
    print(f"- Estado EKF: {vehicle.ekf_ok}")
    print("-" * 30)

try:
    # Asegúrate de que los parámetros están cargados
    print("Esperando que los parámetros estén listos...")
    vehicle.parameters.wait_ready(timeout=120)

    # Desactiva los chequeos de armado para pruebas
    print("Desactivando chequeos de armado...")
    vehicle.parameters['ARMING_CHECK'] = 0
    time.sleep(2)

    # Cambia al modo STABILIZE
    print("Cambiando al modo STABILIZE para pruebas sin GPS...")
    vehicle.mode = VehicleMode("STABILIZE")
    while not vehicle.mode.name == "STABILIZE":
        print("Esperando que el dron cambie a modo STABILIZE...")
        time.sleep(1)

    # Verifica si el vehículo está armable
    print("Verificando conexión al vehículo...")
    while not vehicle.is_armable:
        print("El vehículo aún no está armable. Diagnóstico:")
        check_system_status(vehicle)
        time.sleep(5)

    # Arma los motores
    print("Armando motores...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Esperando que los motores se armen...")
        time.sleep(1)

    print("¡Motores armados y listos para pruebas!")

except KeyboardInterrupt:
    print("\nConexión finalizada por el usuario.")

finally:
    # Cierra la conexión al dron
    vehicle.close()
    print("Conexión cerrada.")
