import asyncio
from mavsdk import System

async def run():
    # Conéctate al servidor MAVLink en el puerto 50051
    drone = System()
    
    # Cambia la dirección si usas un puerto diferente
    await drone.connect(system_address="localhost:50051")

    # Verifica que la conexión fue exitosa
    print("Conectando al dron...")
    
    # Espera hasta que el sistema esté listo
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("¡Conectado al servidor MAVSDK!")
            break

    # Obtén información básica del sistema
    print("Recibiendo información del sistema:")
    async for health in drone.telemetry.health():
        print(f"GPS: {health.gps_info}")
        print(f"Batería: {health.battery}")
        print(f"EKF OK: {health.ekf_ok}")
        break

if __name__ == "__main__":
    asyncio.run(run())
