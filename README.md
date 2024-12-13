# Drones

Este repositorio contiene documentación y proyectos relacionados con el uso de drones y cámaras en sistemas embebidos, como la Raspberry Pi 4B.

---

## Instalacin del SO Raspberry Pi + Conexion con Pixhawk

Video de instalacion:
https://www.youtube.com/watch?v=kB9YyG2V-nA&t=477s

Aclaracion: Para la instalacion con el comando PIP fr algunas librerias fue necesario crear un Virtual Enviroment (VENV)(No aparece en el video).
Para que funcione la libreria de DroneKit, es necesario modificar una parte:
En la carpeta de la libreria donde se halla instalado DroneKit, buscar la carpeta _init_.py. En esta hay que arreglar la clase Parameters ( en los parámetros corregir collections.abc.MutableMapping)
Cambiar:
```bash
   class Parametes(collections.MutableMapping, HasObservers):
   ```
Por:
```bash
   class Parametes(collections.abc.MutableMapping, HasObservers):
   ```

## Instalación de ArduCam (OV5647) en Raspberry Pi 4B

Para configurar correctamente la cámara ArduCam OV5647 y usarla con OpenCV, es necesario ajustar el archivo de configuración del sistema.

### Pasos para configurar:

1. Abre una terminal en tu Raspberry Pi.
2. Edita el archivo `/boot/firmware/config.txt` utilizando el siguiente comando:
   ```bash
   nano /boot/firmware/config.txt
   ```
3. Asegúrate de que las siguientes líneas están presentes en el archivo de configuración. Si no están, agrégalas o modifícalas según corresponda:
   ```
   camera_auto_detect=0
   start_x=1
   gpu_mem=128
   dtoverlay=vc4-kms-v3d
   ```
4. Guarda los cambios y cierra el editor (`Ctrl+O`, luego `Enter` y `Ctrl+X` para salir de `nano`).
5. Reinicia la Raspberry Pi para que los cambios surtan efecto:
   ```bash
   sudo reboot
   ```

### Notas:
- **`camera_auto_detect=0`**: Desactiva la detección automática de cámaras.
- **`start_x=1`**: Habilita el soporte para la cámara.
- **`gpu_mem=128`**: Asigna 128 MB de memoria a la GPU para soportar el procesamiento de video.
- **`dtoverlay=vc4-kms-v3d`**: Habilita la aceleración de gráficos necesaria para OpenCV.

Con esta configuración, tu cámara debería funcionar correctamente con OpenCV en tu Raspberry Pi 4B.
