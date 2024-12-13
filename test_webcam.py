import cv2

# Abre la camara 0 (webcam por defecto)
cap = cv2.VideoCapture(0)

# Verifica si la camara se ha abierto correctamente
if not cap.isOpened():
    print("No se pudo acceder a la camara")
    exit()

# Establece la resolucion a 1280x720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True:
    # Lee un frame de la camara
    ret, frame = cap.read()

    if not ret:
        print("No se pudo leer el frame")
        break

    # Muestra el frame en una ventana llamada 'Webcam'
    cv2.imshow('Webcam', frame)

    # Toma una captura cuando se presiona la tecla 'c'
    if cv2.waitKey(1) & 0xFF == ord('c'):
        # Guarda la captura con un nombre unico basado en la hora actual, en formato JPG
        filename = "captura_{}.jpg".format(cv2.getTickCount())
        cv2.imwrite(filename, frame)
        print(f"Captura guardada como {filename}")

    # Sale del bucle cuando se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera la camara y cierra las ventanas al finalizar
cap.release()
cv2.destroyAllWindows()
