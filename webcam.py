import cv2

# Abre la webcam (por defecto la cámara 0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("No se pudo acceder a la webcam.")
    exit()

while True:
    # Captura cada cuadro de la webcam
    ret, frame = cap.read()

    # Verifica si la captura fue exitosa
    if not ret:
        print("No se pudo capturar el cuadro.")
        break

    # Muestra el cuadro en una ventana llamada 'Webcam'
    cv2.imshow('Webcam', frame)

    # Presiona 'q' para salir del bucle
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera el objeto de la cámara y cierra las ventanas
cap.release()
cv2.destroyAllWindows()

