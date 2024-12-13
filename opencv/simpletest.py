import cv2
import cv2.aruco as aruco

# Configura el diccionario y los parámetros de detector
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
detector_params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, detector_params)

# Inicia la cámara
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convierte la imagen a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detecta los marcadores en la imagen
    corners, ids, rejected = detector.detectMarkers(gray)

    # Verifica si se detectó el marcador específico (ID 72)
    if ids is not None:
        for i, id_marker in enumerate(ids):
            if id_marker[0] == 72:
                # Dibuja el marcador detectado con ID 72
                aruco.drawDetectedMarkers(frame, [corners[i]], [id_marker])

    # Muestra el resultado en una ventana
    cv2.imshow('Aruco Marker Detection', frame)

    # Salir con la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera la cámara y cierra las ventanas
cap.release()
cv2.destroyAllWindows()
