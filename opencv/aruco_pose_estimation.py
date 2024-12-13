import numpy as np
import cv2
import cv2.aruco as aruco
import math

#--- Define Tag
id_to_find = 20
marker_size = 10  # [cm]

#--- Load the camera calibration parameters
calib_path = "C:\\Users\\leone\\OneDrive\\Escritorio\\how_do_drones_work\\opencv\\"

camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

#--- Define the ArUco dictionary and detector parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

#--- Capture the videocamera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al capturar la imagen")
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find all the ArUco markers in the image
    corners, ids, rejected = aruco_detector.detectMarkers(gray)

    # Check if any marker was detected
    if ids is not None:
        print(f"Marcador detectado con ID(s): {ids.flatten()}")
        
        # If the marker we want is detected
        if id_to_find in ids:
            # Get the index of the marker with id `id_to_find`
            index = list(ids.flatten()).index(id_to_find)

            # Define los puntos 3D del marcador en su sistema de coordenadas
            marker_corners_3d = np.array([
                [-marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [-marker_size / 2, -marker_size / 2, 0]
            ], dtype=np.float32)

            # Usa solvePnP para estimar rvec y tvec
            ret, rvec, tvec = cv2.solvePnP(
                marker_corners_3d,
                corners[index].reshape(-1, 2),  # Convierte esquinas a 2D
                camera_matrix,
                camera_distortion
            )


            # Draw the detected marker and axis
            aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 0.10)

            # Print the tag position
            str_position = f"Posicion MARCADOR x={tvec[0][0]:.0f}  y={tvec[1][0]:.0f}  z={tvec[2][0]:.0f}"

            cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            print(str_position)

            # Obtain rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            # Calculate Euler angles
            def rotationMatrixToEulerAngles(R):
                sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
                singular = sy < 1e-6
                if not singular:
                    x = math.atan2(R[2, 1], R[2, 2])
                    y = math.atan2(-R[2, 0], sy)
                    z = math.atan2(R[1, 0], R[0, 0])
                else:
                    x = math.atan2(-R[1, 2], R[1, 1])
                    y = math.atan2(-R[2, 0], sy)
                    z = 0
                return np.array([x, y, z])

            # Get marker attitude in Euler angles
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip * R_tc)
            str_attitude = f"Orientacion MARCADOR r={math.degrees(roll_marker):.0f} p={math.degrees(pitch_marker):.0f} y={math.degrees(yaw_marker):.0f}"
            cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            print(str_attitude)

    else:
        print("No se detectaron marcadores en el cuadro actual.")

    # Display the frame
    cv2.imshow('frame', frame)

    # Press 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()
