import numpy as np
import cv2
import cv2.aruco as aruco
import time
import math


class ArucoSingleTracker:
    def __init__(self, id_to_find, marker_size, camera_matrix, camera_distortion, camera_size=(1280, 720), show_video=False):
        self.id_to_find = id_to_find
        self.marker_size = marker_size
        self._show_video = show_video

        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion

        self.is_detected = False
        self._kill = False

        # 180° rotation matrix around the x-axis
        self._R_flip = np.array([[1.0, 0.0, 0.0],
                                  [0.0, -1.0, 0.0],
                                  [0.0, 0.0, -1.0]], dtype=np.float32)

        # Define the aruco dictionary
        self._aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self._parameters = aruco.DetectorParameters()

        # Capture the video feed
        self._cap = cv2.VideoCapture(0)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

        # Font for text
        self.font = cv2.FONT_HERSHEY_PLAIN

        self._t_read = time.time()
        self._t_detect = self._t_read
        self.fps_read = 0.0
        self.fps_detect = 0.0

    def _rotationMatrixToEulerAngles(self, R):
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6

        assert isRotationMatrix(R)

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

    def _update_fps_read(self):
        t = time.time()
        self.fps_read = 1.0 / (t - self._t_read)
        self._t_read = t

    def _update_fps_detect(self):
        t = time.time()
        self.fps_detect = 1.0 / (t - self._t_detect)
        self._t_detect = t

    def stop(self):
        self._kill = True

    def track(self, loop=True, verbose=False, show_video=None):
        self._kill = False
        if show_video is None:
            show_video = self._show_video

        marker_found = False
        x = y = z = 0

        while not self._kill:
            # Read the camera frame
            ret, frame = self._cap.read()
            if not ret:
                print("Failed to capture frame.")
                break

            self._update_fps_read()

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect aruco markers
            corners, ids, _ = aruco.detectMarkers(
                image=gray,
                dictionary=self._aruco_dict,
                parameters=self._parameters,
            )

            if ids is not None and self.id_to_find in ids:
                marker_found = True
                self._update_fps_detect()

                # Get rotation and translation vectors
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                x, y, z = tvec

                # Draw detected marker and axis
                aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)


                # Compute rotation matrix and camera position
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T

                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(self._R_flip @ R_tc)
                pos_camera = -R_tc @ np.matrix(tvec).T

                if verbose:
                    print(f"Marker X={tvec[0]:.1f} Y={tvec[1]:.1f} Z={tvec[2]:.1f} - FPS={self.fps_detect:.0f}")

                if show_video:
                    # Display marker information
                    cv2.putText(frame, f"Position x={tvec[0]:.0f} y={tvec[1]:.0f} z={tvec[2]:.0f}",
                                (10, 50), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(frame, f"Attitude r={math.degrees(roll_marker):.0f} p={math.degrees(pitch_marker):.0f} y={math.degrees(yaw_marker):.0f}",
                                (10, 100), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)


            elif verbose:
                print(f"Nothing detected - FPS={self.fps_read:.0f}")

            if show_video:
                cv2.imshow("frame", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self._cap.release()
                    cv2.destroyAllWindows()
                    break

            if not loop:
                return marker_found, x, y, z


if __name__ == "__main__":
    # Define Tag
    id_to_find = 20
    marker_size = 30  # cm

    # Get the camera calibration path
    calib_path = "/home/admin/Drones/opencv/arducam/"
    camera_matrix = np.loadtxt(calib_path + "cameraMatrix.txt", delimiter=",")
    camera_distortion = np.loadtxt(calib_path + "cameraDistortion.txt", delimiter=",")
    aruco_tracker = ArucoSingleTracker(id_to_find=id_to_find, marker_size=marker_size,
                                        show_video=True, camera_matrix=camera_matrix,
                                        camera_distortion=camera_distortion)

    aruco_tracker.track(verbose=True)
