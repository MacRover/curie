import cv2
import numpy as np
from cv2 import aruco
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Quaternion
import threading
import time
from keyboard_interfaces.srv import KeyPose

# Pointing to the camera_calibration folder images to get array values for the camera's calibration
camera_calibration, dist_coefficients = load_coefficients("/camera_calibration.yml")

W = 0.354
H = 0.123
def make_key_positions():
    r2, r3, r4, r5, r6 = 17.0, 36.0, 55.0, 74.0, 93.0
    keys = {
        '`': (10.0, r2), '1': (29.05, r2), '2': (48.1, r2), '3': (67.15, r2),
        '4': (86.2, r2), '5': (105.25, r2), '6': (124.3, r2), '7': (143.35, r2),
        '8': (162.4, r2), '9': (181.45, r2), '0': (200.5, r2), '-': (219.55, r2),
        '=': (238.6, r2), 'BACKSPACE': (267.15, r2),
        'q': (38.6, r3), 'w': (57.65, r3), 'e': (76.7, r3), 'r': (95.75, r3),
        't': (114.8, r3), 'y': (133.85, r3), 'u': (152.9, r3), 'i': (171.95, r3),
        'o': (191.0, r3), 'p': (210.05, r3), '[': (229.1, r3), ']': (248.15, r3),
        'CAPS': (16.67, r4), 'a': (45.72, r4), 's': (64.77, r4), 'd': (83.82, r4),
        'f': (102.87, r4), 'g': (121.92, r4), 'h': (140.97, r4), 'j': (160.02, r4),
        'k': (179.07, r4), 'l': (198.12, r4), ';': (217.17, r4), "'": (236.22, r4),
        'ENTER': (265.77, r4),
        'SHIFT': (22.86, r5), 'z': (55.88, r5), 'x': (74.93, r5), 'c': (93.98, r5),
        'v': (113.03, r5), 'b': (132.08, r5), 'n': (151.13, r5), 'm': (170.18, r5),
        ',': (189.23, r5), '.': (208.28, r5), '/': (227.33, r5),
        'SPACE': (139.7, r6),
    }
    return {k: (x / 1000.0, y / 1000.0) for k, (x, y) in keys.items()}


class MarkerDetectionSystem(Node):

    def __init__(self):
        super().__init__('marker_detection_system')

        self.KEY_POSITIONS = make_key_positions()

        aruco_dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(aruco_dictionary, aruco_params)

        self.cap = cv2.VideoCapture(0)
        time.sleep(1.0)
        for _ in range(10):
            self.cap.read()
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            
        ret, probe_frame = self.cap.read()
        if ret:
            self.get_logger().info(
                f"[DEBUG] Camera frame shape: {probe_frame.shape} "
                f"(calibration cx={camera_calibration[0,2]}, cy={camera_calibration[1,2]})"
            )
        else:
            self.get_logger().warn("[DEBUG] Could not read a probe frame at startup to check resolution")

        self.pose_cv = threading.Condition()
        self.last_rvec = None
        self.last_tvec = None

        # frame shared with main thread for display only, guarded by its own lock
        self.frame_lock = threading.Lock()
        self.last_frame = None

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        self.service = self.create_service(
            KeyPose, 'key_pose', self.handle_request,
            callback_group=self.service_cb_group
        )
        self.timer = self.create_timer(
            1.0 / 30.0, self.timer_callback,
            callback_group=self.timer_cb_group
        )

        self.get_logger().info('MarkerDetectionSystem node started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera read failed', throttle_duration_sec=2.0)
            return

        corners, ids, rejected = self.detector.detectMarkers(frame)

        # DEBUG: what did detectMarkers actually see this frame?
        id_list = ids.flatten().tolist() if ids is not None else []
        self.get_logger().info(
            f"[DEBUG] Frame saw {len(id_list)} marker(s): {id_list}, "
            f"rejected candidates: {len(rejected) if rejected is not None else 0}",
            throttle_duration_sec=1.0
        )

        # draw detected markers onto a copy for display, regardless of pose success
        display_frame = frame.copy()
        if ids is not None:
            aruco.drawDetectedMarkers(display_frame, corners, ids)

        with self.frame_lock:
            self.last_frame = display_frame

        rvec, tvec = self.detect_markers(corners, ids)

        if rvec is not None and tvec is not None:
            # DEBUG: confirm a pose was actually produced this frame
            self.get_logger().info(
                f"[DEBUG] Pose computed successfully. "
                f"tvec={tvec.flatten().tolist()} rvec={rvec.flatten().tolist()}",
                throttle_duration_sec=1.0
            )
            with self.pose_cv:
                self.last_rvec = rvec
                self.last_tvec = tvec
                self.pose_cv.notify_all()
        else:
            # DEBUG: explicitly note failure to compute a pose this frame
            self.get_logger().info(
                "[DEBUG] No pose computed this frame (detect_markers returned None, None)",
                throttle_duration_sec=1.0
            )

    def detect_markers(self, corners, ids):
        if ids is None or len(ids) < 4:
            self.get_logger().info(
                f"[DEBUG] detect_markers: insufficient markers "
                f"(have {0 if ids is None else len(ids)}, need >= 4)",
                throttle_duration_sec=1.0
            )
            return None, None

        # DEBUG: flag if more than 4 markers are visible, since the corner-role
        # assignment logic below assumes exactly 4.
        if len(ids) > 4:
            self.get_logger().warn(
                f"[DEBUG] detect_markers: {len(ids)} markers visible, but corner-role "
                f"logic assumes exactly 4. Extra/duplicate markers may cause wrong "
                f"corner assignment.",
                throttle_duration_sec=1.0
            )

        centers = [(corners[i][0].mean(axis=0), mid, i)
                   for i, mid in enumerate(ids.flatten())]
        centers.sort(key=lambda c: c[0][0] + c[0][1])
        tl, br = centers[0], centers[-1]
        remaining = sorted(centers[1:3], key=lambda c: c[0][0])
        bl, tr = remaining[0], remaining[1]

        # DEBUG: show which marker ID got assigned to which corner role this frame
        self.get_logger().info(
            f"[DEBUG] Corner assignment -> tl:id{tl[1]} tr:id{tr[1]} "
            f"bl:id{bl[1]} br:id{br[1]}",
            throttle_duration_sec=1.0
        )

        marker_positions = {
            tl[1]: np.array([0.0, 0.0, 0.0], dtype=np.float32),
            tr[1]: np.array([W, 0.0, 0.0], dtype=np.float32),
            bl[1]: np.array([0.0, H, 0.0], dtype=np.float32),
            br[1]: np.array([W, H, 0.0], dtype=np.float32),
        }

        object_pts, image_pts = [], []
        for i, mid in enumerate(ids.flatten()):
            if mid in marker_positions:
                object_pts.append(marker_positions[mid])
                image_pts.append(corners[i][0].mean(axis=0))

        # DEBUG: how many correspondences were actually built
        self.get_logger().info(
            f"[DEBUG] Built {len(object_pts)} object/image point correspondences "
            f"(need >= 4 for solvePnP)",
            throttle_duration_sec=1.0
        )

        if len(object_pts) < 4:
            self.get_logger().info(
                "[DEBUG] Not enough matched correspondences to run solvePnP",
                throttle_duration_sec=1.0
            )
            return None, None

        object_pts = np.array(object_pts, dtype=np.float32).reshape(-1, 1, 3)
        image_pts = np.array(image_pts, dtype=np.float32).reshape(-1, 1, 2)

        try:
            success, rvec, tvec = cv2.solvePnP(
                object_pts, image_pts, camera_calibration, dist_coefficients,
                flags=cv2.SOLVEPNP_IPPE
            )
        except cv2.error as e:
            # DEBUG: surface any OpenCV-level exception from solvePnP explicitly
            self.get_logger().error(f"[DEBUG] solvePnP raised cv2.error: {e}")
            raise

        # DEBUG: explicit success/failure report from solvePnP itself
        self.get_logger().info(
            f"[DEBUG] solvePnP success={success}, "
            f"object_pts={len(object_pts)}, image_pts={len(image_pts)}",
            throttle_duration_sec=1.0
        )

        return (rvec, tvec) if success else (None, None)

    def handle_request(self, request, response):
        key = request.key
        self.get_logger().info(f"Looking for key '{key}'")

        if key not in self.KEY_POSITIONS:
            self.get_logger().warn(f"[DEBUG] Key '{key}' not found in KEY_POSITIONS")
            response.success = False
            return response

        with self.pose_cv:
            if self.last_rvec is None or self.last_tvec is None:
                self.get_logger().warn(f"Waiting for Aruco marker for key '{key}'...")
            self.pose_cv.wait_for(
                lambda: self.last_rvec is not None and self.last_tvec is not None
            )
            rvec = self.last_rvec.copy()
            tvec = self.last_tvec.copy()

        # DEBUG: confirm handle_request unblocked and which pose it's using
        self.get_logger().info(
            f"[DEBUG] handle_request for '{key}' unblocked. "
            f"Using tvec={tvec.flatten().tolist()}",
        )

        position = self.key_to_3d(key, rvec, tvec)

        # DEBUG: final computed 3D position for this key, in camera frame
        self.get_logger().info(
            f"[DEBUG] Key '{key}' -> camera-frame position: "
            f"x={position[0]:.4f} y={position[1]:.4f} z={position[2]:.4f}"
        )

        response.pose.header.stamp = self.get_clock().now().to_msg()
        response.pose.header.frame_id = "camera_frame"
        response.pose.pose.position.x = float(position[0])
        response.pose.pose.position.y = float(position[1])
        response.pose.pose.position.z = float(position[2])

        R, _ = cv2.Rodrigues(rvec)
        response.pose.pose.orientation = self.rotation_matrix_to_quaternion(R)
        response.success = True
        return response

    def key_to_3d(self, key, rvec, tvec):
        x, y = self.KEY_POSITIONS[key]
        key_pos_kb = np.array([[x, y, 0.0]], dtype=np.float32)
        R, _ = cv2.Rodrigues(rvec)
        pos_camera = R @ key_pos_kb.T + tvec

        # DEBUG: show the keyboard-local coordinate being transformed
        self.get_logger().info(
            f"[DEBUG] key_to_3d: key '{key}' keyboard-local (x={x:.4f}, y={y:.4f}) "
            f"-> camera-frame {pos_camera.flatten().tolist()}",
            throttle_duration_sec=1.0
        )

        return pos_camera.flatten()

    def rotation_matrix_to_quaternion(self, R):
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        q = Quaternion()
        q.x, q.y, q.z, q.w = x, y, z, w
        return q

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectionSystem()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # spin ROS callbacks in a background thread; keep imshow on the main thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            with node.frame_lock:
                frame = None if node.last_frame is None else node.last_frame.copy()

            if frame is not None:
                cv2.imshow('Aruco Feed', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()