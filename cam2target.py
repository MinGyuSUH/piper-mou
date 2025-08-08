import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class HandEyeDataCollector(Node):
    def __init__(self):
        super().__init__('handeye_data_collector')
        self.ee_pose = None
        self.create_subscription(Pose, '/end_pose', self.ee_pose_callback, 10)

        # ArUco 보드 설정
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.board = aruco.GridBoard_create(5, 7, 0.031, 0.00775, self.dictionary)
        self.parameters = aruco.DetectorParameters_create()

        # 카메라 파라미터
        self.camera_matrix = np.array([
            [606.53295898, 0.0, 325.43719482],
            [0.0, 606.7074585, 248.36122131],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)

        # RealSense 파이프라인
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # 시각화용 3D plot
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        plt.ion()

        self.dataset = []
        self.last_valid_rvec = None
        self.last_valid_tvec = None
        self.last_save_time = time.time()

        self.get_logger().info("HandEyeDataCollector node has been started.")

    def ee_pose_callback(self, msg):
        pos = msg.position
        ori = msg.orientation
        r = R.from_quat([ori.x, ori.y, ori.z, ori.w])
        T = np.eye(4)
        T[:3, :3] = r.as_matrix()
        T[:3, 3] = [pos.x, pos.y, pos.z]
        self.ee_pose = T

    def save_all(self, filename='handeye_data.npz'):
        if not self.dataset:
            print("[!] No data to save.")
            return
        cam_poses = [x[0] for x in self.dataset]
        ee_poses = [x[1] for x in self.dataset]
        np.savez(filename, cam2target=cam_poses, base2ee=ee_poses)
        print(f"[💾] Saved {len(self.dataset)} pose pairs to {filename}")

    def run(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)

                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                frame = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

                pose_detected = False
                rvec, tvec = None, None

                if ids is not None and len(ids) > 0:
                    retval, rvec, tvec = aruco.estimatePoseBoard(
                        corners, ids, self.board, self.camera_matrix, self.dist_coeffs,
                        None, None
                    )
                    if retval > 0:
                        pose_detected = True
                        self.last_valid_rvec = rvec
                        self.last_valid_tvec = tvec
                        aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

                aruco.drawDetectedMarkers(frame, corners, ids)

                # 텍스트
                if pose_detected and self.ee_pose is not None:
                    rvec_str = np.array2string(rvec.flatten(), precision=2, separator=',')
                    tvec_str = np.array2string(tvec.flatten(), precision=2, separator=',')
                    pos = self.ee_pose[:3, 3]
                    quat = R.from_matrix(self.ee_pose[:3, :3]).as_quat()

                    cv2.putText(frame, f"rvec: {rvec_str}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(frame, f"tvec: {tvec_str}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    cv2.putText(frame, f"pos: {pos}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

                else:
                    cv2.putText(frame, "Waiting for detection...", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                cv2.imshow("Hand-Eye Data Collector", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Quitting...")
                    break

                elif key == ord('s') and pose_detected and self.ee_pose is not None:
                    T_cam = np.eye(4)
                    R_cam, _ = cv2.Rodrigues(self.last_valid_rvec)
                    T_cam[:3, :3] = R_cam
                    T_cam[:3, 3] = self.last_valid_tvec.flatten() ### T_cam2target
                    self.dataset.append((T_cam, self.ee_pose.copy()))
                    print(f"[✔] Pose pair #{len(self.dataset)} saved by user input.")
                
        finally:
            self.save_all()
            self.pipeline.stop()
            cv2.destroyAllWindows()
            plt.ioff()
            plt.close()
            self.get_logger().info("Node has been shut down cleanly.")

def main(args=None):
    rclpy.init(args=args)
    node = HandEyeDataCollector()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
