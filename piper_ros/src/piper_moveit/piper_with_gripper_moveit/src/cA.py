import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
import pyrealsense2 as rs
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import threading
import time

class CameraHoverNode(Node):
    def __init__(self):
        super().__init__('camera_hover_node')

        self.bTc = np.eye(4)
        self.T_ec = self.get_eTc_matrix()
        self.mouse_x = -1
        self.mouse_y = -1
        self.intrinsics = None
        self.latest_text_lines = []

        # ROS 2 구독
        self.create_subscription(Pose, '/end_pose', self.end_pose_callback, 10)

        # Realsense 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        # OpenCV 마우스 콜백
        cv2.namedWindow("Color Frame")
        cv2.setMouseCallback("Color Frame", self.mouse_callback)

        # GUI 스레드 실행
        self.gui_thread = threading.Thread(target=self.run_gui_loop)
        self.gui_thread.start()

    def get_eTc_matrix(self):
        eR = np.array([[0, 1, 0],
                       [-1, 0, 0],
                       [0, 0, 1]])
        # eT = np.array([0.045, -0.045, 0.05])
        eT = np.array([-0.1,0,0])
        T = np.eye(4)
        T[:3, :3] = eR
        T[:3, 3] = eT
        return T

    def end_pose_callback(self, msg):
        # print("[CALLBACK] /end_pose 수신함")
        T_be = self.pose_to_matrix(msg)
        self.bTc = T_be @ self.T_ec
        # self.get_logger().info('Updated bTc from /end_pose')
        # print("[UPDATE] bTc:\n", self.bTc)

    def pose_to_matrix(self, pose):
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R_mat = R.from_quat(q).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = t
        return T

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_x = x
            self.mouse_y = y

    def draw_text(self, img, lines, start=(10, 30), line_height=25):
        for i, line in enumerate(lines):
            pos = (start[0], start[1] + i * line_height)
            cv2.putText(img, line, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def run_gui_loop(self):
        try:
            while rclpy.ok():
                frames = self.pipeline.wait_for_frames()
                aligned = self.align.process(frames)
                depth_frame = aligned.get_depth_frame()
                color_frame = aligned.get_color_frame()
                if not color_frame or not depth_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())

                if self.mouse_x >= 0 and self.mouse_y >= 0:
                    depth = depth_frame.get_distance(self.mouse_x, self.mouse_y)
                    if depth > 0:
                        if self.intrinsics is None:
                            self.intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                        cA = rs.rs2_deproject_pixel_to_point(
                            self.intrinsics, [self.mouse_x, self.mouse_y], depth)
                        cA_hom = np.array([cA[0], cA[1], cA[2], 1.0]).reshape((4, 1))
                        bA = self.bTc @ cA_hom
                        self.latest_text_lines = [
                            f"Pixel: ({self.mouse_x}, {self.mouse_y})",
                            f"Depth: {depth:.3f} m",
                            f"cA (camera): {cA[0]:.3f}, {cA[1]:.3f}, {cA[2]:.3f}",
                            f"bA (base): {bA[0,0]:.3f}, {bA[1,0]:.3f}, {bA[2,0]:.3f}"
                        ]
                    else:
                        self.latest_text_lines = [
                            f"Pixel: ({self.mouse_x}, {self.mouse_y})",
                            f"No depth data"
                        ]

                self.draw_text(color_image, self.latest_text_lines)
                cv2.imshow("Color Frame", color_image)
                if cv2.waitKey(1) == 27:
                    break
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraHoverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()