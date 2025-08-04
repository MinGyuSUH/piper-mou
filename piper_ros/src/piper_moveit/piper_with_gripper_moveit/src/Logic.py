import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration # <--- 이 Duration은 이미 메시지 타입입니다.

from xela_server_ros2.msg import SensStream
import numpy as np
import threading

class ControlTower(Node):
    def __init__(self):
        super().__init__('control_tower_node')

        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.cum_x = [0.0] * 16
        self.cum_y = [0.0] * 16
        self.cum_z = [0.0] * 16

        self.z_aligned = False
        self.contacted = False

        self.sensor_update_count = 0

        self.subscription = self.create_subscription(
            SensStream,
            '/xServTopic',
            self.sensor_callback,
            10
        )       
        
        self._arm_client = ActionClient(self, FollowJointTrajectory, '/moveit_action/arm_controller/follow_joint_trajectory')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joint_names = ['joint7', 'joint8']
        
        self.ba_init = [-1.6379, 1.5724, -1.2721, -0.0566, 1.2838, 0.2908, 0.0, 0.0] 
        self.wm_init = [1.5640, 0.4538, -1.5319, -0.0488, 1.2797, 0.3169, 0.0, 0.0] 
        self.ba_end = [-1.5527, 2.6613, -1.5133, -0.2642, 0.6119, 0.3951, 0.0, 0.0]
        self.wm_end = [1.6093, 2.1485, -2.2774, -0.0159, 0.3409, 0.2625, 0.0, 0.0]      
        self.center_under = [1.5423, 1.0435, -1.0787, -0.0129, 0.6644, 0.2447, 0.0, 0.0]
        self.center_upper = [1.5654, 1.2227, -1.4379, -0.0288, 0.8714, 0.2447, 0.0, 0.0]

    def _send_goal(self, client, joint_names, joint_values, wait_for_result=True, timeout_sec=10.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in joint_values]
        
        # 시간 분기: 그리퍼는 1초, 팔은 5초
        if set(joint_names) == set(self.gripper_joint_names):
            point.time_from_start = Duration(sec=1, nanosec=0)
        else:
            point.time_from_start = Duration(sec=5, nanosec=0)
        
        goal_msg.trajectory.points.append(point)

        if not client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error(f"Action server '{client._action_name}' not available.")
            return False

        send_goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        if not wait_for_result:
            return True

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result()
        if not result:
            self.get_logger().error("결과를 받지 못했습니다.")
            return False

        return result.status == GoalStatus.STATUS_SUCCEEDED

    def send_joint_command(self, joint_values_8dof):
        self.get_logger().info(f"명령 수신: {joint_values_8dof}")
        arm_target = joint_values_8dof[0:6]
        gripper_target = joint_values_8dof[6:8]

        arm_success = self._send_goal(self._arm_client, self.arm_joint_names, arm_target, wait_for_result=True)
        if not arm_success:
            self.get_logger().error("팔 이동 실패. 다음 동작을 취소합니다.")
            return False

        self._send_goal(self._gripper_client, self.gripper_joint_names, gripper_target, wait_for_result=True)
        
        self.get_logger().info(f"명령 완료: {joint_values_8dof}")
        return True

    def goal(self, joint_values_8dof, gripper_action='keep'):

        self.get_logger().info(f"[Goal 명령] Pose: {joint_values_8dof}, Gripper: {gripper_action}")
        
        arm_target = joint_values_8dof[:6]
        gripper_target = joint_values_8dof[6:8]

        # 팔 먼저 이동
        success = self._send_goal(self._arm_client, self.arm_joint_names, arm_target, wait_for_result=True)
        if not success:
            self.get_logger().error("  >> 팔 이동 실패")
            return False

        # 그리퍼 동작 선택
        if gripper_action == 'open':
            self.open_gripper()
        elif gripper_action == 'close':
            self.close_gripper()
        elif gripper_action == 'keep':
            pass  # 아무것도 하지 않음
        else:
            self.get_logger().warn(f"  >> 알 수 없는 그리퍼 명령: {gripper_action}")

        return True

    def send_gripper_command(self, joint7, joint8, wait_for_result=True):
        gripper_target = [joint7, joint8]
        return self._send_goal(
            self._gripper_client,
            self.gripper_joint_names,
            gripper_target,
            wait_for_result=wait_for_result
        )

    def open_gripper(self):
        self.get_logger().info("그리퍼 오픈")
        return self.send_gripper_command(0.035, -0.035, wait_for_result=True)

    def close_gripper(self):
        self.get_logger().info("그리퍼 클로즈")
        return self.send_gripper_command(0.0, -0.0, wait_for_result=True)

    def check_z_and_contact(self):
        try:
            vecs = np.array([self.cum_x, self.cum_y, self.cum_z]).T
            if vecs.shape[0] < 1:
                self.get_logger().warn("센서 데이터 없음")
                return False, False

            norms = np.linalg.norm(vecs, axis=1)
            norms[norms == 0] = 1e-6
            unit_vecs = vecs / norms[:, None]
            weights = norms
            mean_vec = np.sum(unit_vecs * weights[:, None], axis=0) / np.sum(weights)

            norm_mean = np.linalg.norm(mean_vec)
            angle_deg = 0.0 if norm_mean < 1e-6 else np.degrees(np.arccos(
                np.clip(mean_vec[2] / norm_mean, -1.0, 1.0)
            ))

            self.z_aligned = norm_mean >= 0.7 and (angle_deg <= 45.0)
            self.contacted = norm_mean >= 0.7 and (45.0 < angle_deg < 135.0)

            self.get_logger().info(
                f"[정렬 판단] MRL={norm_mean:.3f}, θ={angle_deg:.2f}°, 접촉={self.contacted}"
            )

            return self.z_aligned, self.contacted
        except Exception as e:
            self.get_logger().error(f"[Z 및 접촉 판단 오류] {e}")
            return False, False

    def reset_sensor_data(self):
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.cum_x = [0.0] * 16
        self.cum_y = [0.0] * 16
        self.cum_z = [0.0] * 16
        self.sensor_update_count = 0

        self.z_aligned = False
        self.contacted = False

        self.get_logger().info("센서 기준값 및 누적값 초기화됨.")


    def sensor_callback(self, msg):
        try:
            x_vals = [t.x for t in msg.sensors[0].taxels]
            y_vals = [t.y for t in msg.sensors[0].taxels]
            z_vals = [t.z for t in msg.sensors[0].taxels]

            if self.prev_x is not None:
                dx = [c - p for c, p in zip(x_vals, self.prev_x)]
                dy = [c - p for c, p in zip(y_vals, self.prev_y)]
                dz = [c - p for c, p in zip(z_vals, self.prev_z)]

                for i in range(16):
                    self.cum_x[i] += dx[i]
                    self.cum_y[i] += dy[i]
                    self.cum_z[i] += dz[i]

            self.prev_x = x_vals
            self.prev_y = y_vals
            self.prev_z = z_vals

            self.sensor_update_count += 1
            # print(self.sensor_update_count)

        except Exception as e:
            self.get_logger().error(f"[센서 콜백 오류] {e}")

    def target_get(self):
        time.sleep(1)
        self.close_gripper()
        time.sleep(2)

    def overcome(self):
        self.goal(self.center_upper, 'close')


    def execute_sequence(self):
        self.check_z_and_contact()
        self.get_logger().info("Step 1: 이니셜 포즈(ba_init)로 이동 중")
        if not self.goal(self.ba_init, 'open'):
            self.get_logger().warn("  >> 실패 → 재시도")
            if not self.goal(self.ba_init, 'open'):
                self.get_logger().error("  >> 재시도 실패 → 시퀀스 종료")
                return
        self.reset_sensor_data()

        count = 0

        while True:
            # break
            self.get_logger().info("Step 2: 잡기 시도 중...")
            self.target_get()  # 감지 및 잡기 시도

            self.get_logger().info("🔍 Z-축 정렬 감지")
            self.z_aligned, _ = self.check_z_and_contact()

            if count == 1:
                break

            if self.z_aligned:
                self.get_logger().info("✅ 잡기 성공")
                break
            else:
                self.get_logger().warn("❌ 잡기 실패 → 이니셜 포즈(ba_init) 복귀 후 재시도")
                if not self.goal(self.ba_init, 'open'):
                    self.get_logger().warn("  >> 실패 → 재시도")
                    if not self.goal(self.ba_init, 'open'):
                        self.get_logger().error("  >> 재시도 실패 → 시퀀스 종료")
                        return
                self.reset_sensor_data()
            count = 1


        self.get_logger().info("Step 3: 엔드 포즈(wm_end)로 이동 중")
        if not self.goal(self.wm_end, 'keep'):
            self.get_logger().warn("  >> 실패 → 재시도")
            if not self.goal(self.wm_end, 'keep'):
                self.get_logger().error("  >> 재시도 실패 → 시퀀스 종료")
                return

        self.get_logger().info("Step 4: 접촉 감지 결과 확인 중")
        _, self.contacted = self.check_z_and_contact()
        self.open_gripper()

        if self.contacted:
            self.get_logger().info("  >> 접촉 감지됨 → 가운데로 이동")
            if not self.overcome():
                self.get_logger().warn("  >> 이동 실패 → 재시도")
                if not self.overcome():
                    self.get_logger().error("  >> 재시도 실패 → 시퀀스 종료")
                    return

            self.get_logger().info("  >> 다시 엔드 포즈 재시도")
            if not self.goal(self.wm_end, 'open'):
                self.get_logger().warn("  >> 재시도 실패 → 재시도")
                if not self.goal(self.wm_end, 'open'):
                    self.get_logger().error("  >> 재시도 실패 → 시퀀스 종료")
                    return

        else:
            self.get_logger().info("  >> 접촉 없음")

        self.get_logger().info("  >> 이니셜 포즈로 복귀")
        if not self.goal(self.wm_init, 'keep'):
            self.get_logger().warn("  >> 복귀 실패 → 재시도")
            if not self.goal(self.wm_init, 'keep'):
                self.get_logger().error("  >> 복귀 재시도 실패 → 시퀀스 종료")
                return
        if not self.goal(self.ba_init, 'keep'):
            self.get_logger().warn("  >> 복귀 실패 → 재시도")
            if not self.goal(self.ba_init, 'keep'):
                self.get_logger().error("  >> 복귀 재시도 실패 → 시퀀스 종료")
                return

        self.get_logger().info("--- 모든 시퀀스 완료 ---")
        return
    

def main(args=None):
    rclpy.init(args=args)
    node = ControlTower()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    threading.Thread(target=node.execute_sequence, daemon=True).start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의해 중단됨.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()