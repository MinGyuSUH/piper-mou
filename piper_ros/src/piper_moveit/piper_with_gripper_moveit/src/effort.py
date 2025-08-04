import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
import threading

class EffortPlotter(Node):
    def __init__(self):
        super().__init__('effort_plotter')

        self.joint_names = [f'joint{i+1}' for i in range(8)]
        self.joint_count = len(self.joint_names)
        self.history_len = 100  # 최근 100개 데이터만 유지

        self.effort_history = [[] for _ in range(self.joint_count)]

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # 플롯 설정
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [], label=name)[0] for name in self.joint_names]
        self.ax.set_ylim(-1, 3)
        self.ax.set_xlim(0, self.history_len)
        self.ax.set_title("Joint Efforts")
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Effort")
        self.ax.legend(loc='upper left')

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

    def joint_callback(self, msg: JointState):
        # name → effort dict 생성
        effort_map = dict(zip(msg.name, msg.effort))
        efforts = [effort_map.get(jn, float('nan')) for jn in self.joint_names]

        # NaN 체크
        if any(math.isnan(e) for e in efforts):
            self.get_logger().warn("Effort contains NaN - skipping frame")
            return

        for i in range(self.joint_count):
            self.effort_history[i].append(efforts[i])
            if len(self.effort_history[i]) > self.history_len:
                self.effort_history[i].pop(0)

    def update_plot(self, frame):
        for i, line in enumerate(self.lines):
            y = self.effort_history[i]
            x = list(range(len(y)))
            line.set_data(x, y)
        return self.lines

def main():
    rclpy.init()
    node = EffortPlotter()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    plt.show()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()