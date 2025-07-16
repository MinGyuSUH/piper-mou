import rclpy
import os
from .multi_thread_interface import MujocoROSBridge
from ament_index_python.packages import get_package_share_directory

def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    rclpy.init()

    share_dir = get_package_share_directory('piper_mujoco')
   
    xml_path = os.path.join(share_dir, 'environment.xml')


    robot_info = [xml_path, 1000]
    # camera_info = ['hand_eye', 320, 240, 30]

    # bridge = MujocoROSBridge(robot_info, camera_info, rc)
    bridge = MujocoROSBridge(robot_info)

    # time.sleep(2.0)
    try:
        bridge.run()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


    
if __name__=="__main__":    
    main()
