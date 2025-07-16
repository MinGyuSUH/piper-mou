import mujoco
import mujoco.viewer
import os

# xml 경로 설정
xml_path = os.path.join(os.path.dirname(__file__), 'piper.xml')

# 모델 로드
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 뷰어 실행
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()