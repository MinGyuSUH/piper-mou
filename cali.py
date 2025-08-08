import numpy as np
import cv2

def invert_transform(T):
    """4x4 변환 행렬의 역변환을 계산합니다."""
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = R.T
    t_inv = -R.T @ t
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    return T_inv

# 🔹 1. 포즈 쌍 데이터 로드
data = np.load("handeye_data.npz", allow_pickle=True)
# data = np.load("handeye_data_filtered.npz", allow_pickle=True)

cam2target_list = data['cam2target'] 
base2ee_list = data['base2ee']       # 실제 데이터: T_base2ee

print(f"로드된 데이터 개수: {len(base2ee_list)}개")
assert len(cam2target_list) == len(base2ee_list), "Pose pair 개수가 일치하지 않습니다."
N = len(cam2target_list)

# 🔹 2. OpenCV 함수 형식에 맞게 데이터 준비
R_gripper2base = []
t_gripper2base = []
R_target2cam = []
t_target2cam = []

for i in range(N):
    # 로봇 포즈: base->ee 를 gripper->base 로 역변환
    T_base2ee = base2ee_list[i]
    T_gripper2base = invert_transform(T_base2ee)
    R_gripper2base.append(T_gripper2base[:3, :3])
    t_gripper2base.append(T_gripper2base[:3, 3])

    T_cam2target = cam2target_list[i]
    T_target2cam = invert_transform(T_cam2target)
    R_target2cam.append(T_target2cam[:3, :3])
    t_target2cam.append(T_target2cam[:3, 3])

# 🔹 3. 핸드-아이 캘리브레이션 실행 (결과는 T_cam2ee)
R_cam2ee, t_cam2ee = cv2.calibrateHandEye(
    R_gripper2base=R_gripper2base, 
    t_gripper2base=t_gripper2base,
    R_target2cam=R_target2cam, 
    t_target2cam=t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI
)

# 🔹 4. OpenCV 결과를 4x4 행렬 (T_cam2ee)로 변환
T_cam2ee = np.eye(4)
T_cam2ee[:3, :3] = R_cam2ee
T_cam2ee[:3, 3] = t_cam2ee.flatten()

# --- [핵심] ---
# 🔹 5. T_cam2ee를 역변환하여 최종 목표인 T_ee2cam (eeTc) 계산
T_ee2cam = invert_transform(T_cam2ee)
# --- [끝] ---

print("\n[✅] 최종 결과: End-Effector to Camera (T_ee2cam) transform:")
print(np.round(T_ee2cam, 4))

# 🔹 6. 최종 결과(T_ee2cam) 저장
np.save("T_ee2cam.npy", T_ee2cam)
print("\n[💾] T_ee2cam.npy 파일로 결과가 저장되었습니다.")


# 🔹 7. (보너스) 결과 검증 (Sanity Check)

# 첫 번째 데이터로 bTtarget 계산
T_c2target_0 = cam2target_list[0]
bTtarget_0 = base2ee_list[0] @ T_ee2cam @ T_c2target_0

# 마지막 데이터로 bTtarget 계산
T_c2target_N = cam2target_list[-1]
bTtarget_N = base2ee_list[-1] @ T_ee2cam @ T_c2target_N

# 두 행렬의 차이 계산
diff = np.linalg.norm(bTtarget_0 - bTtarget_N)

print("\n--- 결과 검증 ---")
print("첫 번째 데이터로 계산한 bTtarget:\n", np.round(bTtarget_0, 4))
print("마지막 데이터로 계산한 bTtarget:\n", np.round(bTtarget_N, 4))
print(f"두 bTtarget 행렬 간의 차이 (Frobenius Norm): {diff:.6f}")

if diff < 0.01: # 임의의 임계값 (보통 매우 작아야 함)
    print(">> 검증 결과: 매우 일관성 있음 (캘리브레이션 성공 확률 높음)")
else:
    print(">> 검증 결과: 일관성 부족 (데이터나 계산 과정 확인 필요)")
