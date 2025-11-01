import mujoco
from mujoco.viewer import launch_passive
import numpy as np
import threading
import time

import numpy as np
import os
import time
import math

from redis_transfer import RedisTransfer
from redis_receiver import RedisReceiver

# 構造体を宣言する
from dataclasses import dataclass, field
from typing import List

MSG_SIZE = 90                   # Meridim配列の長さ
REDIS_KEY_WRITE = "meridis"     # 読み込むRedisキー (キーA)
REDIS_KEY_READ  = "meridis2"    # 書き込むRedisキー (キーB)
CMD_VEL_GAIN = 1.0              # cmd_velのゲイン (0~1)
FLG_SET_RCVD = True             # Redisからのデータ受信フラグ
FLG_CREATE_CTRL = False          # 制御信号作成フラグ
FLG_SET_SNDD = True             # Redisへのデータ送信フラグ
FLG_RESET_REQUEST = False       # リセット要求フラグ

MOT_START_FRAME = 200   # 開始フレーム
MOT_START_TIME = 1.0  # 開始時間

@dataclass
class Header:
    stamp: float  # UNIX時間など（ROSのTimeに相当）
    frame_id: str

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class Imu:
    header: Header
    orientation: Vector3
    orientation_covariance: List[float] = field(default_factory=lambda: [0.0]*9)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    linear_acceleration: Vector3 = field(default_factory=lambda: Vector3(0.0, 0.0, 0.0))
    angular_velocity_covariance: List[float] = field(default_factory=lambda: [0.0]*9)

@dataclass
class Twist:
    linear: Vector3
    angular: Vector3


# ダミーの関節名リスト
joint_names = [
    "c_chest", "c_head", "l_shoulder_pitch", "l_shoulder_roll", "l_elbow_yaw", "l_elbow_pitch",
    "r_shoulder_pitch", "r_shoulder_roll", "r_elbow_yaw", "r_elbow_pitch",
    "l_hip_yaw", "l_hip_roll", "l_thigh_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_thigh_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll"
]

# Enemy robot joint names
enemy_joint_names = [
    "enemy_c_chest", "enemy_c_head", "enemy_l_shoulder_pitch", "enemy_l_shoulder_roll", "enemy_l_elbow_yaw", "enemy_l_elbow_pitch",
    "enemy_r_shoulder_pitch", "enemy_r_shoulder_roll", "enemy_r_elbow_yaw", "enemy_r_elbow_pitch",
    "enemy_l_hip_yaw", "enemy_l_hip_roll", "enemy_l_thigh_pitch", "enemy_l_knee_pitch", "enemy_l_ankle_pitch", "enemy_l_ankle_roll",
    "enemy_r_hip_yaw", "enemy_r_hip_roll", "enemy_r_thigh_pitch", "enemy_r_knee_pitch", "enemy_r_ankle_pitch", "enemy_r_ankle_roll"
]


# Joint mapping dictionary
joint_to_meridis = {
    # Base link
    "base_roll":        [12, 1],
    "base_pitch":       [13, 1],
    "base_yaw":         [14, 1],
    # Head
    "c_head":           [21, 1],
    # Left arm
    "l_shoulder_pitch": [23, 1],
    "l_shoulder_roll":  [25, 1],
    "l_elbow_yaw":      [27, 1],
    "l_elbow_pitch":    [29, 1],
    # Left leg
    "l_hip_yaw":        [31, 1],
    "l_hip_roll":       [33, 1],
    "l_thigh_pitch":    [35, 1],
    "l_knee_pitch":     [37, 1],
    "l_ankle_pitch":    [39, 1],
    "l_ankle_roll":     [41, 1],
    # chest
    "c_chest":          [51, 1],
    # Right arm
    "r_shoulder_pitch": [53, 1],
    "r_shoulder_roll":  [55,-1],
    "r_elbow_yaw":      [57,-1],
    "r_elbow_pitch":    [59, 1],
    # Right leg
    "r_hip_yaw":        [61,-1],
    "r_hip_roll":       [63,-1],
    "r_thigh_pitch":    [65, 1],
    "r_knee_pitch":     [67, 1],
    "r_ankle_pitch":    [69, 1],
    "r_ankle_roll":     [71,-1]
}

# Enemy robot joint mapping (same indices as main robot)
enemy_joint_to_meridis = {
    # Base link
    "enemy_base_roll":        [12, 1],
    "enemy_base_pitch":       [13, 1],
    "enemy_base_yaw":         [14, 1],
    # Head
    "enemy_c_head":           [21, 1],
    # Left arm
    "enemy_l_shoulder_pitch": [23, 1],
    "enemy_l_shoulder_roll":  [25, 1],
    "enemy_l_elbow_yaw":      [27, 1],
    "enemy_l_elbow_pitch":    [29, 1],
    # Left leg
    "enemy_l_hip_yaw":        [31, 1],
    "enemy_l_hip_roll":       [33, 1],
    "enemy_l_thigh_pitch":    [35, 1],
    "enemy_l_knee_pitch":     [37, 1],
    "enemy_l_ankle_pitch":    [39, 1],
    "enemy_l_ankle_roll":     [41, 1],
    # chest
    "enemy_c_chest":          [51, 1],
    # Right arm
    "enemy_r_shoulder_pitch": [53, 1],
    "enemy_r_shoulder_roll":  [55,-1],
    "enemy_r_elbow_yaw":      [57,-1],
    "enemy_r_elbow_pitch":    [59, 1],
    # Right leg
    "enemy_r_hip_yaw":        [61,-1],
    "enemy_r_hip_roll":       [63,-1],
    "enemy_r_thigh_pitch":    [65, 1],
    "enemy_r_knee_pitch":     [67, 1],
    "enemy_r_ankle_pitch":    [69, 1],
    "enemy_r_ankle_roll":     [71,-1]
}

redis_transfer = RedisTransfer(redis_key=REDIS_KEY_WRITE)
redis_receiver = RedisReceiver(redis_key=REDIS_KEY_READ)

total_frames = 0    # 全体のフレーム数
elapsed = 0.0       # 経過時間

# モデルを読み込む
#model = mujoco.MjModel.from_xml_path('/opt/mujoco/model/humanoid/humanoid.xml')
#model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/roborecipe4_go2_with_motors2.xml')
#model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/scene.xml')
model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/scene_vs.xml')
data = mujoco.MjData(model)


# --- 起動時に強制的に物理パラメータを上書き ---
model.opt.gravity[:] = [0, 0, -9.8]           # 重力を半分に
model.opt.timestep = 0.001                    # タイムステップ調整
model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4  # 安定な積分器に変更
# 全関節の減衰（damping）を強制上書き
model.dof_damping[:] = 5.0                    # ブレ防止
# 全geomの摩擦係数を上書き（静止摩擦、動摩擦、粘着摩擦）
model.geom_friction[:, :] = [1.2, 0.8, 0.01]  # 着地安定化用の摩擦調整



# ビューアを初期化（描画は別スレッドで自動）
viewer = launch_passive(model, data)

mdata = [0.0] * 90  # 初期化
imu_mjc = Imu(
    header=Header(stamp=0.0, frame_id="c_chest"),
    # orientation を roll/pitch/yaw(deg) として保持する
    orientation=Vector3(0.0, 0.0, 0.0),
    angular_velocity=Vector3(0.0, 0.0, 0.0),
    linear_acceleration=Vector3(0.0, 0.0, 0.0)
)

def motor_controller_thread():
    global imu_mjc, FLG_RESET_REQUEST
    while True:
        if FLG_SET_RCVD and elapsed >= MOT_START_TIME:  # データ受信フラグが立っていて、開始時間を超えたら
            # meridis2キーからデータを読み込む
            #start_time = time.perf_counter()
            rcv_data = redis_receiver.get_data()
            #elapsed_time = time.perf_counter() - start_time
            #print(f"receive elapsed time: {elapsed_time*1000000:.2f} microseconds ({elapsed_time:.6f} seconds)")

            if rcv_data:

                #print(f"rcv data: {rcv_data}") # meridian -> redis データを確認

                if len(rcv_data) == MSG_SIZE:
                    # データの更新
                    #print(f"rcv data: {rcv_data}") # meridian -> redis データを確認

                    if rcv_data[0] == 5556:
                        # リセット要求フラグを立てる（メインループで実行）
                        print(f"[motor_controller_thread] mujoco reset request {rcv_data[0]}")
                        FLG_RESET_REQUEST = True

                        
                    # IMU
                    # 受信側 imu は roll/pitch/yaw を直接使う想定（deg）
                    imu_r = Imu(
                        header=Header(stamp=0.0, frame_id="base"),
                        orientation=Vector3(
                            x=float(rcv_data[12]),
                            y=float(rcv_data[13]),
                            z=float(rcv_data[14])
                        ),
                        angular_velocity=Vector3(
                            x=float(rcv_data[5]),
                            y=float(rcv_data[6]),
                            z=float(rcv_data[7])
                        )
                    )

                    # Remo
                    cmd_btn = float(rcv_data[15])

                    line_vel_x = float(rcv_data[16] * CMD_VEL_GAIN)     # line_vel_x
                    line_vel_y = float(rcv_data[17] * CMD_VEL_GAIN)     # line_vel_y
                    ang_vel_z = float(rcv_data[18] * CMD_VEL_GAIN)     # ang_vel_z +Hori 20250510 Test

                    cmd_vel = Twist(
                        linear=Vector3(x=line_vel_x, y=line_vel_y, z=0.0),       # x:前進, y:左右
                        angular=Vector3(x=0.0, y=0.0, z=ang_vel_z)     # z:z軸=yaw軸旋回
                    )

                    # 受信したimuとcmd_velのデータを表示
                    # print(f"[Debug] rcv: {imu_r.orientation.x}, {imu_r.orientation.y}, {imu_r.orientation.z} + cmd_vel: {cmd_vel.linear.x}, {cmd_vel.linear.y}, {cmd_vel.angular.z}, cmd_btn: {cmd_btn}")

                    for joint_name, meridis_index in joint_to_meridis.items():
                        if joint_name in joint_names:
                            # Handle joint positions (convert from radians to degrees)
                            joint_idx = joint_names.index(joint_name)
                            meridis_idx = joint_to_meridis[joint_name][0]
                            meridis_mul = joint_to_meridis[joint_name][1]
                            data.ctrl[joint_idx] = round(np.radians(float(rcv_data[meridis_idx])*meridis_mul), 2)
                            #print(f"joint_name: {joint_name}, joint_idx: {joint_idx}, ctrl: {data.ctrl[joint_idx]}, mul: {joint_to_meridis[joint_name][1]}")

                    # Update enemy robot joints from redis data
                    for enemy_joint_name, meridis_index in enemy_joint_to_meridis.items():
                        if enemy_joint_name in enemy_joint_names:
                            # Handle joint positions (convert from radians to degrees)
                            enemy_joint_idx = enemy_joint_names.index(enemy_joint_name)
                            # enemy joint index offset = number of main robot joints
                            enemy_ctrl_idx = len(joint_names) + enemy_joint_idx
                            meridis_idx = enemy_joint_to_meridis[enemy_joint_name][0]
                            meridis_mul = enemy_joint_to_meridis[enemy_joint_name][1]
                            data.ctrl[enemy_ctrl_idx] = round(np.radians(float(rcv_data[meridis_idx])*meridis_mul), 2)
                            #print(f"enemy_joint_name: {enemy_joint_name}, enemy_ctrl_idx: {enemy_ctrl_idx}, ctrl: {data.ctrl[enemy_ctrl_idx]}")

                    # mdataの更新 +Hori 20250628
                    for i in range(len(mdata)):
                        if i < len(rcv_data):
                            mdata[i] = float(rcv_data[i])
                        else:
                            mdata[i] = 0.0

                    #print(f"mdata: {mdata}")

            if FLG_CREATE_CTRL and elapsed >= MOT_START_TIME:  # 制御信号作成フラグが立っていて、開始時間を超えたら
                # make actions:データの更新

                for joint_name, meridis_index in joint_to_meridis.items():

                    if joint_name in joint_names:
                        # Handle joint positions (convert from radians to degrees)
                        joint_idx = joint_names.index(joint_name)

                        mot_ctrl = (elapsed - MOT_START_TIME)

                        # 各関節に対応する制御振幅（中心を0とする正弦波）
                        amplitude = {
                            "thigh_pitch": math.radians(-30),   # -30°
                            "knee_pitch": math.radians(60),   # 60°
                            "ankle_pitch": math.radians(-30)   # -30°
                        }

                        # モデルの関節に応じて制御信号を設定
                        for joint_type, amp in amplitude.items():
                            if joint_name == f"l_{joint_type}" or joint_name == f"r_{joint_type}":
                                data.ctrl[joint_idx] = amp * abs(math.sin(mot_ctrl))

                            mdata[meridis_index[0]] = round(np.degrees(float(data.ctrl[joint_idx])), 2)

                joint_idx = joint_names.index("c_head")  # c_headのインデックスを取得
                data.ctrl[joint_idx] = 1.0 * ang_vel_z
                mdata[joint_to_meridis["c_head"][0]] = round(np.degrees(float(data.ctrl[joint_idx])), 2)
                joint_idx = joint_names.index("l_shoulder_pitch")  # c_headのインデックスを取得
                data.ctrl[joint_idx] = 1.0 * -line_vel_y
                mdata[joint_to_meridis["l_shoulder_pitch"][0]] = round(np.degrees(float(data.ctrl[joint_idx])), 2)
                print(f"mdata: {mdata}")

            # Redis にデータを送信
            if FLG_SET_SNDD and elapsed >= MOT_START_TIME:  # データ送信フラグが立っていて、開始時間を超えたら

                # mujocoのIMUデータを小数点2桁で表示
                print(f"[Debug] mjc: {imu_mjc.orientation.x:.2f}, {imu_mjc.orientation.y:.2f}, {imu_mjc.orientation.z:.2f}")

                # Main robot data to Redis
                mdata[2] = round(imu_mjc.linear_acceleration.x, 4)   # ax(m/s^2)
                mdata[3] = round(imu_mjc.linear_acceleration.y, 4)   # ay(m/s^2)
                mdata[4] = round(imu_mjc.linear_acceleration.z, 4)   # az(m/s^2)
                mdata[5]  = round(imu_mjc.angular_velocity.x, 4)   # wx(deg/s)
                mdata[6]  = round(imu_mjc.angular_velocity.y, 4)   # wy(deg/s)
                mdata[7]  = round(imu_mjc.angular_velocity.z, 4)   # wz(deg/s)
                mdata[12] = round(imu_mjc.orientation.x, 4)   # roll(deg)
                mdata[13] = round(imu_mjc.orientation.y, 4)   # pitch(deg)
                mdata[14] = round(imu_mjc.orientation.z, 4)   # yaw(deg)
                
                # Enemy robot data to Redis (commented out)
                # mdata[?] = round(imu_enemy.linear_acceleration.x, 4)   # enemy ax(m/s^2)
                # mdata[?] = round(imu_enemy.linear_acceleration.y, 4)   # enemy ay(m/s^2)
                # mdata[?] = round(imu_enemy.linear_acceleration.z, 4)   # enemy az(m/s^2)
                # mdata[?] = round(imu_enemy.angular_velocity.x, 4)      # enemy wx(deg/s)
                # mdata[?] = round(imu_enemy.angular_velocity.y, 4)      # enemy wy(deg/s)
                # mdata[?] = round(imu_enemy.angular_velocity.z, 4)      # enemy wz(deg/s)
                # mdata[?] = round(imu_enemy.orientation.x, 4)           # enemy roll(deg)
                # mdata[?] = round(imu_enemy.orientation.y, 4)           # enemy pitch(deg)
                # mdata[?] = round(imu_enemy.orientation.z, 4)           # enemy yaw(deg)
                
                #start_time = time.perf_counter()
                redis_transfer.set_data(REDIS_KEY_WRITE, mdata)
                #elapsed_time = time.perf_counter() - start_time
                #print(f"transfer elapsed time: {elapsed_time*1000000:.2f} microseconds ({elapsed_time:.6f} seconds)")


        time.sleep(0.02)  # 10ms待機

# スレッドを開始
mot_ctrl_thread = threading.Thread(target=motor_controller_thread, daemon=True)
mot_ctrl_thread.start()




# メインループで制御＋mj_step
start_time = time.time()
chest_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "c_chest")
while viewer.is_running():
    total_frames += 1
    elapsed = time.time() - start_time

    # リセット要求がある場合はリセットを実行
    if FLG_RESET_REQUEST:
        print(f"[mainloop] executing mujoco reset")
        mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        viewer.sync()
        FLG_RESET_REQUEST = False
        print(f"[mainloop] reset completed")
        continue

    mujoco.mj_step(model, data)  # ステップ更新
    viewer.sync()                # 描画更新

    # --- c_chestのIMU計算（mj_step直後）---
    # 姿勢
    xmat = data.xmat
    arr = np.array(xmat)
    if arr.ndim == 2 and arr.shape[1] == 9:
        chest_mat = arr[chest_body_id].reshape(3, 3)
    else:
        flat = arr.flatten()
        start_idx = chest_body_id * 9
        end_idx = (chest_body_id + 1) * 9
        if end_idx <= flat.size:
            chest_mat = flat[start_idx:end_idx].reshape(3, 3)
        else:
            chest_mat = np.eye(3)
    yaw = math.atan2(float(chest_mat[1, 0]), float(chest_mat[0, 0]))
    pitch = math.asin(max(-1.0, min(1.0, -float(chest_mat[2, 0]))))
    roll = math.atan2(float(chest_mat[2, 1]), float(chest_mat[2, 2]))
    yaw_deg = math.degrees(yaw)
    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)
    # 角速度（data.cvel: shape=(nbody, 6)）
    ang_vel = Vector3(0.0, 0.0, 0.0)
    try:
        cvel = np.array(data.cvel)
        if cvel.ndim == 2 and cvel.shape[1] >= 6:
            wx, wy, wz = cvel[chest_body_id, 3:6]
        else:
            flat = cvel.flatten()
            start_idx = chest_body_id * 6
            end_idx = (chest_body_id + 1) * 6
            if end_idx <= flat.size:
                seg = flat[start_idx:end_idx]
                wx, wy, wz = seg[3:6]
            else:
                wx, wy, wz = 0.0, 0.0, 0.0
        ang_vel = Vector3(math.degrees(float(wx)), math.degrees(float(wy)), math.degrees(float(wz)))
    except Exception as e:
        print(f"[mainloop] 角速度取得エラー: {e}")
        ang_vel = Vector3(0.0, 0.0, 0.0)
    # 加速度（重力ベクトルをc_chest座標系へ変換）
    try:
        g = np.array(model.opt.gravity)  # shape=(3,)
        # chest_mat: ワールド→c_chest の回転行列
        # gはワールド座標系なので、c_chest座標系へは R^T @ g
        lin_acc_arr = chest_mat.T @ g
        lin_acc = Vector3(float(lin_acc_arr[0]), float(lin_acc_arr[1]), float(lin_acc_arr[2]))
    except Exception as e:
        print(f"[mainloop] 重力変換エラー: {e}")
        lin_acc = Vector3(0.0, 0.0, 0.0)
    imu_mjc = Imu(
        header=Header(stamp=time.time(), frame_id="c_chest"),
        orientation=Vector3(roll_deg, pitch_deg, yaw_deg),
        angular_velocity=ang_vel,
        linear_acceleration=lin_acc
    )
    # デバッグ表示
    #print(f"IMU Debug Info - Time: {time.time()}")
    #print(f"  Orientation (deg): {imu_mjc.orientation}")
    #print(f"  Angular Velocity (deg/s): {imu_mjc.angular_velocity}")
    #print(f"  Linear Acceleration (m/s^2): {imu_mjc.linear_acceleration}")


