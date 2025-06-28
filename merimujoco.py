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

MOT_START_FRAME = 200   # 開始フレーム
MOT_START_TIME = 1.0  # 開始時間

@dataclass
class Header:
    stamp: float  # UNIX時間など（ROSのTimeに相当）
    frame_id: str

@dataclass
class Quaternion:
    x: float
    y: float
    z: float
    w: float

@dataclass
class Vector3:
    x: float
    y: float
    z: float

@dataclass
class Imu:
    header: Header
    orientation: Quaternion
    orientation_covariance: List[float] = field(default_factory=lambda: [0.0]*9)
    angular_velocity: Vector3 = field(default_factory=Vector3)
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

redis_transfer = RedisTransfer(redis_key=REDIS_KEY_WRITE)
redis_receiver = RedisReceiver(redis_key=REDIS_KEY_READ)

total_frames = 0    # 全体のフレーム数
elapsed = 0.0       # 経過時間

# モデルを読み込む
#model = mujoco.MjModel.from_xml_path('/opt/mujoco/model/humanoid/humanoid.xml')
#model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/roborecipe4_go2_with_motors2.xml')
model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/scene.xml')
data = mujoco.MjData(model)


# --- 起動時に強制的に物理パラメータを上書き ---
model.opt.gravity[:] = [0, 0, -4.9]           # 重力を半分に
model.opt.timestep = 0.001                    # タイムステップ調整
model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4  # 安定な積分器に変更

# 全関節の減衰（damping）を強制上書き
model.dof_damping[:] = 5.0                    # ブレ防止

# 全geomの摩擦係数を上書き（静止摩擦、動摩擦、粘着摩擦）
model.geom_friction[:, :] = [2.0, 0.01, 0.001]  # 足裏の滑り防止




# ビューアを初期化（描画は別スレッドで自動）
viewer = launch_passive(model, data)

mdata = [0.0] * 90  # 初期化

def motor_controller_thread():
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

                    # IMU
                    imu = Imu(
                        header=Header(stamp=0.0, frame_id="base"),
                        orientation=Quaternion(
                            x=float(rcv_data[12]),
                            y=float(rcv_data[13]),
                            z=float(rcv_data[14]),
                            w=1.0
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

                    # imuとcmd_velのデータを表示
                    print(f"[Debug] real-sensor: {imu.orientation.x}, {imu.orientation.y}, {imu.orientation.z} + cmd_vel: {cmd_vel.linear.x}, {cmd_vel.linear.y}, {cmd_vel.angular.z}, cmd_btn: {cmd_btn}")

                    for joint_name, meridis_index in joint_to_meridis.items():
                        if joint_name in joint_names:
                            # Handle joint positions (convert from radians to degrees)
                            joint_idx = joint_names.index(joint_name)
                            meridis_idx = joint_to_meridis[joint_name][0]
                            meridis_mul = joint_to_meridis[joint_name][1]
                            data.ctrl[joint_idx] = round(np.radians(float(rcv_data[meridis_idx])*meridis_mul), 2)
                            #print(f"joint_name: {joint_name}, joint_idx: {joint_idx}, ctrl: {data.ctrl[joint_idx]}, mul: {joint_to_meridis[joint_name][1]}")

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
while viewer.is_running():

    total_frames += 1
    #print(f"Total frames: {total_frames}")

    elapsed = time.time() - start_time
    #print(f"Elapsed time: {elapsed:.4f} seconds")

    # mj_stepを呼び出し
    mujoco.mj_step(model, data)  # ステップ更新
    viewer.sync()                # 描画更新

    #time.sleep(0.001)            # 制御ループ：Hz