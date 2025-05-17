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
FLG_CREATE_CTRL = True          # 制御信号作成フラグ
FLG_SET_SNDD = True             # Redisへのデータ送信フラグ

MOT_START_FRAME = 200

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

total_frames = 0

# モデルを読み込む
#model = mujoco.MjModel.from_xml_path('/opt/mujoco/model/humanoid/humanoid.xml')
#model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/roborecipe4_go2_with_motors2.xml')
model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/scene.xml')
data = mujoco.MjData(model)

# ビューアを初期化（描画は別スレッドで自動）
viewer = launch_passive(model, data)

# メインループで制御＋mj_step
start_time = time.time()
while viewer.is_running():

    total_frames += 1
    print(f"Total frames: {total_frames}")
    # joint_pos, joint_names, base_euler を 90要素の配列に変換
    mdata = [0.0] * 90  # 初期化

    elapsed = time.time() - start_time

    if FLG_SET_RCVD:
        # meridis2キーからデータを読み込む
        rcv_data = redis_receiver.get_data()
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
                        #print(f"joint_name: {joint_name}, joint_idx: {joint_idx}, data.ctrl: {data.ctrl[joint_idx]}")


        if FLG_CREATE_CTRL:
            # make actions:データの更新

            for joint_name, meridis_index in joint_to_meridis.items():

                if joint_name in joint_names:
                    # Handle joint positions (convert from radians to degrees)
                    joint_idx = joint_names.index(joint_name)

                    # 回転テスト
                    if total_frames >= MOT_START_FRAME:
                        mot_cnt = total_frames - MOT_START_FRAME
                        mot_ctrl = mot_cnt*0.01

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

                #print(f"mdata: {mdata}")

        # Redis にデータを送信
        if FLG_SET_SNDD:
            redis_transfer.set_data(REDIS_KEY_WRITE, mdata)


    # mj_stepを呼び出し
    mujoco.mj_step(model, data)  # ステップ更新
    viewer.sync()                # 描画更新

    time.sleep(0.001)             # 制御ループ：100Hz