import mujoco
from mujoco.viewer import launch
from mujoco.glfw import glfw

import numpy as np
import os
import time
import math

from redis_transfer import RedisTransfer
from redis_receiver import RedisReceiver

# 構造体を宣言する
from dataclasses import dataclass, field
from typing import List

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


MSG_SIZE = 90                   # Meridim配列の長さ
REDIS_KEY_WRITE = "meridis"     # 読み込むRedisキー (キーA)
REDIS_KEY_READ  = "meridis2"    # 書き込むRedisキー (キーB)
CMD_VEL_GAIN = 1.0              # cmd_velのゲイン (0~1)
FLG_MAKE_OBS = 0                # 0: obsを作らない, 1: obsを作る(Meridis_managerを起動しておくこと)

MOT_START_FRAME = 200

# ダミーの関節名リスト
joint_names = [
    "c_chest", "c_head", "l_shoulder_pitch", "l_shoulder_roll", "l_arm_upper_to_l_elbow", "l_elbow_pitch",
    "r_shoulder_pitch", "r_shoulder_roll", "r_arm_upper_to_r_elbow", "r_elbow_pitch",
    "l_hip_yaw", "l_hip_roll", "l_thigh_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_thigh_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll"
]


# Joint mapping dictionary
joint_to_meridis = {
    # Base link
    "base_roll":        12,
    "base_pitch":       13,
    "base_yaw":         14,
    # Left leg
    "l_hip_yaw":        31,
    "l_hip_roll":       33,
    "l_thigh_pitch":    35,
    "l_knee_pitch":     37,
    "l_ankle_pitch":    39,
    "l_ankle_roll":     41,
    # Right leg
    "r_hip_yaw":        61,
    "r_hip_roll":       63,
    "r_thigh_pitch":    65,
    "r_knee_pitch":     67,
    "r_ankle_pitch":    69,
    "r_ankle_roll":     71
}

# XMLモデルファイルを読み込む
#model = mujoco.MjModel.from_xml_path('/opt/mujoco/model/humanoid/humanoid.xml')
#model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/roborecipe4_go2_with_motors2.xml')
mjc_model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/scene.xml')
mjc_data = mujoco.MjData(mjc_model)

# メインループ
# ビジュアライザの設定
def render_model(model, data):

    total_frames = 0

    window = glfw.create_window(800, 600, "URDF Viewer", None, None)
    glfw.init()
    glfw.make_context_current(window)
    glfw.swap_interval(0)
    
    # ビューワーの初期化
    opt = mujoco.MjvOption()  # オプションオブジェクトを作成
    
    cam = mujoco.MjvCamera()    
    cam.distance = 1.0  # カメラの距離を調整
    cam.azimuth = 180    # カメラの方位角を調整
    cam.elevation = -30   # カメラの仰角を調整
    cam.lookat = [0.2, 0.2, 0.4]  # カメラの注視点を調整
    scene = new_func(model)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    
    # perturbオブジェクトの作成（必要なければNoneでも可）
    pert = mujoco.MjvPerturb()
    
    # メインループ
    while not glfw.window_should_close(window):

        total_frames += 1
        print(f"Total frames: {total_frames}")

        viewport = mujoco.MjrRect(0, 0, 1200, 900)

        # make actions:データの更新
        # joint_pos, joint_names, base_euler を 90要素の配列に変換
        mdata = [0.0] * 90  # 初期化

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
                            mjc_data.ctrl[joint_idx] = amp * abs(math.sin(mot_ctrl))

                        mdata[meridis_index] = round(np.degrees(float(mjc_data.ctrl[joint_idx])), 2)

        #print(f"mdata: {mdata}")

        # Redis にデータを送信
        redis_transfer.set_data(REDIS_KEY_WRITE, mdata)

        # データの更新
        mujoco.mj_step(model, data)
        
        # シーンの更新 - pert引数を追加
        mujoco.mjv_updateScene(model, data, opt, pert, cam, mujoco.mjtCatBit.mjCAT_ALL, scene)
        
        # レンダリング
        mujoco.mjr_render(viewport, scene, context)
        
        # ウィンドウの更新
        glfw.swap_buffers(window)
        glfw.poll_events()
    
    glfw.terminate()

def new_func(model):
    scene = mujoco.MjvScene(model, maxgeom=10000)
    return scene

if __name__ == "__main__":
    redis_transfer = RedisTransfer(redis_key=REDIS_KEY_WRITE)
    redis_receiver = RedisReceiver(redis_key=REDIS_KEY_READ)

    # 標準ビューワ
    #launch(mjc_model, mjc_data)  # ビューアを更新し、終了イベントをチェック

    # レンダーを実行
    # グラフィカルインターフェースの初期化
    glfw.init()
    render_model(mjc_model, mjc_data)
