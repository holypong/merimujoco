import mujoco
import mujoco.viewer
import glfw
import numpy as np
import threading
import time
import platform
import os
import signal
import sys
import argparse

import numpy as np
import time
import math
import json

# macOSでMuJoCoビューアーを動作させるための環境変数設定
os.environ['MUJOCO_GL'] = 'glfw'

from redis_transfer import RedisTransfer
from redis_receiver import RedisReceiver

# 構造体を宣言する
from dataclasses import dataclass, field
from typing import List

MSG_SIZE = 90                   # Meridim配列の長さ
CMD_VEL_GAIN = 1.0              # cmd_velのゲイン (0~1)
FLG_SET_RCVD = True             # Redisからのデータ受信フラグ
FLG_CREATE_CTRL = False          # 制御信号作成フラグ
FLG_SET_SNDD = True             # Redisへのデータ送信フラグ
FLG_RESET_REQUEST = False       # リセット要求フラグ
FLG_REDIS_TO_JOINT = True        # Redisから受信した値を関節にセットするフラグ
FLG_JOINT_TO_REDIS = False       # 関節角度をRedisに送信するフラグ

viewer = None  # MuJoCo viewer object

MOT_START_FRAME = 200   # 開始フレーム
MOT_START_TIME = 1.0  # 開始時間

# Redisサーバー設定（デフォルト値）
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
REDIS_KEY_READ = "meridis2"
REDIS_KEY_WRITE = "meridis"

def load_redis_config(json_file="redis.json"):
    """Redis設定をJSONファイルから読み込む"""
    global REDIS_HOST, REDIS_PORT, REDIS_KEY_READ, REDIS_KEY_WRITE
    
    try:
        if not os.path.exists(json_file):
            print(f"[Warning] Redis config file '{json_file}' not found. Using default values.")
            return False
        
        with open(json_file, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        # Redis接続設定の読み込み
        if 'redis' in config:
            if 'host' in config['redis']:
                REDIS_HOST = config['redis']['host']
            if 'port' in config['redis']:
                REDIS_PORT = config['redis']['port']
        
        # Redisキーの設定を読み込み
        if 'redis_keys' in config:
            if 'read' in config['redis_keys']:
                REDIS_KEY_READ = config['redis_keys']['read']
            if 'write' in config['redis_keys']:
                REDIS_KEY_WRITE = config['redis_keys']['write']
        
        print(f"[Config] Loaded Redis configuration from '{json_file}'")
        print(f"[Config] Redis Server: {REDIS_HOST}:{REDIS_PORT}")
        print(f"[Config] Redis Keys: Read='{REDIS_KEY_READ}', Write='{REDIS_KEY_WRITE}'")
        print(f"[Debug] redis: {config.get('redis', {})}")
        print(f"[Debug] redis_keys: {config.get('redis_keys', {})}")
        return True
        
    except json.JSONDecodeError as e:
        print(f"[Error] Failed to parse JSON file '{json_file}': {e}")
        return False
    except Exception as e:
        print(f"[Error] Failed to load Redis config from '{json_file}': {e}")
        return False

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

# コマンドライン引数の解析
parser = argparse.ArgumentParser(description='MuJoCo simulation with Redis configuration')
parser.add_argument('--redis', 
                    type=str, 
                    default='redis.json',
                    help='Redis configuration JSON file (default: redis.json)')
parser.add_argument('--redis_to_joint',
                    type=str,
                    choices=['true', 'false'],
                    default='true',
                    help='Set MuJoCo joint angles from Redis (default: true)')
parser.add_argument('--joint_to_redis',
                    type=str,
                    choices=['true', 'false'],
                    default='false',
                    help='Set Redis from MuJoCo joint angles (default: false)')
args = parser.parse_args()

# redis_to_jointフラグを設定
FLG_REDIS_TO_JOINT = (args.redis_to_joint.lower() == 'true')
# joint_to_redisフラグを設定
FLG_JOINT_TO_REDIS = (args.joint_to_redis.lower() == 'true')

# Redis設定の読み込み
load_redis_config(args.redis)

redis_transfer = RedisTransfer(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_WRITE)
redis_receiver = RedisReceiver(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_READ)

total_frames = 0    # 全体のフレーム数
elapsed = 0.0       # 経過時間
start_time = 0.0    # 開始時間
line_vel_x = 0.0    # 前進速度
line_vel_y = 0.0    # 左右速度
ang_vel_z = 0.0     # 旋回速度

# スレッドセーフのためのロック
imu_lock = threading.Lock()

# モデルを読み込む
#model = mujoco.MjModel.from_xml_path('urdf/scene.xml')
model = mujoco.MjModel.from_xml_path('mjcf/scene.xml')
data = mujoco.MjData(model)


# --- 起動時に強制的に物理パラメータを上書き ---
model.opt.gravity[:] = [0, 0, -9.8]           # 重力
model.opt.timestep = 0.001                    # タイムステップ調整
model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4  # 安定な積分器に変更
# 全関節の減衰（damping）を強制上書き
model.dof_damping[:] = 5.0                    # ブレ防止
# 全geomの摩擦係数を上書き（静止摩擦、動摩擦、粘着摩擦）
#model.geom_friction[:, :] = [1.2, 0.8, 0.01]  # 着地安定化用の摩擦調整



# ビューアを初期化
print(f"[Info] Detected OS: {platform.system()}")
print(f"[Info] MUJOCO_GL environment variable: {os.environ.get('MUJOCO_GL', 'not set')}")

mdata = [0.0] * 90  # 初期化
imu_mjc = Imu(
    header=Header(stamp=0.0, frame_id="c_chest"),
    # orientation を roll/pitch/yaw(deg) として保持する
    orientation=Vector3(0.0, 0.0, 0.0),
    angular_velocity=Vector3(0.0, 0.0, 0.0),
    linear_acceleration=Vector3(0.0, 0.0, 0.0)
)

def motor_controller_thread():
    global imu_mjc, FLG_RESET_REQUEST, elapsed, total_frames, start_time, line_vel_x, line_vel_y, ang_vel_z

    # chest_body_idを取得
    chest_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "c_chest")

    while True:
        # 時間を更新
        total_frames += 1
        elapsed = time.time() - start_time

        # リセット要求がある場合はリセットを実行
        if FLG_RESET_REQUEST:
            print(f"[motor_controller_thread] executing mujoco reset")
            mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            FLG_RESET_REQUEST = False
            print(f"[motor_controller_thread] reset completed")

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

                    # FLG_REDIS_TO_JOINTがTrueの場合のみRedisから受信した値を関節にセット
                    if FLG_REDIS_TO_JOINT:
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

                # --- c_chestのIMU計算 (Redis送信直前) ---
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
                    print(f"[motor_controller_thread] 角速度取得エラー: {e}")
                    ang_vel = Vector3(0.0, 0.0, 0.0)
                # 加速度（重力ベクトルをc_chest座標系へ変換）
                try:
                    g = np.array(model.opt.gravity)  # shape=(3,)
                    # chest_mat: ワールド→c_chest の回転行列
                    # gはワールド座標系なので、c_chest座標系へは R^T @ g
                    lin_acc_arr = chest_mat.T @ g
                    lin_acc = Vector3(float(lin_acc_arr[0]), float(lin_acc_arr[1]), float(lin_acc_arr[2]))
                except Exception as e:
                    print(f"[motor_controller_thread] 重力変換エラー: {e}")
                    lin_acc = Vector3(0.0, 0.0, 0.0)
                
                with imu_lock:
                    imu_mjc = Imu(
                        header=Header(stamp=time.time(), frame_id="c_chest"),
                        orientation=Vector3(roll_deg, pitch_deg, yaw_deg),
                        angular_velocity=ang_vel,
                        linear_acceleration=lin_acc
                    )

                # mujocoのIMUデータを小数点2桁で表示
                print(f"[Debug] mjc: {imu_mjc.orientation.x:.2f}, {imu_mjc.orientation.y:.2f}, {imu_mjc.orientation.z:.2f}")

                mdata[2] = round(imu_mjc.linear_acceleration.x, 4)   # ax(m/s^2)
                mdata[3] = round(imu_mjc.linear_acceleration.y, 4)   # ay(m/s^2)
                mdata[4] = round(imu_mjc.linear_acceleration.z, 4)   # az(m/s^2)
                mdata[5]  = round(imu_mjc.angular_velocity.x, 4)   # wx(deg/s)
                mdata[6]  = round(imu_mjc.angular_velocity.y, 4)   # wy(deg/s)
                mdata[7]  = round(imu_mjc.angular_velocity.z, 4)   # wz(deg/s)
                mdata[12] = round(imu_mjc.orientation.x, 4)   # roll(deg)
                mdata[13] = round(imu_mjc.orientation.y, 4)   # pitch(deg)
                mdata[14] = round(imu_mjc.orientation.z, 4)   # yaw(deg)
                
                # FLG_JOINT_TO_REDISがTrueの場合、関節角度をRedisに送信
                if FLG_JOINT_TO_REDIS:
                    for joint_name, meridis_index in joint_to_meridis.items():
                        if joint_name in joint_names:
                            joint_idx = joint_names.index(joint_name)
                            meridis_idx = joint_to_meridis[joint_name][0]
                            meridis_mul = joint_to_meridis[joint_name][1]
                            # data.ctrl[joint_idx]はラジアン、度数に変換して格納（multiplierで元に戻す）
                            joint_angle_deg = math.degrees(data.ctrl[joint_idx]) / meridis_mul
                            mdata[meridis_idx] = round(joint_angle_deg, 4)
                
                #start_time = time.perf_counter()
                redis_transfer.set_data(REDIS_KEY_WRITE, mdata)
                #elapsed_time = time.perf_counter() - start_time
                #print(f"transfer elapsed time: {elapsed_time*1000000:.2f} microseconds ({elapsed_time:.6f} seconds)")


        time.sleep(0.01)  # 10ms待機

# スレッドを開始
mot_ctrl_thread = threading.Thread(target=motor_controller_thread, daemon=True)
mot_ctrl_thread.start()

# シグナルハンドラを設定
def signal_handler(sig, frame):
    print(f"\n[Info] Signal {sig} received. Exiting...")
    try:
        glfw.terminate()
    except:
        pass
    os._exit(0)

# SIGINT (Ctrl+C) とSIGTSTP (Ctrl+Z) のシグナルハンドラを設定
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTSTP, signal_handler)

# メインループで制御＋mj_step
start_time = time.time()

print("[Info] Simulation started. Press Esc, Ctrl+C, or Ctrl+Z to stop.")

# MuJoCoビューアーを起動（ブロッキング実行）
print("[Info] Launching MuJoCo viewer...")
mujoco.viewer.launch(model, data)

# ビューアー終了後にGLFWをクリーンアップ
try:
    glfw.terminate()
except:
    pass


