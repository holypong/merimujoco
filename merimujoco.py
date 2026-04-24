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
import logging

import math
import json
import base64
try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False
    logger = None  # logging設定前のフォールバック（後で上書きされる）

# Logger configuration
# level=logging.DEBUG       Detailed logs (development)
# level=logging.INFO        Normal info logs (production)
# level=logging.WARNING     Warning level and above
# level=logging.ERROR       Error level and above
logging.basicConfig(
    level=logging.INFO,  # For production
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger(__name__)

# macOSでMuJoCoビューアーを動作させるための環境変数設定
os.environ['MUJOCO_GL'] = 'glfw'

from redis_transfer import RedisTransfer
from redis_receiver import RedisReceiver

# Declare structures
from dataclasses import dataclass, field
from typing import List

MSG_SIZE = 90                   # Meridim配列の長さ
CMD_VEL_GAIN = 1.0              # cmd_velのゲイン (0~1)
FLG_SET_RCVD = True             # Redisからのデータ受信フラグ
FLG_CREATE_CTRL = False          # 制御信号作成フラグ
FLG_SET_SEND = True             # Redisへのデータ送信フラグ
FLG_RESET_REQUEST = False       # リセット要求フラグ
FLG_REDIS_TO_JOINT = True        # Redisから受信した値を関節にセットするフラグ
FLG_JOINT_TO_REDIS = False       # 関節角度をRedisに送信するフラグ

MASTER_CMD_RESET = 5556     # リセットコマンド番号
RESET_CMD_COOLDOWN_SEC = 0.3  # 連続リセット抑止の最短間隔

viewer = None  # MuJoCo viewer object

MOT_START_TIME = 1.0  # 開始時間
FOOT_CALIB_DELAY = 1.5  # Calibration delay (s)
USE_CONTACT_CALIB = True  # Trueのとき接地contact検出でキャリブ、FalseのときFOOT_CALIB_DELAY秒待機
ACTUATOR_FORCE_SCALE = 1.0  # 実機寄りにトルク感を上げる係数
FOOT_POS_DECIMALS = 4  # 足先・手先位置(m)の小数点桁数

# Redisサーバー設定（デフォルト値）
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
REDIS_KEY_READ = "meridis_calc_pub"
REDIS_KEY_WRITE = "meridis_sim_pub"

def load_redis_config(json_file: str ="redis.json"):
    """Load Redis configuration from a JSON file"""
    global REDIS_HOST, REDIS_PORT, REDIS_KEY_READ, REDIS_KEY_WRITE
    global FLG_REDIS_TO_JOINT, FLG_JOINT_TO_REDIS
    
    try:
        if not os.path.exists(json_file):
            logger.warning(f"Redis config file '{json_file}' not found. Using default values.")
            return False
        
        with open(json_file, 'r', encoding='utf-8') as f:
            config = json.load(f)
        
        # Load Redis connection settings
        if 'redis' in config:
            if 'host' in config['redis']:
                REDIS_HOST = config['redis']['host']
            if 'port' in config['redis']:
                REDIS_PORT = config['redis']['port']
        
        # Load Redis key settings
        if 'redis_keys' in config:
            if 'read' in config['redis_keys']:
                REDIS_KEY_READ = config['redis_keys']['read']
            if 'write' in config['redis_keys']:
                REDIS_KEY_WRITE = config['redis_keys']['write']
        
        # Load data flow settings
        if 'data_flow' in config:
            if 'redis_to_joint' in config['data_flow']:
                FLG_REDIS_TO_JOINT = config['data_flow']['redis_to_joint']
            if 'joint_to_redis' in config['data_flow']:
                FLG_JOINT_TO_REDIS = config['data_flow']['joint_to_redis']
        
        logger.info(f"Loaded Redis configuration from '{json_file}'")
        logger.info(f"Redis Server: {REDIS_HOST}:{REDIS_PORT}")
        logger.info(f"Redis Keys: Read='{REDIS_KEY_READ}', Write='{REDIS_KEY_WRITE}'")
        logger.info(f"Data Flow: Redis->Joint={FLG_REDIS_TO_JOINT}, Joint->Redis={FLG_JOINT_TO_REDIS}")
        logger.debug(f"redis: {config.get('redis', {})}")
        logger.debug(f"redis_keys: {config.get('redis_keys', {})}")
        logger.debug(f"data_flow: {config.get('data_flow', {})}")
        return True
        
    except json.JSONDecodeError as e:
        logger.error(f"Failed to parse JSON file '{json_file}': {e}")
        return False
    except Exception as e:
        logger.error(f"Failed to load Redis config from '{json_file}': {e}")
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

# Joint names list for 20260125 version
# Must match the actuator order in the XML file for data.ctrl indexing.
joint_names = [
    "c_chest", "c_head", "l_shoulder_pitch", "l_shoulder_roll", "l_elbow_yaw", "l_elbow_pitch",
    "r_shoulder_pitch", "r_shoulder_roll", "r_elbow_yaw", "r_elbow_pitch",
    "l_hip_yaw", "l_hip_roll", "l_thigh_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_thigh_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll"
]


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
parser.add_argument('--getfoot',
                    type=lambda x: x.lower() != 'false',
                    default=True,
                    metavar='BOOL',
                    help='Write foot positions to mdata (default: true)')
parser.add_argument('--gethand',
                    type=lambda x: x.lower() != 'false',
                    default=False,
                    metavar='BOOL',
                    help='Write hand positions to mdata[44-46,74-76] (default: false)')
parser.add_argument('--view',
                    type=str,
                    default=None,
                    metavar='MODE',
                    help='Camera view mode: fpv (first-person view from head center)')
parser.add_argument('--sphere',
                    type=str,
                    default=None,
                    metavar='X,Y,Z',
                    help='Touch sphere position in meters (e.g. --sphere 0.15,-0.05,0.35)') # 0.15,-0.05,0.1 で右手が到達可能な位置
parser.add_argument('--stream',
                    action='store_true',
                    default=False,
                    help='Enable offscreen FPV streaming to Redis key "meridis_frame_pub" via camera "head_fpv"')
args = parser.parse_args()

FLG_GET_FOOT = args.getfoot  # 両足位置書き込みフラグ
FLG_GET_HAND = args.gethand  # 両手位置書き込みフラグ
VIEW_MODE = args.view        # カメラビューモード
FLG_STREAM = args.stream     # FPVストリーミングフラグ

# --sphere オプションの解析
SPHERE_POS = None
if args.sphere:
    try:
        vals = [float(v) for v in args.sphere.split(',')]
        if len(vals) != 3:
            raise ValueError("3つの値が必要です")
        SPHERE_POS = vals
    except ValueError as e:
        print(f"[ERROR] --sphere の形式が不正です ({e})。例: --sphere 0.05,0.0,0.2")
        sys.exit(1)

# id80: 0=球なし, 1=球あり(bit0), 3=接触検知(bit0+bit1)
sphere_status = 1 if SPHERE_POS is not None else 0

# Redis設定の読み込み（data_flowの設定も含む）
load_redis_config(args.redis)

redis_transfer = RedisTransfer(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_WRITE)
redis_receiver = RedisReceiver(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_READ)

total_frames = 0    # 全体のフレーム数
elapsed = 0.0       # 経過時間
start_time = 0.0   # 開始時間（メインループで設定）
line_vel_x = 0.0    # 前進速度
line_vel_y = 0.0    # 左右速度
ang_vel_z = 0.0     # 旋回速度

# スレッドセーフのためのロック
imu_lock = threading.Lock()
sim_lock = threading.Lock()

# モデルを読み込む
model = mujoco.MjModel.from_xml_path('roid1_mjcf/scene.xml')
data = mujoco.MjData(model)

# FPVカメラID (XMLの head_fpv カメラを使用)
FPV_CAM_ID = -1
if VIEW_MODE == 'fpv':
    FPV_CAM_ID = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_fpv")
    if FPV_CAM_ID < 0:
        logger.warning("--view fpv: camera 'head_fpv' not found in model, falling back to default view")
        VIEW_MODE = None
    else:
        logger.info(f"--view fpv: using fixed camera 'head_fpv' (id={FPV_CAM_ID})")

# FPVオフスクリーンレンダラー (--stream オプション)
FPV_RENDERER = None
FPV_INTERVAL = 10   # 10ステップごとに配信 (10ms×10 = 100ms = 10fps)
FPV_REDIS_KEY = "meridis_frame_pub"
if FLG_STREAM:
    if not _CV2_AVAILABLE:
        logger.error("--stream requires opencv-python. Install with: pip install opencv-python")
        FLG_STREAM = False
    else:
        _stream_cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_fpv")
        if _stream_cam_id < 0:
            logger.error("--stream: camera 'head_fpv' not found in model. Streaming disabled.")
            FLG_STREAM = False
        else:
            FPV_RENDERER = mujoco.Renderer(model, height=240, width=320)
            logger.info(f"FPV streaming enabled: camera 'head_fpv' -> Redis key '{FPV_REDIS_KEY}' at ~10fps")


# タッチ検出球のgeom ID
TOUCH_SPHERE_GEOM_ID = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "touch_sphere")
touch_sphere_visible = TOUCH_SPHERE_GEOM_ID >= 0
if TOUCH_SPHERE_GEOM_ID < 0:
    logger.warning("touch_sphere geom not found in model")

# --sphere オプションで球の位置を上書き、省略時は非表示
SPHERE_BODY_ID = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "touch_sphere_body")
SPHERE_INITIAL_POS = model.body_pos[SPHERE_BODY_ID].copy() if SPHERE_BODY_ID >= 0 else None

if TOUCH_SPHERE_GEOM_ID >= 0:
    if SPHERE_POS is not None:
        if SPHERE_BODY_ID >= 0:
            model.body_pos[SPHERE_BODY_ID] = SPHERE_POS
            logger.info(f"touch_sphere position set to {SPHERE_POS}")
    else:
        model.geom_rgba[TOUCH_SPHERE_GEOM_ID, 3] = 0.0
        touch_sphere_visible = False
        logger.info("touch_sphere hidden (no --sphere option)")

# --- 起動時に強制的に物理パラメータを上書き ---
model.opt.gravity[:] = [0, 0, -9.8]           # 重力（地球標準）
model.opt.timestep = 0.001                    # タイムステップ（1ms）
model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4  # 安定な積分器
# 関節減衰を適度に設定（XMLのデフォルト0.1を維持）
model.dof_damping[:] = 0.5      # 小型ロボット 0.1-0.5
                                # 中型ロボット 0.5-2.0
                                # 大型ロボット 2.0-10.0
# 全geomの摩擦係数を上書き（静止摩擦、動摩擦、粘着摩擦）
model.geom_friction[:, :] = [1.2, 0.8, 0.01]  # 着地安定化用の摩擦調整
# 実機より弱くなりやすい出力上限を補正（position actuatorのforcelimitを拡大）
model.actuator_forcerange[:, 0] *= ACTUATOR_FORCE_SCALE
model.actuator_forcerange[:, 1] *= ACTUATOR_FORCE_SCALE
logger.info(f"Gravity: {model.opt.gravity}, Timestep: {model.opt.timestep}")
logger.info(f"Actuator force range scaled by x{ACTUATOR_FORCE_SCALE}")



# ビューアを初期化
logger.info(f"Detected OS: {platform.system()}")
logger.info(f"MUJOCO_GL environment variable: {os.environ.get('MUJOCO_GL', 'not set')}")

mdata = [0.0] * 90  # 初期化
imu_mjc = Imu(
    header=Header(stamp=0.0, frame_id="c_chest"),
    # orientation を roll/pitch/yaw(deg) として保持する
    orientation=Vector3(0.0, 0.0, 0.0),
    angular_velocity=Vector3(0.0, 0.0, 0.0),
    linear_acceleration=Vector3(0.0, 0.0, 0.0)
)

def motor_controller_thread():
    global imu_mjc, FLG_RESET_REQUEST, elapsed, total_frames, start_time, line_vel_x, line_vel_y, ang_vel_z, touch_sphere_visible, sphere_status
    _cnt_self = False  # Trueになると以降メリムジョコがカウンタを自己管理

    # リセットコマンドは立ち上がりエッジでのみ受理する。
    reset_cmd_latched = False
    last_reset_request_time = 0.0

    # chest_body_idを取得
    chest_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "c_chest")
    # foot_body_idを取得
    l_foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_foot")
    r_foot_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_foot")
    # 左右の手先body IDを取得
    l_hand_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_arm_lower")
    r_hand_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_arm_lower")

    # 手先端のローカルオフセットをcollision geom（カプセル）から取得
    # カプセルのZ軸先端 = geom_pos_z - half_length - radius
    def get_hand_tip_local_offset(hand_body_id):
        tip_local = np.zeros(3)
        min_z = 0.0
        for gid in range(model.ngeom):
            if model.geom_bodyid[gid] != hand_body_id:
                continue
            if model.geom_type[gid] not in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
                continue
            pos = model.geom_pos[gid]       # ローカル座標のgeom中心
            sz  = model.geom_size[gid]      # [radius, half_length, ...]
            tip_z = pos[2] - sz[1] - sz[0]  # 先端Z（下方向）
            if tip_z < min_z:
                min_z = tip_z
                tip_local = np.array([pos[0], pos[1], tip_z])
        return tip_local

    l_hand_tip_local = get_hand_tip_local_offset(l_hand_body_id)
    r_hand_tip_local = get_hand_tip_local_offset(r_hand_body_id)
    logger.info("Hand tip local offset: L=(%.4f, %.4f, %.4f), R=(%.4f, %.4f, %.4f)",
        l_hand_tip_local[0], l_hand_tip_local[1], l_hand_tip_local[2],
        r_hand_tip_local[0], r_hand_tip_local[1], r_hand_tip_local[2])
    # 左右の股関節ヨー軸中心（body原点）を取得
    l_hip_yaw_center_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "l_hipjoint_upper")
    r_hip_yaw_center_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "r_hipjoint_upper")

    # 足底面のZオフセット（body原点→足底面最下点）をコリジョンジオメトリから取得
    def get_foot_sole_z_offset(foot_body_id):
        min_z = 0.0
        for gid in range(model.ngeom):
            if model.geom_bodyid[gid] != foot_body_id:
                continue
            geom_type = model.geom_type[gid]
            pos_z = model.geom_pos[gid][2]
            sz = model.geom_size[gid]
            if geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                sole_z = pos_z - sz[2]
            elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                sole_z = pos_z - sz[0]
            elif geom_type in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
                sole_z = pos_z - sz[0]
            else:
                continue
            if sole_z < min_z:
                min_z = sole_z
        return min_z

    l_foot_sole_z = get_foot_sole_z_offset(l_foot_body_id)
    r_foot_sole_z = get_foot_sole_z_offset(r_foot_body_id)
    logger.info("Foot sole Z offset from body origin: L=%.6f m, R=%.6f m", l_foot_sole_z, r_foot_sole_z)

    # 足先Zオフセット：起動時の初期姿勢（地面接触状態）を0基準とする
    def capture_foot_offsets():
        with sim_lock:
            mujoco.mj_forward(model, data)
            xpos = np.array(data.xpos)
        l_hip = xpos[l_hip_yaw_center_body_id]
        r_hip = xpos[r_hip_yaw_center_body_id]

        logger.info("Foot XYZ orign captured. L=(%.6f, %.6f, %.6f), R=(%.6f, %.6f, %.6f)",
            xpos[l_foot_body_id][0], xpos[l_foot_body_id][1], xpos[l_foot_body_id][2],
            xpos[r_foot_body_id][0], xpos[r_foot_body_id][1], xpos[r_foot_body_id][2])


        l_off = np.array([
            xpos[l_foot_body_id][0] - l_hip[0],
            xpos[l_foot_body_id][1] - l_hip[1],
            xpos[l_foot_body_id][2] + l_foot_sole_z,
        ], dtype=float)
        r_off = np.array([
            xpos[r_foot_body_id][0] - r_hip[0],
            xpos[r_foot_body_id][1] - r_hip[1],
            xpos[r_foot_body_id][2] + r_foot_sole_z,
        ], dtype=float)
        logger.info("Foot XYZ offset captured. L=(%.6f, %.6f, %.6f), R=(%.6f, %.6f, %.6f)",
            l_off[0], l_off[1], l_off[2], r_off[0], r_off[1], r_off[2])
        return l_off, r_off

    # 足のgeomが床と接触しているか判定
    l_foot_geom_ids = [g for g in range(model.ngeom) if model.geom_bodyid[g] == l_foot_body_id]
    r_foot_geom_ids = [g for g in range(model.ngeom) if model.geom_bodyid[g] == r_foot_body_id]

    def is_foot_grounded(foot_geom_ids):
        for i in range(data.ncon):
            c = data.contact[i]
            if c.geom1 in foot_geom_ids or c.geom2 in foot_geom_ids:
                return True
        return False

    # 足先オフセットは起動・リセット後 FOOT_CALIB_DELAY 秒後に整定してからキャプチャする
    l_foot_offset_m = np.zeros(3)
    r_foot_offset_m = np.zeros(3)
    foot_offset_captured = False
    foot_calib_due_at = FOOT_CALIB_DELAY  # 次回キャプチャ予定時刻(elapsed基準)

    while True:
        # 時間を更新
        total_frames += 1
        elapsed = time.time() - start_time

        # 足先オフセットキャプチャ
        if not foot_offset_captured:
            if USE_CONTACT_CALIB:
                # contact検出方式: 両足が接地したタイミングでキャプチャ
                if is_foot_grounded(l_foot_geom_ids) and is_foot_grounded(r_foot_geom_ids):
                    logger.info("Both feet grounded (contact detected). Capturing foot offsets.")
                    l_foot_offset_m, r_foot_offset_m = capture_foot_offsets()
                    foot_offset_captured = True
            else:
                # 時間遅延方式: FOOT_CALIB_DELAY 秒経過してからキャプチャ
                if elapsed >= foot_calib_due_at:
                    l_foot_offset_m, r_foot_offset_m = capture_foot_offsets()
                    foot_offset_captured = True

        # リセット要求がある場合はリセットを実行
        if FLG_RESET_REQUEST:
            logger.info("Executing MuJoCo reset")
            with sim_lock:
                mujoco.mj_resetData(model, data)
                mujoco.mj_forward(model, data)
                # タッチ球をxyz位置に戻す
                if TOUCH_SPHERE_GEOM_ID >= 0 and SPHERE_POS is not None and SPHERE_BODY_ID >= 0:
                    model.body_pos[SPHERE_BODY_ID] = SPHERE_POS
            FLG_RESET_REQUEST = False
            touch_sphere_visible = SPHERE_POS is not None
            if SPHERE_POS is not None:
                sphere_status = 1  # bit1をクリア（接触フラグをリセット）
            foot_offset_captured = False
            foot_calib_due_at = elapsed + FOOT_CALIB_DELAY
            logger.info("MuJoCo reset completed")

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

                    is_reset_cmd = (rcv_data[0] == MASTER_CMD_RESET)

                    if is_reset_cmd and not reset_cmd_latched:
                        now = time.time()
                        if (now - last_reset_request_time) >= RESET_CMD_COOLDOWN_SEC:
                            # リセット要求フラグを立てる（メインループで実行）
                            logger.info(f"MuJoCo reset request received: {rcv_data[0]}")
                            FLG_RESET_REQUEST = True
                            last_reset_request_time = now
                        reset_cmd_latched = True
                    elif not is_reset_cmd:
                        # コマンド解除後に次のリセット要求を受け付ける
                        reset_cmd_latched = False

                        
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
                        with sim_lock:
                            for joint_name, meridis_index in joint_to_meridis.items():
                                if joint_name in joint_names:
                                    # Handle joint positions (convert from radians to degrees)
                                    joint_idx = joint_names.index(joint_name)
                                    meridis_idx = joint_to_meridis[joint_name][0]
                                    meridis_mul = joint_to_meridis[joint_name][1]
                                    data.ctrl[joint_idx] = round(np.radians(float(rcv_data[meridis_idx])*meridis_mul), 2)
                                    #print(f"joint_name: {joint_name}, joint_idx: {joint_idx}, ctrl: {data.ctrl[joint_idx]}, mul: {joint_to_meridis[joint_name][1]}")

                    # mdataの更新 +Hori 20250628
                    # mdata[1]の受信値が0の場合、以降は自己インクリメントモードにラッチ
                    if not _cnt_self and float(rcv_data[1]) == 0:
                        _cnt_self = True
                    for i in range(len(mdata)):
                        if i < len(rcv_data):
                            if i == 1 and _cnt_self:
                                continue  # メリムジョコが自己管理するため受信値を使わない
                            mdata[i] = float(rcv_data[i])
                        else:
                            mdata[i] = 0.0

                    #print(f"mdata: {mdata}")

            if FLG_CREATE_CTRL and elapsed >= MOT_START_TIME:  # 制御信号作成フラグが立っていて、開始時間を超えたら
                # make actions:データの更新

                with sim_lock:
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
            if FLG_SET_SEND and elapsed >= MOT_START_TIME:  # データ送信フラグが立っていて、開始時間を超えたら

                # --- c_chestのIMU計算 (Redis送信直前) ---
                # 姿勢
                with sim_lock:
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

                        # chest_mat: ワールド→c_chest の回転行列　を使って、ワールド座標系の角速度をc_chest座標系に変換
                        omega_world = np.array([wx, wy, wz])            # ワールド座標系の角速度
                        omega_local = chest_mat.T @ omega_world         # c_chest座標系の角速度
                        ang_vel = Vector3(math.degrees(float(omega_local[0])), math.degrees(float(omega_local[1])), math.degrees(float(omega_local[2])))
                    except Exception as e:
                        logger.error(f"Angular velocity error: {e}")
                        ang_vel = Vector3(0.0, 0.0, 0.0)
                    # 加速度（重力ベクトルをc_chest座標系へ変換）
                    try:
                        g = np.array(model.opt.gravity)  # shape=(3,)
                        # chest_mat: ワールド→c_chest の回転行列
                        # gはワールド座標系なので、c_chest座標系へは R^T @ g
                        lin_acc_arr = chest_mat.T @ g
                        lin_acc = Vector3(float(lin_acc_arr[0]), float(lin_acc_arr[1]), float(lin_acc_arr[2]))
                    except Exception as e:
                        logger.error(f"Gravity transform error: {e}")
                        lin_acc = Vector3(0.0, 0.0, 0.0)
                
                with imu_lock:
                    imu_mjc = Imu(
                        header=Header(stamp=time.time(), frame_id="c_chest"),
                        orientation=Vector3(roll_deg, pitch_deg, yaw_deg),
                        angular_velocity=ang_vel,
                        linear_acceleration=lin_acc
                    )

                # mujocoのIMUデータを小数点2桁で表示
                logger.debug("IMU: %.2f, %.2f, %.2f", imu_mjc.orientation.x, imu_mjc.orientation.y, imu_mjc.orientation.z)

                mdata[2] = round(imu_mjc.linear_acceleration.x, 4)   # ax(m/s^2)
                mdata[3] = round(imu_mjc.linear_acceleration.y, 4)   # ay(m/s^2)
                mdata[4] = round(imu_mjc.linear_acceleration.z, 4)   # az(m/s^2)
                mdata[5]  = round(imu_mjc.angular_velocity.x, 4)   # wx(deg/s)
                mdata[6]  = round(imu_mjc.angular_velocity.y, 4)   # wy(deg/s)
                mdata[7]  = round(imu_mjc.angular_velocity.z, 4)   # wz(deg/s)
                mdata[12] = round(imu_mjc.orientation.x, 4)   # roll(deg)
                mdata[13] = round(imu_mjc.orientation.y, 4)   # pitch(deg)
                mdata[14] = round(imu_mjc.orientation.z, 4)   # yaw(deg)

                # 足先XYZ位置をm単位で書き込む
                # X/Y: 左右それぞれの股関節ヨー軸中心からの相対位置, Z: 地面からの高さ(ワールドZ)
                with sim_lock:
                    xpos = np.array(data.xpos)  # shape=(nbody, 3)
                    l_hip_origin = xpos[l_hip_yaw_center_body_id]
                    r_hip_origin = xpos[r_hip_yaw_center_body_id]
                    l_foot_raw_m = np.array([
                        (xpos[l_foot_body_id][0] - l_hip_origin[0]),
                        (xpos[l_foot_body_id][1] - l_hip_origin[1])/2,  # 左右の股関節ヨー軸中心からの相対位置はY方向に半分に縮める（実機の脚幅を考慮して補正）
                        xpos[l_foot_body_id][2] + l_foot_sole_z,  # body原点 + 足底オフセット = 床からの足底高さ
                    ], dtype=float)
                    r_foot_raw_m = np.array([
                        (xpos[r_foot_body_id][0] - r_hip_origin[0]),
                        (xpos[r_foot_body_id][1] - r_hip_origin[1])/2,  # 左右の股関節ヨー軸中心からの相対位置はY方向に半分に縮める（実機の脚幅を考慮して補正）
                        xpos[r_foot_body_id][2] + r_foot_sole_z,  # body原点 + 足底オフセット = 床からの足底高さ
                    ], dtype=float)

                if foot_offset_captured and FLG_GET_FOOT:
                    l_foot_cal_m = l_foot_raw_m - l_foot_offset_m
                    r_foot_cal_m = r_foot_raw_m - r_foot_offset_m

                    mdata[47] = round(float(l_foot_cal_m[0]), FOOT_POS_DECIMALS)  # l_foot_x (m, 左股関節ヨー軸中心相対・ゼロ補正後)
                    mdata[48] = round(float(l_foot_cal_m[1]), FOOT_POS_DECIMALS)  # l_foot_y (m, 左股関節ヨー軸中心相対・ゼロ補正後)
                    mdata[49] = round(float(l_foot_cal_m[2]), FOOT_POS_DECIMALS)  # l_foot_z (m, 床面からの高さ・ゼロ補正後)
                    mdata[77] = round(float(r_foot_cal_m[0]), FOOT_POS_DECIMALS)  # r_foot_x (m, 右股関節ヨー軸中心相対・ゼロ補正後)
                    mdata[78] = round(float(r_foot_cal_m[1]), FOOT_POS_DECIMALS)  # r_foot_y (m, 右股関節ヨー軸中心相対・ゼロ補正後)
                    mdata[79] = round(float(r_foot_cal_m[2]), FOOT_POS_DECIMALS)  # r_foot_z (m, 床面からの高さ・ゼロ補正後)

                # 手先端XYZ位置をm単位で書き込む（カプセルgeom先端→ワールド座標変換）
                if FLG_GET_HAND:
                    with sim_lock:
                        xpos_w = np.array(data.xpos)   # shape=(nbody, 3)
                        xmat_w = np.array(data.xmat)   # shape=(nbody, 9) or (nbody*9,)
                    # 回転行列を取得
                    if xmat_w.ndim == 2 and xmat_w.shape[1] == 9:
                        l_rot = xmat_w[l_hand_body_id].reshape(3, 3)
                        r_rot = xmat_w[r_hand_body_id].reshape(3, 3)
                    else:
                        flat = xmat_w.flatten()
                        l_rot = flat[l_hand_body_id*9:(l_hand_body_id+1)*9].reshape(3, 3)
                        r_rot = flat[r_hand_body_id*9:(r_hand_body_id+1)*9].reshape(3, 3)
                    # body原点 + 回転行列 × ローカルオフセット = ワールド座標の手先端
                    l_tip_world = xpos_w[l_hand_body_id] + l_rot @ l_hand_tip_local
                    r_tip_world = xpos_w[r_hand_body_id] + r_rot @ r_hand_tip_local
                    mdata[44] = round(float(l_tip_world[0]), FOOT_POS_DECIMALS)  # l_hand_x (m)
                    mdata[45] = round(float(l_tip_world[1]), FOOT_POS_DECIMALS)  # l_hand_y (m)
                    mdata[46] = round(float(l_tip_world[2]), FOOT_POS_DECIMALS)  # l_hand_z (m)
                    mdata[74] = round(float(r_tip_world[0]), FOOT_POS_DECIMALS)  # r_hand_x (m)
                    mdata[75] = round(float(r_tip_world[1]), FOOT_POS_DECIMALS)  # r_hand_y (m)
                    mdata[76] = round(float(r_tip_world[2]), FOOT_POS_DECIMALS)  # r_hand_z (m)

                # FLG_JOINT_TO_REDISがTrueの場合、関節角度をRedisに送信
                if FLG_JOINT_TO_REDIS:
                    with sim_lock:
                        for joint_name, meridis_index in joint_to_meridis.items():
                            if joint_name in joint_names:
                                joint_idx = joint_names.index(joint_name)
                                meridis_idx = joint_to_meridis[joint_name][0]
                                meridis_mul = joint_to_meridis[joint_name][1]
                                # data.ctrl[joint_idx]はラジアン、度数に変換して格納（multiplierで元に戻す）
                                joint_angle_deg = math.degrees(data.ctrl[joint_idx]) / meridis_mul
                                mdata[meridis_idx] = round(joint_angle_deg, 4)
                
                #start_time = time.perf_counter()
                # mdata[1]カウンタ処理（Redis書き込み直前）
                # ・未受信 または 自己モードラッチ済み → 65535 まで累積インクリメント
                # ・受信値が0以外かつラッチなし     → 受信値をそのまま使用
                if (not rcv_data or _cnt_self) and int(mdata[1]) < 65535:
                    mdata[1] = int(mdata[1]) + 1
                # mdata[80-83]: 球状態を常に上書き（受信データより優先）
                mdata[80] = float(sphere_status)
                if SPHERE_POS is not None:
                    mdata[81] = float(SPHERE_POS[0])
                    mdata[82] = float(SPHERE_POS[1])
                    mdata[83] = float(SPHERE_POS[2])
                else:
                    mdata[81] = 0.0
                    mdata[82] = 0.0
                    mdata[83] = 0.0
                redis_transfer.set_data(REDIS_KEY_WRITE, mdata)

        time.sleep(0.01)  # 10ms待機

# スレッドを開始
start_time = time.time()
mot_ctrl_thread = threading.Thread(target=motor_controller_thread, daemon=True)
mot_ctrl_thread.start()

# シグナルハンドラを設定
def signal_handler(sig, frame):
    logger.info(f"Signal {sig} received. Exiting...")
    try:
        glfw.terminate()
    except:
        pass
    os._exit(0)

# SIGINT (Ctrl+C) とSIGTSTP (Ctrl+Z) のシグナルハンドラを設定
signal.signal(signal.SIGINT, signal_handler)
# SIGTSTPはUnix/Linux系でのみ利用可能（Windowsでは利用不可）
if hasattr(signal, 'SIGTSTP'):
    signal.signal(signal.SIGTSTP, signal_handler)

logger.info("Simulation started. Push [x]button or Select Menu [File -> Quit] to stop.")

# MuJoCoビューアーを起動（パッシブ実行）
# Note: フォントスケールはビューワーのUI内で手動調整可能です
logger.info("Launching MuJoCo viewer...")
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        if VIEW_MODE == 'fpv' and FPV_CAM_ID >= 0:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = FPV_CAM_ID
        prev_sim_time = 0.0
        _fpv_cnt = 0  # FPV配信カウンタ（メインスレッドで管理）
        while viewer.is_running():
            with sim_lock:
                mujoco.mj_step(model, data)
                # UIリセット検出: data.timeが減少したらリセットとみなす
                if data.time < prev_sim_time - model.opt.timestep:
                    if TOUCH_SPHERE_GEOM_ID >= 0 and SPHERE_POS is not None and SPHERE_BODY_ID >= 0:
                        model.body_pos[SPHERE_BODY_ID] = SPHERE_POS
                    touch_sphere_visible = SPHERE_POS is not None
                    if SPHERE_POS is not None:
                        sphere_status = 1  # bit1をクリア（システムリセット）
                    logger.info("touch_sphere: UI reset detected, moved sphere to xyz")
                prev_sim_time = data.time
                # タッチ球接触検出: ロボットが触れたら非表示
                if touch_sphere_visible:
                    for i in range(data.ncon):
                        c = data.contact[i]
                        if c.geom1 == TOUCH_SPHERE_GEOM_ID or c.geom2 == TOUCH_SPHERE_GEOM_ID:
                            if SPHERE_BODY_ID >= 0 and SPHERE_INITIAL_POS is not None:
                                model.body_pos[SPHERE_BODY_ID] = SPHERE_INITIAL_POS
                            sphere_status = 3  # bit1をセット（接触検知）
                            logger.info("touch_sphere: contact detected, moved to initial position")
                            break
                viewer.sync()

            # FPVオフスクリーンレンダリング＆Redis配信（メインスレッドで実行、WGLコンテキスト競合尋避）
            if FLG_STREAM and FPV_RENDERER is not None and redis_transfer.is_connected:
                _fpv_cnt += 1
                if _fpv_cnt >= FPV_INTERVAL:
                    _fpv_cnt = 0
                    try:
                        with sim_lock:
                            FPV_RENDERER.update_scene(data, camera="head_fpv")
                        frame_rgb = FPV_RENDERER.render()              # [H,W,3] uint8
                        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                        _, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        payload = json.dumps({
                            "count": int(mdata[1]),
                            "frame": base64.b64encode(buf).decode()
                        })
                        redis_transfer.redis_client.set(FPV_REDIS_KEY, payload)
                    except Exception as e:
                        logger.error(f"FPV stream error: {e}")

            time.sleep(model.opt.timestep)
except Exception as e:
    logger.exception(f"MuJoCo viewer loop error: {e}")

# ビューアー終了後にGLFWをクリーンアップ
try:
    glfw.terminate()
except:
    pass
