import mujoco
from mujoco.viewer import launch_passive
import numpy as np
import threading
import time
import math

# 構造体を宣言する
from dataclasses import dataclass, field
from typing import List

MSG_SIZE = 90                   # Meridim配列の長さ
FLG_RESET_REQUEST = False       # リセット要求フラグ

MOT_START_FRAME = 200   # 開始フレーム
MOT_START_TIME = 1.0    # 開始時間

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
    angular_velocity: Vector3 = field(default_factory=lambda: Vector3(0.0, 0.0, 0.0))
    linear_acceleration: Vector3 = field(default_factory=lambda: Vector3(0.0, 0.0, 0.0))
    angular_velocity_covariance: List[float] = field(default_factory=lambda: [0.0]*9)

# ========== 歩行制御用のパラメータとクラス（mcp-meridis.mdから抽出） ==========

def param_field(default, description, type_):
    return field(default=default, metadata={"description": description, "type": type_})

@dataclass
class WalkParams:
    base_height: float = param_field(0.29, "ロボットの初期配置高さ[m]", "float")
    phase_offset: float = param_field(np.pi, "左右の足の位相差[rad]", "float")
    init_wait_time: float = param_field(2.0, "初期待機時間[秒]", "float")
    end_of_simulation: float = param_field(8.0, "シミュレーション終了時間[秒]", "float")
    mix_on: bool = param_field(False, "ロール角を足首に反映するか", "bool")
    cycle_duration: float = param_field(1.8, "1周期の時間[秒]", "float")
    foot_lift: float = param_field(0.014, "遊脚の持ち上げ量[m]", "float")
    hip_swing: float = param_field(0.022, "横方向のスイング量[m]", "float")
    forward_stride: float = param_field(0.02, "前後方向の歩幅[m]", "float")
    swing_duration: float = param_field(2.0 * np.pi / 2.0, "遊脚期間の長さ[rad]", "float")
    duration: float = param_field(10.0, "動作期間[秒]", "float")

@dataclass
class LinkParams:
    HIP_YAW_TO_ROLL_OFFSET: float = param_field(0.07, "ヨー軸とロール軸の間のオフセット[m]", "float")
    THIGH_LENGTH: float = param_field(0.065, "太ももの長さ[m]", "float")
    SHANK_LENGTH: float = param_field(0.065, "すねの長さ[m]", "float")
    ANKLE_LENGTH: float = param_field(0.040, "足首から足首ロールまでの長さ[m]", "float")
    ANKLE_TO_FOOT: float = param_field(0.05, "足首ロール軸から足裏までの距離[m]", "float")
    SHORTEN_LEG_LENGTH: float = param_field(0.02, "短縮時の脚長[m]", "float")

    @property
    def TOTAL_LEG_LENGTH(self):
        return self.THIGH_LENGTH + self.SHANK_LENGTH + self.ANKLE_LENGTH

    @property
    def LINK_LEG_LENGTH(self):
        return self.THIGH_LENGTH + self.SHANK_LENGTH

params = WalkParams()
params_link = LinkParams()

# ステータス
TRQ_ON = 1.0            # サーボパワー 0:OFF, 1:ON
IDLE = 0
WALK = 1
MOT_STS = WALK          # 起動直後から歩行開始
MOT_INTERVAL = 0.008    # 8ms待ち
w_sts = 0

# 歩行ループ用の変数
t = 0
foot_ref_pitch = np.radians(0.0)  # 基準となる足首ピッチ角

# 内部配列（Redisの代わり）
mdata = [0.0] * MSG_SIZE  # 制御データ配列

stop_background = False

def calculate_foot_height(phase, step_height):
    """
    UVCスタイルの遊脚高さを計算する関数
    Args:
        phase: 歩行周期の位相 (0-2π)
        step_height: 最大足上げ高さ
    Returns:
        foot_height: 足の高さ
    """
    # 0-2πの範囲に正規化
    normalized_phase = ((phase % (2 * np.pi)) + 2 * np.pi) % (2 * np.pi)
    
    # 遊脚期間の定義
    swing_start = np.pi - params.swing_duration / 2
    swing_end = np.pi + params.swing_duration / 2

    if swing_start <= normalized_phase <= swing_end:
        # 遊脚期：正弦波で上下動作
        swing_phase = (normalized_phase - swing_start) / params.swing_duration
        return step_height * np.sin(swing_phase * np.pi)
    else:
        # 支持脚期：完全に接地
        return 0.0

def calculate_forward_motion(phase, step_length):
    """
    前後方向の移動量を計算する関数
    Args:
        phase: 歩行周期の位相 (0-2π)
        step_length: 歩幅 [m]
    Returns:
        x_pos: 前後方向の位置
    """
    # 単純な正弦波で前後移動を生成
    return step_length * np.sin(phase)

def geometric_leg_ik(target_pos, target_roll=None, target_pitch=None, is_left=True):
    """
    改善版の幾何学的な逆運動学を解く関数
    """
    # 脚の長さをチェックと制限
    leg_length = np.linalg.norm(target_pos)
    if leg_length > params_link.TOTAL_LEG_LENGTH:
        print("Leg length is too long. Terminate the simulation.")
        return None

    # 実際の脚長を計算（XY平面での投影）
    actual_leg_length = np.sqrt(target_pos[1]**2 + target_pos[2]**2)
    
    # Step 1: hip_rollの計算（脚長考慮版）
    hip_roll = np.arctan2(target_pos[1], actual_leg_length)
    
    # 可動範囲制限（±45度）
    MAX_ROLL = np.pi/4
    hip_roll = np.clip(hip_roll, -MAX_ROLL, MAX_ROLL)
    
    # Step 2: 矢状面での解法
    xz_proj_length = np.sqrt(target_pos[0]**2 + target_pos[2]**2)

    # 余弦定理で膝関節角度を計算
    cos_knee = (params_link.THIGH_LENGTH**2 + params_link.SHANK_LENGTH**2 - xz_proj_length**2) / \
               (2 * params_link.THIGH_LENGTH * params_link.SHANK_LENGTH)
    knee_pitch = np.pi - np.arccos(np.clip(cos_knee, -1.0, 1.0))
    
    if knee_pitch == 0.0:
        print("Knee pitch is zero. Terminate the simulation.")
        return None

    # 脚の各関節角度を計算
    # 地面に対して足裏が平行になるように計算
    leg_angle = np.arctan2(target_pos[0], target_pos[2])  # 脚全体の傾き
    thigh_pitch = leg_angle - knee_pitch / 2.0
    ankle_pitch = -(leg_angle + knee_pitch / 2.0)  # 地面に平行になるように補正

    # ankle_rollの計算（基本は逆位相）
    ankle_roll = -hip_roll

    # 追加の姿勢補正
    if target_roll is not None:
        hip_roll_correction = target_roll * 0.5  # 補正を両関節で分担
        hip_roll += hip_roll_correction
        ankle_roll -= hip_roll_correction
        
        # 補正後も可動範囲内に収める
        hip_roll = np.clip(hip_roll, -MAX_ROLL, MAX_ROLL)
        ankle_roll = np.clip(ankle_roll, -MAX_ROLL, MAX_ROLL)

    if target_pitch is not None:
        ankle_pitch += target_pitch

    # hip_yaw
    hip_yaw = 0.0    
        
    return np.array([hip_yaw, hip_roll, thigh_pitch, knee_pitch, ankle_pitch, ankle_roll])


# ========== MuJoCo関連のセットアップ ==========

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

# Enemy robot joint mapping
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

total_frames = 0    # 全体のフレーム数
elapsed = 0.0       # 経過時間

# モデルを読み込む
model = mujoco.MjModel.from_xml_path('/home/hori/mujoco/urdf/scene_vs.xml')
data = mujoco.MjData(model)

# --- 起動時に強制的に物理パラメータを上書き ---
model.opt.gravity[:] = [0, 0, -9.8]
model.opt.timestep = 0.001
model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
model.dof_damping[:] = 5.0
model.geom_friction[:, :] = [1.2, 0.8, 0.01]

# --- enemyロボットの質量調整 ---
for body_id in range(model.nbody):
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    if body_name and body_name.startswith("enemy_"):
        model.body_mass[body_id] *= 1.00
        model.body_inertia[body_id] *= 1.00
        model.body_ipos[body_id][2] += 0.00
        print(f"[Setup] Adjusted mass for {body_name}: {model.body_mass[body_id]:.4f} kg")

# ビューアを初期化（描画は別スレッドで自動）
viewer = launch_passive(model, data)

imu_mjc = Imu(
    header=Header(stamp=0.0, frame_id="c_chest"),
    orientation=Vector3(0.0, 0.0, 0.0),
    angular_velocity=Vector3(0.0, 0.0, 0.0),
    linear_acceleration=Vector3(0.0, 0.0, 0.0)
)

# ========== 歩行制御スレッド ==========

def walking_controller_thread():
    """歩行制御を行うバックグラウンドスレッド"""
    global MOT_STS, stop_background, mdata, t, w_sts
    
    start_time = time.time()
    
    while not stop_background:
        if MOT_STS == WALK:
            # 経過時間を計算
            t = time.time() - start_time

            # duration指定があれば、経過時間がdurationを超えたら自動でstop
            if params.duration and t >= params.duration:
                MOT_STS = IDLE
                print(f"[Walking] Duration reached. Stopping walk.")
                continue

            # 歩行状態の更新
            if t < params.init_wait_time:
                w_sts = 0
            elif t >= params.init_wait_time and t < (params.init_wait_time + params.cycle_duration * 0.25):
                w_sts = 1
            elif t >= (params.init_wait_time + params.cycle_duration * 0.25):
                w_sts = 2

            if w_sts == 0:
                # 初期位置
                l_target_pos = np.array([0.0, 0.0, (params_link.LINK_LEG_LENGTH - params_link.SHORTEN_LEG_LENGTH)])
                r_target_pos = np.array([0.0, 0.0, (params_link.LINK_LEG_LENGTH - params_link.SHORTEN_LEG_LENGTH)])

                l_joint_angles = geometric_leg_ik(l_target_pos, is_left=True)
                r_joint_angles = geometric_leg_ik(r_target_pos, is_left=False)

            else:
                # 歩行動作の開始
                phase_y = 2 * np.pi * ((t - params.init_wait_time) / params.cycle_duration)
                lateral_swing = params.hip_swing * np.sin(phase_y)
                phase_z = 2 * np.pi * ((t - (params.init_wait_time + params.cycle_duration * 0.25)) / params.cycle_duration)

                if w_sts == 1:
                    l_foot_swing = 0
                    r_foot_swing = 0
                    l_forward = 0
                    r_forward = 0
                else:
                    # 両足を上下に連動させる
                    l_foot_swing = calculate_foot_height(phase_z, params.foot_lift)
                    r_foot_swing = calculate_foot_height(phase_z + params.phase_offset, params.foot_lift)

                    # 前後方向の移動を正弦波で生成（左右逆位相）
                    l_forward = calculate_forward_motion(phase_z, params.forward_stride)
                    r_forward = calculate_forward_motion(phase_z + params.phase_offset, params.forward_stride)

                # 目標位置の計算と更新
                l_target_pos = np.array([l_forward, 0.0, (params_link.LINK_LEG_LENGTH - params_link.SHORTEN_LEG_LENGTH)])
                r_target_pos = np.array([r_forward, 0.0, (params_link.LINK_LEG_LENGTH - params_link.SHORTEN_LEG_LENGTH)])

                # 足の位置更新
                l_target_pos[2] = l_target_pos[2] - l_foot_swing
                l_target_pos[1] = l_target_pos[1] - lateral_swing
                r_target_pos[2] = r_target_pos[2] - r_foot_swing
                r_target_pos[1] = r_target_pos[1] + lateral_swing

                # 姿勢補正の計算(ミキシング処理は省略)
                l_target_roll = 0.0
                r_target_roll = 0.0

                # IKの計算
                l_joint_angles = geometric_leg_ik(l_target_pos, target_pitch=foot_ref_pitch,
                                                target_roll=l_target_roll, is_left=True)
                r_joint_angles = geometric_leg_ik(r_target_pos, target_pitch=foot_ref_pitch,
                                                target_roll=r_target_roll, is_left=False)

            # mdataに書き込み（関節角度をdegreeで格納）
            if l_joint_angles is not None and r_joint_angles is not None:
                for i in range(6):
                    mdata[30+2*i] = float(TRQ_ON)
                    mdata[31+2*i] = float(np.degrees(l_joint_angles[i]))
                    mdata[60+2*i] = float(TRQ_ON)
                    mdata[61+2*i] = float(np.degrees(r_joint_angles[i]))

            # デバッグ情報の表示
            print(f"Time: {t:.2f}, State: {w_sts}, L_knee: {mdata[37]:.2f}, R_knee: {mdata[67]:.2f}")

        # 指定間隔で待機
        time.sleep(MOT_INTERVAL)


def motor_controller_thread():
    """mdataをMuJoCoのdata.ctrlに反映するスレッド"""
    global imu_mjc, FLG_RESET_REQUEST
    
    while not stop_background:
        if elapsed >= MOT_START_TIME:
            # mdataから関節角度を読み取ってMuJoCoのctrlに反映
            for joint_name, meridis_info in joint_to_meridis.items():
                if joint_name in joint_names:
                    joint_idx = joint_names.index(joint_name)
                    meridis_idx = meridis_info[0]
                    meridis_mul = meridis_info[1]
                    # mdataの値は度数、data.ctrlはラジアン
                    data.ctrl[joint_idx] = np.radians(float(mdata[meridis_idx]) * meridis_mul)

            # Update enemy robot joints
            for enemy_joint_name, meridis_info in enemy_joint_to_meridis.items():
                if enemy_joint_name in enemy_joint_names:
                    enemy_joint_idx = enemy_joint_names.index(enemy_joint_name)
                    enemy_ctrl_idx = len(joint_names) + enemy_joint_idx
                    meridis_idx = meridis_info[0]
                    meridis_mul = meridis_info[1]
                    data.ctrl[enemy_ctrl_idx] = np.radians(float(mdata[meridis_idx]) * meridis_mul)

        time.sleep(0.01)  # 10ms待機


# スレッドを開始
walking_thread = threading.Thread(target=walking_controller_thread, daemon=True)
walking_thread.start()

motor_thread = threading.Thread(target=motor_controller_thread, daemon=True)
motor_thread.start()

# ========== メインループ ==========

start_time = time.time()
chest_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "c_chest")

print("[Main] Starting walking simulation. Press Ctrl+C to stop.")

while viewer.is_running():
    total_frames += 1
    elapsed = time.time() - start_time

    mujoco.mj_step(model, data)  # ステップ更新
    viewer.sync()                # 描画更新

    # --- c_chestのIMU計算（mj_step直後）---
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
        g = np.array(model.opt.gravity)
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

    # IMUデータをmdataに格納（必要に応じて）
    mdata[2] = round(imu_mjc.linear_acceleration.x, 4)   # ax(m/s^2)
    mdata[3] = round(imu_mjc.linear_acceleration.y, 4)   # ay(m/s^2)
    mdata[4] = round(imu_mjc.linear_acceleration.z, 4)   # az(m/s^2)
    mdata[5] = round(imu_mjc.angular_velocity.x, 4)      # wx(deg/s)
    mdata[6] = round(imu_mjc.angular_velocity.y, 4)      # wy(deg/s)
    mdata[7] = round(imu_mjc.angular_velocity.z, 4)      # wz(deg/s)
    mdata[12] = round(imu_mjc.orientation.x, 4)          # roll(deg)
    mdata[13] = round(imu_mjc.orientation.y, 4)          # pitch(deg)
    mdata[14] = round(imu_mjc.orientation.z, 4)          # yaw(deg)

print("[Main] Simulation ended.")
stop_background = True
