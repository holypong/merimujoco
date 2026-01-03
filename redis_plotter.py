import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from redis_receiver import RedisReceiver
import argparse
from collections import deque
import json
import os
import sys
from pathlib import Path
import time
from matplotlib.widgets import Button

# 20250429 redis_plotter.py 新規作成
# 20260102 json対応
# Redisサーバー設定
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379

def load_network_config(filename: str = "network.json"):
    """Load Redis host/port from network.json located next to this script.
    Returns (host, port) and falls back to defaults on any error.
    """
    base = Path(__file__).parent
    path = base / filename
    if not path.exists():
        print(f"Error: required network config not found: {path}", file=sys.stderr)
        sys.exit(1)
    with path.open("r", encoding="utf-8") as f:
        cfg = json.load(f)
    redis_cfg = cfg.get("redis", {})
    host = redis_cfg.get("host", REDIS_HOST)
    port = redis_cfg.get("port", REDIS_PORT)
    return host, int(port)


# Load effective Redis settings
REDIS_HOST, REDIS_PORT = load_network_config()

def get_local_ip():
    """自身のIPアドレスを取得する"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        IP = s.getsockname()[0]
        s.close()
        return IP
    except Exception as e:
        return "Error: " + str(e)

class RedisPlotter:
    def __init__(self, receiver, fig_width=10, fig_height=5, enable_log=False, display_mode='joint'):
        self.receiver = receiver
        self.enable_log = enable_log
        self.display_mode = display_mode
        
        # Joint mapping dictionary moved from redis_receiver.py
        self.joint_to_meridis = {
            # Base link
            "imu_temp":         11,
            "imu_roll":         12,
            "imu_pitch":        13,
            "imu_yaw":          14,
            # Left leg
            "l_hip_yaw":        31,
            "l_hip_roll":       33,
            "l_thigh_pitch":    35,
            "l_knee_pitch":     37,
            "l_ankle_pitch":    39,
            "l_ankle_roll":     41,
            # left foot
            "l_foot_x":         47,
            "l_foot_y":         48,
            "l_foot_z":         49,
            # Right leg
            "r_hip_yaw":        61,
            "r_hip_roll":       63,
            "r_thigh_pitch":    65,
            "r_knee_pitch":     67,
            "r_ankle_pitch":    69,
            "r_ankle_roll":     71,
            # right foot
            "r_foot_x":         77,
            "r_foot_y":         78,
            "r_foot_z":         79
            
        }
        
        # Initialize joint data storage
        history_length = self.receiver.get_history_length()
        self.joint_data = {joint: deque(maxlen=history_length) for joint in self.joint_to_meridis.keys()}
        
        # Animation control
        self.animation_paused = False
        
        # Set up the plot
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(fig_width, fig_height))
        gs = self.fig.add_gridspec(3, 1, height_ratios=[1, 1, 1])  # 元の3分割に戻す
        self.axes = [self.fig.add_subplot(gs[i]) for i in range(3)]
        
        # Setup display based on mode
        if self.display_mode == 'foot':
            self._setup_foot_display()
        else:  # joint mode (default)
            self._setup_joint_display()

        # Adjust layout with custom parameters - 下部余白を削減
        plt.tight_layout(rect=[0.05, 0.02, 0.95, 0.95], h_pad=1.2)
        
        # ボタンの設定
        self._setup_control_buttons()

    def _setup_joint_display(self):
        """Setup display for joint mode (original behavior)"""
        # タイトルをウィンドウタイトルバーに表示
        self.fig.canvas.manager.set_window_title('Joint Angles Over Time')
        
        # Get joint groups
        base_joints, left_joints, right_joints = self.get_joint_groups()
        
        # Base link plot
        self.base_lines = {}
        self.axes[0].set_title('Base Link')
        self.axes[0].set_xlabel('Time (s)')
        self.axes[0].set_ylabel('Angle (degrees)')
        self.axes[0].grid(True, alpha=0.3)
        
        # Right leg plot (2枠目)
        self.right_lines = {}
        self.axes[1].set_title('Right Leg Joints')
        self.axes[1].set_xlabel('Time (s)')
        self.axes[1].set_ylabel('Angle (degrees)')
        self.axes[1].grid(True, alpha=0.3)
        
        # Left leg plot (3枠目)
        self.left_lines = {}
        self.axes[2].set_title('Left Leg Joints')
        self.axes[2].set_xlabel('Time (s)')
        self.axes[2].set_ylabel('Angle (degrees)')
        self.axes[2].grid(True, alpha=0.3)

        # Set fixed y-axis limits
        self.axes[0].set_ylim(-10, 10)  # imu_tempを考慮して上限を60に変更
        self.axes[1].set_ylim(-90, 90)
        self.axes[2].set_ylim(-90, 90)

        # Initialize plot lines with different colors
        # Base joint colors (custom colors for specific joints)
        base_color_map = {
            "imu_temp":     "purple",
            "imu_roll":     "red",
            "imu_pitch":    "lime", 
            "imu_yaw":      "cyan"
        }
        for joint in base_joints:
            color = base_color_map.get(joint, "white")  # デフォルトは白
            line, = self.axes[0].plot([], [], label=joint, color=color, linewidth=2)
            self.base_lines[joint] = line
        
        # Leg joint colors
        leg_colors = plt.cm.rainbow(np.linspace(0, 1, 6))
        for i, joint in enumerate(right_joints):
            line, = self.axes[1].plot([], [], label=joint, color=leg_colors[i], linewidth=2)
            self.right_lines[joint] = line
            
        for i, joint in enumerate(left_joints):
            line, = self.axes[2].plot([], [], label=joint, color=leg_colors[i], linewidth=2)
            self.left_lines[joint] = line

        # Add legends with smaller font
        for ax in self.axes:
            ax.legend(loc='upper left', framealpha=0.8, fontsize='small')

        # Store all lines in a list for animation
        self.all_lines = (list(self.base_lines.values()) + 
                         list(self.left_lines.values()) + 
                         list(self.right_lines.values()))

    def _setup_foot_display(self):
        """Setup display for foot mode"""
        # タイトルをウィンドウタイトルバーに表示
        self.fig.canvas.manager.set_window_title('Foot Position Over Time')
        
        # Base link plot (same as joint mode)
        base_joints, _, _ = self.get_joint_groups()
        self.base_lines = {}
        self.axes[0].set_title('Base Link')
        self.axes[0].set_xlabel('Time (s)')
        self.axes[0].set_ylabel('Angle (degrees)')
        self.axes[0].grid(True, alpha=0.3)
        self.axes[0].set_ylim(-10, 10)  # imu_tempを考慮して上限を60に変更

        # Base joint colors
        base_color_map = {
            "imu_temp":     "purple",
            "imu_roll":     "red",
            "imu_pitch":    "lime", 
            "imu_yaw":      "cyan"
        }
        for joint in base_joints:
            color = base_color_map.get(joint, "white")
            line, = self.axes[0].plot([], [], label=joint, color=color, linewidth=2)
            self.base_lines[joint] = line
        
        # Right foot plot (2枠目)
        self.right_lines = {}
        self.axes[1].set_title('Right Foot Position')
        self.axes[1].set_xlabel('Time (s)')
        self.axes[1].set_ylabel('Position')
        self.axes[1].grid(True, alpha=0.3)
        self.axes[1].set_ylim(-100.0, 100.0)  # Adjust based on expected foot position range
        
        right_foot_joints = ["r_foot_x", "r_foot_y", "r_foot_z"]
        foot_colors = ["red", "green", "blue"]
        for i, joint in enumerate(right_foot_joints):
            line, = self.axes[1].plot([], [], label=joint, color=foot_colors[i], linewidth=2)
            self.right_lines[joint] = line
        
        # Left foot plot (3枠目)
        self.left_lines = {}
        self.axes[2].set_title('Left Foot Position')
        self.axes[2].set_xlabel('Time (s)')
        self.axes[2].set_ylabel('Position')
        self.axes[2].grid(True, alpha=0.3)
        self.axes[2].set_ylim(-100.0, 100.0)  # Adjust based on expected foot position range
        
        left_foot_joints = ["l_foot_x", "l_foot_y", "l_foot_z"]
        for i, joint in enumerate(left_foot_joints):
            line, = self.axes[2].plot([], [], label=joint, color=foot_colors[i], linewidth=2)
            self.left_lines[joint] = line

        # Add legends with smaller font
        for ax in self.axes:
            ax.legend(loc='upper left', framealpha=0.8, fontsize='small')

        # Store all lines in a list for animation
        self.all_lines = (list(self.base_lines.values()) + 
                         list(self.left_lines.values()) + 
                         list(self.right_lines.values()))

    def _setup_control_buttons(self):
        """コントロールボタンの設定"""
        # 1つのボタンで切り替える方式
        button_pos = [0.02, 0.96, 0.08, 0.03]  # [left, bottom, width, height]
        control_ax = plt.axes(button_pos)
        
        # コントロールボタンを作成（初期状態はPause）
        self.control_button = Button(control_ax, 'Pause', color='blue', hovercolor='darkblue')
        
        # ボタンのイベントハンドラを設定
        self.control_button.on_clicked(self._toggle_animation)

    def _toggle_animation(self, event):
        """アニメーションの停止・再開を切り替え"""
        if self.animation_paused:
            # 現在停止中 → 再開
            self.animation_paused = False
            self.control_button.label.set_text('Pause')
            self.control_button.color = 'blue'
            self.control_button.hovercolor = 'darkblue'
        else:
            # 現在実行中 → 停止
            self.animation_paused = True
            self.control_button.label.set_text('Resume')
            self.control_button.color = 'green'
            self.control_button.hovercolor = 'darkgreen'
        
        # ボタンの背景色を更新
        self.control_button.ax.set_facecolor(self.control_button.color)
        plt.draw()

    # Moved from redis_receiver.py: get_joint_groups() function
    def get_joint_groups(self):
        #base_joints = ["imu_temp", "imu_roll", "imu_pitch", "imu_yaw"]
        base_joints = ["imu_roll", "imu_pitch", "imu_yaw","imu_temp"]
        left_joints = [j for j in self.joint_to_meridis.keys() if j.startswith('l_') and not j.startswith('l_foot')]
        right_joints = [j for j in self.joint_to_meridis.keys() if j.startswith('r_') and not j.startswith('r_foot')]
        return base_joints, left_joints, right_joints

    # Moved from redis_receiver.py: get_joint_data_series() function
    def get_joint_data_series(self, data):
        if not data:
            return self.joint_data
            
        # Get all values from Redis
        for joint_name, meridis_index in self.joint_to_meridis.items():
            try:
                # インデックスが範囲内かチェックしつつアクセス
                if 0 <= meridis_index < len(data):
                    value = data[meridis_index]
                    value = float(value) if value is not None else 0.0
                else:
                    value = 0.0
                self.joint_data[joint_name].append(value)
            except (ValueError, TypeError):
                self.joint_data[joint_name].append(0.0)

        return self.joint_data

    # Moved from redis_receiver.py: get_visible_data_series() function
    def get_visible_data_series(self):
        time_data = self.receiver.get_time_data()
        window_size = self.receiver.get_window_size()
        
        # ウィンドウの範囲を設定
        current_time = time.time() - self.receiver.get_start_time()
        window_end = current_time
        window_start = window_end - window_size
        
        # 時間データを配列に変換
        time_array = np.array(list(time_data))
        
        # ウィンドウ内のデータを示すマスクを作成
        mask = (time_array >= window_start) & (time_array <= window_end)
        
        # 表示データ用の辞書を初期化
        visible_data = {
            'time': time_array[mask] if any(mask) else np.array([]),
            'window': (window_start, window_end),
            'joints': {}
        }
        
        # 各関節データを処理
        for joint_name, data_queue in self.joint_data.items():
            # キューからデータを配列に変換
            joint_array = np.array(list(data_queue))
            
            # データが十分にある場合はマスクを適用
            if len(joint_array) == len(time_array):
                visible_data['joints'][joint_name] = joint_array[mask] if any(mask) else np.array([])
            # データ長が時間データより短い場合は、利用可能なデータの最後の部分を使用
            elif len(joint_array) > 0:
                # 取得できるデータの中から最新のデータを使用
                data_length = min(sum(mask), len(joint_array))
                visible_data['joints'][joint_name] = joint_array[-data_length:] if data_length > 0 else np.array([])
            else:
                visible_data['joints'][joint_name] = np.array([])
        
        return visible_data

    def update_plot(self, frame):
        # アニメーションが停止中の場合は何もしない
        if self.animation_paused:
            return self.all_lines
            
        # Redisからデータを更新
        data = self.receiver.get_data()
        
        # genesiseのデータ取得
        joint_data = self.get_joint_data_series(data)

        # ログが有効な場合、ハッシュデータ全体（0-89）をカンマ区切りで出力
        if self.enable_log and data is not None:
            data_length = len(data)
            if data_length >= 90:  # インデックス0-89まで安全にアクセスするため90以上必要
                # インデックス0-89のデータをカンマ区切りで出力
                hash_values = [f"{data[i]:.2f}" for i in range(90)]
                print(f"[Frame {frame}] " + ",".join(hash_values))
            elif data_length > 0:
                # データが90未満の場合は利用可能な範囲で出力
                hash_values = [f"{data[i]:.2f}" for i in range(data_length)]
                print(f"[Frame {frame}] (partial {data_length}/90) " + ",".join(hash_values))
            else:
                # データが0要素の場合は警告を出力
                print(f"[Frame {frame}] Warning: Data length is {data_length}, expected at least 90")
        elif self.enable_log:
            # データがNoneの場合の警告
            print(f"[Frame {frame}] No data available from Redis")

        # 表示可能なデータを取得
        plot_data = self.get_visible_data_series()
        
        # 時間軸の範囲を更新
        window_start, window_end = plot_data['window']
        for ax in self.axes:
            ax.set_xlim(window_start, window_end)
        
        # 各関節データを更新
        time_array = plot_data['time']
        
        # Update base link lines
        for joint, line in self.base_lines.items():
            joint_data = plot_data['joints'][joint]
            line.set_data(time_array, joint_data)
        
        # Update left leg joint lines
        for joint, line in self.left_lines.items():
            joint_data = plot_data['joints'][joint]
            line.set_data(time_array, joint_data)
        
        # Update right leg joint lines
        for joint, line in self.right_lines.items():
            joint_data = plot_data['joints'][joint]
            line.set_data(time_array, joint_data)

        return self.all_lines

    def run(self, interval=10):
        """Run the animation with specified interval (in milliseconds)"""
        try:
            ani = animation.FuncAnimation(
                self.fig, 
                self.update_plot, 
                interval=interval,
                blit=False,  # blittingを無効にしてTkinterエラーを回避
                cache_frame_data=False  # フレームキャッシュを無効にして警告を回避
            )
            plt.show()
        except KeyboardInterrupt:
            print("\nPlotting stopped by user")
        finally:
            print("Redis Plotter shutting down...")
            plt.close()


def main():
    # コマンドライン引数の処理
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, default=8, help='Figure width in inches')
    parser.add_argument('--height', type=int, default=9, help='Figure height in inches')
    parser.add_argument('--window', type=float, default=5.0, help='Time window size in seconds')
    parser.add_argument('--log', type=str, default='off', help='Enable logging of base joint values (on/off)')
    parser.add_argument('--redis-key', type=str, default='meridis', help='Redis key name to read data from(default:meridis)')
    parser.add_argument('--display', type=str, default='joint', help='Display mode: joint (all joint angles) or foot (foot positions)')
    args = parser.parse_args()

    # 起動時の情報表示（meridis_manager.pyと同じスタイル）
    print(f"Redis Plotter started. This PC's IP address is {get_local_ip()}")
    print(f"Connected to Redis at {REDIS_HOST}:{REDIS_PORT} for key '{args.redis_key}'")
    print(f"Plot window: {args.window} seconds, Figure size: {args.width*100}x{args.height*100} pixels")
    print(f"Log output: {'enabled' if args.log.lower() == 'on' else 'disabled'}")
    print(f"Display mode: {args.display}")
    print(f"Animation interval: 10ms")
    
    # RedisReceiverのインスタンス化（IPアドレスを指定）
    receiver = RedisReceiver(host=REDIS_HOST, window_size=args.window, redis_key=args.redis_key)
    
    # ログ機能の有効/無効を判定
    enable_log = args.log.lower() == 'on'
    
    # RedisPlotterのインスタンス化と実行
    plotter = RedisPlotter(receiver, fig_width=args.width, fig_height=args.height, enable_log=enable_log, display_mode=args.display)
    print("Starting real-time plotting...")
    plotter.run()
    
    # 終了時にReceiverも閉じる
    receiver.close()
    print("Redis Plotter resources released.")


if __name__ == "__main__":
    main()

