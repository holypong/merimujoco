import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
from redis_receiver import RedisReceiver
import argparse
from collections import deque
# Don't forget to add this import at the top
import time

# 20250429 redis_plotter.py 新規作成

class RedisPlotter:
    def __init__(self, receiver, fig_width=10, fig_height=6):
        self.receiver = receiver
        
        # Joint mapping dictionary moved from redis_receiver.py
        self.joint_to_meridis = {
            # Base link
            "base_roll": 12,
            "base_pitch": 13,
            "base_yaw": 14,
            # Left leg
            "l_hip_roll": 33,
            "l_hip_yaw": 34,
            "l_thigh_pitch": 35,
            "l_knee_pitch": 37,
            "l_ankle_pitch": 39,
            "l_ankle_roll": 41,
            # Right leg
            "r_hip_roll": 63,
            "r_hip_yaw": 64,
            "r_thigh_pitch": 65,
            "r_knee_pitch": 67,
            "r_ankle_pitch": 69,
            "r_ankle_roll": 71
        }
        
        # Initialize joint data storage
        history_length = self.receiver.get_history_length()
        self.joint_data = {joint: deque(maxlen=history_length) for joint in self.joint_to_meridis.keys()}
        
        # Set up the plot
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(fig_width, fig_height))
        gs = self.fig.add_gridspec(3, 1, height_ratios=[1, 1, 1])
        self.axes = [self.fig.add_subplot(gs[i]) for i in range(3)]
        
        self.fig.suptitle('Joint Angles Over Time', y=0.95)
        
        # Get joint groups
        base_joints, left_joints, right_joints = self.get_joint_groups()
        
        # Base link plot
        self.base_lines = {}
        self.axes[0].set_title('Base Link')
        self.axes[0].set_xlabel('Time (s)')
        self.axes[0].set_ylabel('Angle (degrees)')
        self.axes[0].grid(True, alpha=0.3)
        
        # Left leg plot
        self.left_lines = {}
        self.axes[1].set_title('Left Leg Joints')
        self.axes[1].set_xlabel('Time (s)')
        self.axes[1].set_ylabel('Angle (degrees)')
        self.axes[1].grid(True, alpha=0.3)
        
        # Right leg plot
        self.right_lines = {}
        self.axes[2].set_title('Right Leg Joints')
        self.axes[2].set_xlabel('Time (s)')
        self.axes[2].set_ylabel('Angle (degrees)')
        self.axes[2].grid(True, alpha=0.3)

        # Set fixed y-axis limits
        for ax in self.axes:
            ax.set_ylim(-180, 180)

        # Initialize plot lines with different colors
        # Base joint colors (using a different colormap for base)
        base_colors = plt.cm.Set1(np.linspace(0, 1, len(base_joints)))
        for i, joint in enumerate(base_joints):
            line, = self.axes[0].plot([], [], label=joint, color=base_colors[i], linewidth=2)
            self.base_lines[joint] = line
        
        # Leg joint colors
        leg_colors = plt.cm.rainbow(np.linspace(0, 1, 6))
        for i, joint in enumerate(left_joints):
            line, = self.axes[1].plot([], [], label=joint, color=leg_colors[i], linewidth=2)
            self.left_lines[joint] = line
            
        for i, joint in enumerate(right_joints):
            line, = self.axes[2].plot([], [], label=joint, color=leg_colors[i], linewidth=2)
            self.right_lines[joint] = line

        # Add legends with smaller font
        for ax in self.axes:
            ax.legend(loc='upper right', framealpha=0.8, fontsize='small')

        # Store all lines in a list for animation
        self.all_lines = (list(self.base_lines.values()) + 
                         list(self.left_lines.values()) + 
                         list(self.right_lines.values()))

        # Adjust layout with custom parameters
        plt.tight_layout(rect=[0.05, 0.05, 0.95, 0.95], h_pad=1.5)

    # Moved from redis_receiver.py: get_joint_groups() function
    def get_joint_groups(self):
        base_joints = ["base_roll", "base_pitch", "base_yaw"]
        left_joints = [j for j in self.joint_to_meridis.keys() if j.startswith('l_')]
        right_joints = [j for j in self.joint_to_meridis.keys() if j.startswith('r_')]
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
        # Redisからデータを更新
        data = self.receiver.get_data()
        
        # genesiseのデータ取得
        joint_data = self.get_joint_data_series(data)

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
                blit=True
            )
            plt.show()
        except KeyboardInterrupt:
            print("\nPlotting stopped by user")
        finally:
            plt.close()


def main():
    # コマンドライン引数の処理
    parser = argparse.ArgumentParser()
    parser.add_argument('--width', type=int, default=8, help='Figure width in inches')
    parser.add_argument('--height', type=int, default=10, help='Figure height in inches')
    parser.add_argument('--window', type=float, default=5.0, help='Time window size in seconds')
    args = parser.parse_args()

    # RedisReceiverのインスタンス化
    receiver = RedisReceiver(window_size=args.window, redis_key='meridis')
    
    # RedisPlotterのインスタンス化と実行
    plotter = RedisPlotter(receiver, fig_width=args.width, fig_height=args.height)
    plotter.run()
    
    # 終了時にReceiverも閉じる
    receiver.close()


if __name__ == "__main__":
    main()

