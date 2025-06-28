import redis
import numpy as np
import time

# 20250429 redis_transfer.py 新規作成

class RedisTransfer:
    def __init__(self, host='localhost', port=6379, redis_key='meridis'):
        self.redis_client = None
        self.meridis_size = 90
        self.is_connected = False
        self.redis_key = redis_key  # 使用するRedisキーを設定

        try:
            self.redis_client = redis.Redis(host=host, port=port, decode_responses=True)
            self.redis_client.ping()  # 接続テスト
            self.is_connected = True

            # Initialize redis_key if it doesn't exist
            if not self.redis_client.exists(self.redis_key):
                for i in range(self.meridis_size):                                   # ハッシュの場合
                    self.redis_client.hset(self.redis_key, i, 0)
                print(f"Initialized Redis list '{self.redis_key}' with {self.meridis_size} elements.")
            else:
                print(f"Redis list '{self.redis_key}' already exists.")

            # Joint mapping dictionary
            self.joint_to_meridis = {
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

        except redis.ConnectionError as e:
            print(f"could not connect to Redis server: {e}")
            print(f"continuing without Redis")
        except Exception as e:
            print(f"unexpected error: {e}")
            print(f"continuing without Redis")
            
    def transfer_joint_data(self, joint_positions, joint_names, base_euler=None, key=None):
        """Transfer joint position and base orientation data to Redis
        
        Args:
            joint_positions (numpy.ndarray): Array of joint positions
            joint_names (list): List of joint names corresponding to the positions
            base_euler (numpy.ndarray, optional): Base link euler angles (roll, pitch, yaw) in degrees
            key (str, optional): 使用するRedisキー。指定がなければインスタンス変数を使用
        """
        if not self.is_connected:
            return

        # 使用するキーを決定
        redis_key = key if key is not None else self.redis_key

        # キーが存在するか確認
        if not self.redis_client.exists(redis_key):
            # キーが存在しない場合はmeridis_size分の要素を初期化（ハッシュの場合）Hash値は配列の要素番号
            for i in range(self.meridis_size):
                self.redis_client.hset(redis_key, i, 0)
            print(f"Initialized Redis hash '{redis_key}' with {self.meridis_size} elements.")

        try:
            # Transfer joint positions
            for joint_name, meridis_index in self.joint_to_meridis.items():
                if joint_name.startswith('base_') and base_euler is not None:
                    # Handle base orientation (already in degrees)
                    if joint_name == "base_roll":
                        value = round(float(base_euler[0]), 2)
                    elif joint_name == "base_pitch":
                        value = round(float(base_euler[1]), 2)
                    elif joint_name == "base_yaw":
                        value = round(float(base_euler[2]), 2)
                    self.redis_client.hset(redis_key, meridis_index, value)
                elif joint_name in joint_names:
                    # Handle joint positions (convert from radians to degrees)
                    joint_idx = joint_names.index(joint_name)
                    value = round(np.degrees(float(joint_positions[joint_idx])), 2)
                    self.redis_client.hset(redis_key, meridis_index, value)    
                    
        except redis.RedisError as e:
            print(f"[Redis Error | set_data] Unexpected error: {str(e)}")        
            self.is_connected = False

    def set_data(self, key=None, data=None):
        """Redisにハッシュ構造でデータを保存する
        
        Args:
            key (str, optional): 使用するRedisキー。指定がなければインスタンス変数を使用
            data (list[float]): 保存する90要素のfloat型配列
        """
        if not self.is_connected:
            print("[Redis Error | set_data] Cound not connect to Redis server.")
            return

        if data is None or len(data) != 90:
            print("[Redis Error | set_data] wrong data format.")
            return

        # 使用するキーを決定
        redis_key = key if key is not None else self.redis_key

        try:    
            # データをハッシュ構造で保存
            #for i, value in enumerate(data):
            #    self.redis_client.hset(redis_key, i, value)
            
            # データをハッシュ構造で保存（1回の操作で全てを設定）
            hash_data = {str(i): value for i, value in enumerate(data)}
            self.redis_client.hset(redis_key, mapping=hash_data)

        except redis.RedisError as e:
            print(f"[Redis Error | set_data] Unexpected error: {str(e)}")        
    

    def close(self):
        """Close the Redis connection"""
        if self.is_connected and self.redis_client:
            try:
                self.redis_client.close()
            except:
                pass

def main():
    """テスト用のメイン関数"""
    import argparse
    import time
    
    parser = argparse.ArgumentParser(description='Redisに関節データを転送するテストプログラム')
    parser.add_argument('--host', default='localhost', help='Redisサーバーのホスト名')
    parser.add_argument('--port', type=int, default=6379, help='Redisサーバーのポート番号')
    parser.add_argument('--key', default='meridis', help='使用するRedisキー')
    args = parser.parse_args()
    
    # テスト用のダミーデータ（ライブラリとして呼ばれる場合は使わない）
    joint_positions = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    joint_names = ["l_hip_yaw", "l_hip_roll", "l_thigh_pitch", "r_hip_roll", "r_hip_yaw", "r_thigh_pitch"]
    base_euler = np.array([10.0, 20.0, 30.0])
    
    # RedisTransferのインスタンス化
    transfer = RedisTransfer(host=args.host, port=args.port, redis_key=args.key)
    
    try:
        print(f"Redisサーバー {args.host}:{args.port} にキー '{args.key}' でデータ転送を開始")
        
        # データを3回転送してみる
        for i in range(3):
            # 値を少し変更
            joint_positions = joint_positions + 0.1
            base_euler = base_euler + 5.0
            
            # 転送実行
            transfer.transfer_joint_data(joint_positions, joint_names, base_euler)
            print(f"転送 {i+1}: 完了")
            time.sleep(1)
                    
        print("完了しました。")
        
    except KeyboardInterrupt:
        print("\nユーザーによって停止されました")
    finally:
        transfer.close()


if __name__ == "__main__":
    main()