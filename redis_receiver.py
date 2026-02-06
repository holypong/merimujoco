import redis
import time
import socket
import sys
from collections import deque
import numpy as np
import argparse

# 20250429 redis_receiver.py 新規作成
# 20260102 エラー処理強化

REDIS_HOST = 'localhost'
REDIS_PORT = 6379
REDIS_KEY = 'meridis_calc_pub'

# RedisReceiverクラス
class RedisReceiver:
    def __init__(self, host=REDIS_HOST, port=REDIS_PORT, window_size=5.0, redis_key=REDIS_KEY,
                 connect_timeout: float = 0.5, socket_timeout: float = 0.5):
        # Save host/port and use short timeouts so ping/checks return quickly when Redis is unreachable
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        # Use short timeouts so ping/checks return quickly when Redis is unreachable
        self.redis_client = redis.Redis(host=host, port=port, decode_responses=True,
                                        socket_connect_timeout=connect_timeout,
                                        socket_timeout=socket_timeout)
        self.window_size = window_size  # 表示する時間幅（秒）
        self.history_length = int(window_size * 200)  # バッファサイズ
        self.redis_key = redis_key  # Redisから取得するキー
        
        # Initialize data storage
        self.time_data = deque(maxlen=self.history_length)
        self.start_time = time.time()

    # Check connection to Redis server and optionally verify key has data
    def check_connection(self, key=None):
        """Quick connection check. Returns True if Redis responds to PING and (optionally) the
        given key contains data. Uses short socket timeouts configured in __init__.
        """
        # First do a quick TCP connect to fail fast on unreachable hosts
        try:
            conn = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
            conn.close()
        except Exception:
            return False

        try:
            # ping() will raise on timeout/connection error because of short timeouts
            if not self.redis_client.ping():
                return False
        except Exception:
            return False

        if key:
            try:
                data = self.redis_client.hgetall(key)
                return bool(data)
            except Exception:
                return False

        return True

    # Redisからデータを取得する
    def get_data(self, key=None):
        # 使用するキーを決定
        redis_key = key if key is not None else self.redis_key
        sorted_data = None

        try:
            # ハッシュ構造のデータを取得
            data = self.redis_client.hgetall(redis_key)
            #print(f"[Debug] Raw data from Redis: {data}")

            if not data:
                print(f"[Redis Error | get_data] No data found for key '{redis_key}'.")
                return

            try:
                # ハッシュの値をfloatに変換し、キー順にソートしてリスト化
                sorted_data = [float(data[str(i)]) for i in range(len(data))]
                #print(f"[Debug] Converted float data: {sorted_data}")
            except (ValueError, KeyError) as e:
                print(f"[Redis Error | get_data] Invalid data format or missing keys: {e}")
                return
        
        except redis.ConnectionError:
            print("[Redis Error | get_data] Could not connect to Redis server.")
        except Exception as e:
            print(f"[Redis Error | get_data] Unexpected error: {str(e)}")        
    
        # 現在時刻を更新
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        return sorted_data
        
    # 接続を閉じる関数
    def close(self):
        self.redis_client.close()
    
    # 時間データを取得する
    def get_time_data(self):
        return self.time_data
    
    # ウィンドウサイズを取得する
    def get_window_size(self):
        return self.window_size
    
    # 開始時間を取得する
    def get_start_time(self):
        return self.start_time
    
    # 履歴長を取得する
    def get_history_length(self):
        return self.history_length

# メイン関数
def main():
    parser = argparse.ArgumentParser(description='Retrieve joint data from Redis')
    parser.add_argument('--host', default=REDIS_HOST, help='Redis server hostname')
    parser.add_argument('--port', type=int, default=REDIS_PORT, help='Redis server port')
    parser.add_argument('--key', default=REDIS_KEY, help='Redis key to retrieve')
    parser.add_argument('--window', type=float, default=5.0, help='Time window to display (seconds)')
    args = parser.parse_args()
    
    receiver = RedisReceiver(host=args.host, port=args.port, window_size=args.window, redis_key=args.key)
    
    print(f"Redis server {args.host}:{args.port} - starting data retrieval for key '{args.key}'")
    # Verify Redis connection and that the target key has data at startup
    if not receiver.check_connection(args.key):
        print(f"[Redis Error] Could not connect to Redis or key '{args.key}' has no data at {args.host}:{args.port}")
        receiver.close()
        sys.exit(1)
    
    try:
        for _ in range(10):
            data = receiver.get_data()  # Redisからデータを取得(str x 90)
            if data:
                print(f"Retrieved data: {len(data)} elements")
                print(f"Data: {data}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped by user (KeyboardInterrupt)")
    finally:
        receiver.close()


if __name__ == "__main__":
    main()