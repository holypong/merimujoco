import redis
import numpy as np
import sys
import time
import socket

# 20250429 redis_transfer.py 新規作成
# 20260102 エラー処理強化

REDIS_HOST = 'localhost'
REDIS_PORT = 6379
REDIS_KEY = 'meridis_calc_pub'

class RedisTransfer:
    def __init__(self, host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY,
                 connect_timeout: float = 0.5, socket_timeout: float = 0.5):
        self.redis_client = None
        self.meridis_size = 90
        self.is_connected = False
        self.redis_key = redis_key  # 使用するRedisキーを設定
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout

        try:
            # Configure short timeouts so unreachable Redis fails fast
            self.redis_client = redis.Redis(host=host, port=port, decode_responses=True,
                                            socket_connect_timeout=connect_timeout,
                                            socket_timeout=socket_timeout)

            # Quick TCP-level connect to fail fast on unreachable hosts
            try:
                conn = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
                conn.close()
            except Exception:
                raise redis.ConnectionError("TCP connect failed")

            self.redis_client.ping()  # 接続テスト (uses socket timeouts)
            self.is_connected = True

            # Initialize redis_key if it doesn't exist
            if not self.redis_client.exists(self.redis_key):
                for i in range(self.meridis_size):                                   # ハッシュの場合
                    self.redis_client.hset(self.redis_key, str(i), "0")  # キーを文字列に変換
                print(f"Initialized Redis list '{self.redis_key}' with {self.meridis_size} elements.")
            else:
                print(f"Redis list '{self.redis_key}' already exists.")

        except redis.ConnectionError as e:
            print(f"could not connect to Redis server: {e}")
            print(f"continuing without Redis")
        except Exception as e:
            print(f"unexpected error: {e}")
            print(f"continuing without Redis")
            
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
            # データをハッシュ構造で保存（パフォーマンスとバージョン互換性を両立）
            # Ensure values are plain numeric strings to avoid storing numpy reprs like 'np.float64(22.92)'
            hash_data = {str(i): str(float(value)) for i, value in enumerate(data)}
            
            try:
                # 新しいRedis版（Redis >= 4.0）を試行（高速）
                self.redis_client.hset(redis_key, mapping=hash_data)
            except (TypeError, redis.RedisError):
                # 古いRedis版にフォールバック（pipelineで高速化）
                pipe = self.redis_client.pipeline()
                for i, value in enumerate(data):
                    pipe.hset(redis_key, str(i), str(float(value)))
                pipe.execute()

        except redis.RedisError as e:
            print(f"[Redis Error | set_data] Unexpected error: {str(e)}")        
    

    # Check connection to Redis server (no key check)
    def check_connection(self):
        # Quick TCP connect first to fail fast
        try:
            conn = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
            conn.close()
        except Exception:
            return False

        try:
            return bool(self.redis_client.ping())
        except redis.exceptions.ConnectionError:
            return False
        except Exception as e:
            print(f"[Redis Error | check_connection] Unexpected error: {e}")
            return False

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
    
    parser = argparse.ArgumentParser(description='Test program to transfer joint data to Redis')
    parser.add_argument('--host', default=REDIS_HOST, help='Redis server hostname')
    parser.add_argument('--port', type=int, default=REDIS_PORT, help='Redis server port')
    parser.add_argument('--key', default=REDIS_KEY, help='Redis key to use')
    args = parser.parse_args()
    
    # RedisTransferのインスタンス化
    transfer = RedisTransfer(host=args.host, port=args.port, redis_key=args.key)
    # Verify Redis server is reachable on startup (no key validation)
    if not transfer.check_connection():
        print(f"[Redis Error] Could not connect to Redis at {args.host}:{args.port}")
        transfer.close()
        sys.exit(1)
    
    try:
        print(f"Starting data transfer to Redis server {args.host}:{args.port} with key '{args.key}'")
        
            # Transfer data 3 times as a test
        # Prepare base dummy data: 90 elements, values 0.1, 0.2, 0.3, ... (start at 0.1)
        base_data = [round((i + 1) / 10.0, 1) for i in range(transfer.meridis_size)]

        for i in range(3):
            # shift values slightly each iteration
            #full_data = [round(val + 0.1 * i, 1) for val in base_data]
            full_data = base_data  # Reset to base data for consistent testing

            # Use set_data() convenience method to write the hash
            transfer.set_data(key=args.key, data=full_data)
            print(f"Wrote 90-element hash to '{args.key}' (iteration {i+1})")
            time.sleep(1)
                    
        print("Completed.")
        
    except KeyboardInterrupt:
        print("\nStopped by user (KeyboardInterrupt)")
    finally:
        transfer.close()


if __name__ == "__main__":
    main()