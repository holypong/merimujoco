import redis
import socket

# RedisTransferクラスをインポート（redis_transfer.pyから）
# 簡略版をここに定義するか、インポートする
# ここでは簡略版を定義

class RedisTransfer:
    def __init__(self, host, port=6379, redis_key='meridis',
                 connect_timeout: float = 0.5, socket_timeout: float = 0.5):
        self.redis_client = None
        self.meridis_size = 90
        self.is_connected = False
        self.redis_key = redis_key
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout

        try:
            self.redis_client = redis.Redis(host=host, port=port, decode_responses=True,
                                            socket_connect_timeout=connect_timeout,
                                            socket_timeout=socket_timeout)

            # TCP接続テスト
            conn = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
            conn.close()

            self.redis_client.ping()
            self.is_connected = True

            # キーが存在しない場合初期化
            if not self.redis_client.exists(self.redis_key):
                for i in range(self.meridis_size):
                    self.redis_client.hset(self.redis_key, i, 0)
                print(f"Initialized Redis key '{self.redis_key}' with {self.meridis_size} elements.")
            else:
                print(f"Redis key '{self.redis_key}' already exists.")

        except redis.ConnectionError as e:
            print(f"Could not connect to Redis server: {e}")
            self.is_connected = False
        except Exception as e:
            print(f"Unexpected error: {e}")
            self.is_connected = False

def main():
    # RedisサーバーのIPアドレスを尋ねる
    redis_host = input("Enter Redis server IP address: ").strip()
    if not redis_host:
        print("No IP address provided. Exiting.")
        return

    # 作成するキー
    keys = ['meridis_sim_pub', 'meridis_calc_pub', 'meridis_console_pub', 'meridis_mgr_pub', 'meridis_ai_pub']

    for key in keys:
        print(f"Creating key: {key}")
        transfer = RedisTransfer(host=redis_host, redis_key=key)
        if not transfer.is_connected:
            print(f"Failed to connect for key {key}. Skipping.")
        transfer.redis_client.close() if transfer.redis_client else None

    print("All keys created.")

if __name__ == "__main__":
    main()