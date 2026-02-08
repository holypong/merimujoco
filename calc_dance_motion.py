import numpy as np
import time
import argparse
import sys
from redis_transfer import RedisTransfer

# 20260208 calc_dance_motion.py 新規作成
# ロボットの全身をサインカーブで動かすダンスモーションをRedisに送信

REDIS_HOST = 'localhost'
REDIS_PORT = 6379
REDIS_KEY = 'meridis_calc_pub'  # 計算結果を書き込むキー

class DanceMotion:
    def __init__(self, host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY):
        """ダンスモーション生成クラス
        
        Args:
            host (str): Redisサーバーのホスト名
            port (int): Redisサーバーのポート番号
            redis_key (str): データを書き込むRedisキー
        """
        self.redis_transfer = RedisTransfer(host=host, port=port, redis_key=redis_key)
        self.x = 0.0  # サインカーブ用の変数
        self.meridis_motion = [0.0] * 90  # 90要素のモーションデータ
        
    def calculate_motion(self):
        """サインカーブで全身をくねらせるダンスモーションを計算"""
        # 初期化
        self.meridis_motion = [0.0] * 90
        
        # サインカーブで全身をくねらせる様にダンス
        self.meridis_motion[21] = -int(np.sin(self.x) * 30)          # 頭ヨー
        self.meridis_motion[51] = int(np.sin(self.x) * 20)         # 腰ヨー

        self.meridis_motion[23] = int(np.sin(self.x) * 20)     # 左肩ピッチ
        self.meridis_motion[25] = -int(np.sin(self.x * 2) * 10) + 30  # 左肩ロール
        self.meridis_motion[27] = int(np.sin(self.x) * 10) + 10     # 左肘ヨー
        self.meridis_motion[29] = int(np.sin(self.x) * 45) - 45     # 左肘ピッチ
        self.meridis_motion[31] = int(np.sin(self.x) * 5)           # 左股ヨー

        self.meridis_motion[53] = -int(np.sin(self.x) * 20)    # 右肩ピッチ
        self.meridis_motion[55] = -int(np.sin(self.x * 2) * 10) + 30  # 右肩ロール
        self.meridis_motion[57] = -int(np.sin(self.x) * 10) + 10    # 右肘ヨー
        self.meridis_motion[59] = -int(np.sin(self.x) * 45) - 45    # 右肘ピッチ
        self.meridis_motion[61] = -int(np.sin(self.x) * 5)          # 右股ヨー

        self.meridis_motion[33] = -int(np.sin(self.x) * 5)          # 左股ロール
        self.meridis_motion[35] = int(np.sin(self.x * 2) * 5) - 5      # 左股ピッチ
        self.meridis_motion[37] = -int(np.sin(self.x * 2) * 10) + 10    # 左膝ピッチ
        self.meridis_motion[39] = int(np.sin(self.x * 2) * 5) - 5      # 左足首ピッチ
        self.meridis_motion[41] = int(np.sin(self.x) * 5)           # 左足首ロール

        self.meridis_motion[63] = int(np.sin(self.x) * 5)           # 右股ロール
        self.meridis_motion[65] = -int(np.sin(self.x * 2) * 5) - 5  # 右股ピッチ
        self.meridis_motion[67] = int(np.sin(self.x * 2) * 10) + 10      # 右膝ピッチ
        self.meridis_motion[69] = -int(np.sin(self.x * 2) * 5) - 5  # 右足首ピッチ
        self.meridis_motion[71] = -int(np.sin(self.x) * 5)          # 右足首ロール
        
        return self.meridis_motion
    
    def send_motion(self):
        """計算したモーションデータをRedisに送信"""
        if not self.redis_transfer.is_connected:
            print("[Error] Redisに接続されていません")
            return False
        
        motion_data = self.calculate_motion()
        self.redis_transfer.set_data(data=motion_data)
        return True
    
    def update_x(self, increment=0.05):
        """サインカーブ用の変数xを更新
        
        Args:
            increment (float): xの増分値
        """
        self.x += increment
        if self.x > 2 * np.pi:
            self.x -= 2 * np.pi
    
    def run(self, duration=None, frequency=50):
        """ダンスモーションを継続的に送信
        
        Args:
            duration (float, optional): 実行時間（秒）。Noneの場合は無限ループ
            frequency (int): 送信頻度（Hz）
        """
        interval = 1.0 / frequency
        start_time = time.time()
        loop_count = 0
        
        print(f"ダンスモーション送信開始 (キー: {self.redis_transfer.redis_key}, 頻度: {frequency}Hz)")
        if duration:
            print(f"実行時間: {duration}秒")
        else:
            print("実行時間: 無制限 (Ctrl+Cで停止)")
        
        try:
            while True:
                loop_start = time.time()
                
                # モーション計算と送信
                if self.send_motion():
                    loop_count += 1
                    if loop_count % (frequency * 5) == 0:  # 5秒ごとに進捗表示
                        elapsed = time.time() - start_time
                        print(f"経過時間: {elapsed:.1f}秒, ループ回数: {loop_count}, x値: {self.x:.2f}")
                
                # xを更新
                self.update_x(increment=0.05)
                
                # 終了判定
                if duration and (time.time() - start_time) >= duration:
                    print(f"\n指定時間({duration}秒)が経過しました")
                    break
                
                # 次のループまで待機
                elapsed = time.time() - loop_start
                sleep_time = max(0, interval - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
        except KeyboardInterrupt:
            print("\n\nユーザーによって停止されました (Ctrl+C)")
        finally:
            print(f"総ループ回数: {loop_count}")
            print(f"総実行時間: {time.time() - start_time:.1f}秒")
            self.redis_transfer.close()


def main():
    """メイン関数"""
    parser = argparse.ArgumentParser(
        description='ロボットの全身ダンスモーションをRedisに送信するプログラム'
    )
    parser.add_argument('--host', default=REDIS_HOST, 
                       help=f'Redisサーバーのホスト名 (デフォルト: {REDIS_HOST})')
    parser.add_argument('--port', type=int, default=REDIS_PORT, 
                       help=f'Redisサーバーのポート番号 (デフォルト: {REDIS_PORT})')
    parser.add_argument('--key', default=REDIS_KEY, 
                       help=f'書き込むRedisキー (デフォルト: {REDIS_KEY})')
    parser.add_argument('--duration', type=float, default=None, 
                       help='実行時間（秒）。指定しない場合は無限ループ')
    parser.add_argument('--frequency', type=int, default=50, 
                       help='送信頻度（Hz）(デフォルト: 50Hz)')
    
    args = parser.parse_args()
    
    # DanceMotionインスタンスを作成
    dance = DanceMotion(host=args.host, port=args.port, redis_key=args.key)
    
    # Redis接続確認
    if not dance.redis_transfer.check_connection():
        print(f"[Error] Redisサーバー {args.host}:{args.port} に接続できませんでした")
        dance.redis_transfer.close()
        sys.exit(1)
    
    # ダンスモーション実行
    dance.run(duration=args.duration, frequency=args.frequency)


if __name__ == "__main__":
    main()
