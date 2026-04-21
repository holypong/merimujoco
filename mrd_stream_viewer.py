"""
mrd_stream_viewer.py - Redis から FPV フレームを受信して表示するツール
使い方: python mrd_stream_viewer.py [--redis redis.json] [--key meridis_frame_pub]
終了: ウィンドウを閉じるか Ctrl+C

Redis キーのデータ形式:
  JSON {"count": <int>, "frame": "<base64 JPEG>"}
"""
import argparse
import base64
import json
import os
import sys
import time

try:
    import cv2
except ImportError:
    print("[ERROR] opencv-python が見つかりません。pip install opencv-python でインストールしてください。")
    sys.exit(1)

try:
    import numpy as np
    import redis
except ImportError as e:
    print(f"[ERROR] 必要なパッケージが不足しています: {e}")
    print("pip install redis numpy でインストールしてください。")
    sys.exit(1)

# デフォルト設定
REDIS_HOST = "127.0.0.1"
REDIS_PORT = 6379
REDIS_KEY  = "meridis_frame_pub"
WINDOW_NAME = "FPV Viewer"

def load_redis_config(json_file: str):
    global REDIS_HOST, REDIS_PORT
    if not os.path.exists(json_file):
        print(f"[WARN] 設定ファイル '{json_file}' が見つかりません。デフォルト値を使用します。")
        return
    try:
        with open(json_file, 'r', encoding='utf-8') as f:
            config = json.load(f)
        if 'redis' in config:
            REDIS_HOST = config['redis'].get('host', REDIS_HOST)
            REDIS_PORT = config['redis'].get('port', REDIS_PORT)
        print(f"[INFO] Redis設定を読み込みました: {REDIS_HOST}:{REDIS_PORT}")
    except Exception as e:
        print(f"[WARN] 設定ファイルの読み込みに失敗しました: {e}")

def main():
    parser = argparse.ArgumentParser(description="Redis FPV ビューア")
    parser.add_argument('--redis', type=str, default='redis.json',
                        help='Redis設定JSONファイル (default: redis.json)')
    parser.add_argument('--key', type=str, default=REDIS_KEY,
                        help=f'受信するRedisキー (default: {REDIS_KEY})')
    parser.add_argument('--fps', type=float, default=30.0,
                        help='表示リフレッシュレート (default: 30)')
    args = parser.parse_args()

    load_redis_config(args.redis)
    redis_key = args.key
    interval  = 1.0 / max(args.fps, 1.0)

    # Redis接続
    try:
        client = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, decode_responses=True,
                             socket_connect_timeout=2.0, socket_timeout=2.0)
        client.ping()
        print(f"[INFO] Redis に接続しました: {REDIS_HOST}:{REDIS_PORT}")
    except Exception as e:
        print(f"[ERROR] Redis に接続できません: {e}")
        sys.exit(1)

    print(f"[INFO] キー '{redis_key}' を監視中... ウィンドウを閉じるか Ctrl+C で終了")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    last_raw = None      # 前回の生データ（変化検出用）
    last_frame = None    # デコード済みフレームキャッシュ
    last_count = None    # 前回の Meridim90 カウンタ値
    no_data_warned = False

    try:
        while True:
            loop_start = time.perf_counter()

            # ウィンドウが閉じられたか確認
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break

            raw = client.get(redis_key)

            if raw is None:
                if not no_data_warned:
                    print(f"[WARN] キー '{redis_key}' にデータがありません。--stream が有効なシミュレーターを起動してください。")
                    no_data_warned = True
            else:
                no_data_warned = False
                # データが更新された時だけデコード
                if raw != last_raw:
                    last_raw = raw
                    try:
                        payload = json.loads(raw)
                        count = payload.get("count")
                        frame_b64 = payload.get("frame")
                        if frame_b64 is None:
                            # 旧形式（Base64のみ）にもフォールバック
                            frame_b64 = raw
                            count = None
                        buf = base64.b64decode(frame_b64)
                        arr = np.frombuffer(buf, dtype=np.uint8)
                        decoded = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if decoded is not None:
                            last_frame = decoded
                            last_count = count
                    except (json.JSONDecodeError, Exception):
                        # JSON でなければ旧形式として処理
                        try:
                            buf = base64.b64decode(raw)
                            arr = np.frombuffer(buf, dtype=np.uint8)
                            decoded = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                            if decoded is not None:
                                last_frame = decoded
                                last_count = None
                        except Exception as e:
                            print(f"[WARN] フレームのデコードに失敗しました: {e}")

            # 毎ループ imshow を呼んでウィンドウを維持する
            if last_frame is not None:
                disp = last_frame.copy()
                if last_count is not None:
                    cv2.putText(disp, f"cnt:{last_count}", (4, 16),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow(WINDOW_NAME, disp)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

            # 残り時間だけスリープ
            elapsed = time.perf_counter() - loop_start
            sleep_t = interval - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C で終了します。")
    finally:
        cv2.destroyAllWindows()
        print("[INFO] ビューアを終了しました。")

if __name__ == "__main__":
    main()
