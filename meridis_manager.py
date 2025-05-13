#!/usr/bin/python3
# coding: UTF-8

# Meridis Manager - シンプルなデータ転送マネージャー
# Based on Meridian_console.py by Izumi Ninagawa & Meridian project

import sys
import numpy as np
import socket
from contextlib import closing
import struct
import threading
import time
import atexit
import signal
import redis
import redis_receiver
import redis_transfer

# 20250429 meridis_manager.py 新規作成
# 20250511 タイミング計測機能追加

# 定数
UDP_RESV_PORT = 22222       # 受信ポート
UDP_SEND_PORT = 22224       # 送信ポート
MSG_SIZE = 90               # Meridim配列の長さ
MSG_BUFF = MSG_SIZE * 2     # Meridim配列のバイト長さ
MSG_ERRS = MSG_SIZE - 2     # Meridim配列のエラーフラグの格納場所
MSG_CKSM = MSG_SIZE - 1     # Meridim配列のチェックサムの格納場所

# Redisサーバー設定
REDIS_HOST = "localhost"
#REDIS_HOST = "172.21.242.172"

REDIS_PORT = 6379
REDIS_KEY_READ = "meridis"          # 読み込むRedisキー (キーA)
REDIS_KEY_WRITE = "meridis2"        # 書き込むRedisキー (キーB)

# マイコンボードのIPアドレス
UDP_SEND_IP = "192.168.11.21"     # 送信先のESP32のIPアドレス（必要に応じて変更）

# MeridianConsoleクラス
class MeridianConsole:
    def __init__(self):
        # Meridim配列関連
        self.r_meridim = np.zeros(MSG_SIZE, dtype=np.int16)            # 受信用Meridim配列 int16->uint16
        self.s_meridim = np.zeros(MSG_SIZE, dtype=np.int16)            # 送信用Meridim配列 int16->uint16
        self.r_meridim_char = np.zeros(MSG_SIZE*2, dtype=np.uint8)     # 受信用バイト配列
        self.r_meridim_ushort = np.zeros(MSG_SIZE, dtype=np.uint16)    # 受信用unsigned short配列
        
        # エラー集計表示用変数
        self.loop_count = 1              # フレーム数のカウンタ
        self.frame_sync_s = 0            # 送信するframe_sync_r(0-59999)
        self.frame_sync_r_expect = 0     # 毎フレームカウントし, 受信カウントと比較(0-59999)
        self.frame_sync_r_resv = 0       # 今回受信したframe_sync_r
        self.frame_sync_r_last = 0       # 前回受信したframe_sync_r
        self.error_count_esp_to_pc = 0   # PCからESP32へのUDP送信でのエラー数
        self.error_count_pc_to_esp = 0   # ESP32からPCへのUDP送信でのエラー数
        self.error_count_pc_skip = 0     # PCが受信したデータがクロックカウントスキップしていたか
        
        # フラグ関連
        self.flag_udp_resv = True        # UDP受信の完了フラグ
        self.flag_servo_power = 1        # 全サーボのパワーオンオフフラグ
        
        # ロックの追加
        self.lock = threading.Lock()
        
        # Redisクライアント関連
        self.receiver = redis_receiver.RedisReceiver(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_READ)
        self.transfer = redis_transfer.RedisTransfer(host=REDIS_HOST, port=REDIS_PORT, redis_key=REDIS_KEY_WRITE)
        
        # 実行時間計測用
        self.start_time = time.time()
        
        # メッセージ表示用
        self.message = ""

        # タイミング計測用変数
        self.frame_times = []         # 直近のフレーム時間を保存するリスト
        self.max_frame_times = 1000   # 保存するフレーム時間の最大数
        self.last_frame_time = time.time()  # 前回のフレーム時間
        self.target_fps = 100.0       # 目標フレームレート
        self.target_frame_time = 1.0 / self.target_fps  # 理想的なフレーム時間
        self.timing_stats = {
            "min": 0.0,
            "max": 0.0,
            "avg": 0.0,
            "over_budget_count": 0,
            "total_frames": 0
        }

        # Joint mapping dictionary
        self.joint_to_meridis = {
            # Base link=IMU
            "base_roll":        12,
            "base_pitch":       13,
            "base_yaw":         14,
            # Left leg
            "l_hip_roll":       33,
            "l_hip_yaw":        34,
            "l_thigh_pitch":    35,
            "l_knee_pitch":     37,
            "l_ankle_pitch":    39,
            "l_ankle_roll":     41,
            # Right leg
            "r_hip_roll":       63,
            "r_hip_yaw":        64,
            "r_thigh_pitch":    65,
            "r_knee_pitch":     67,
            "r_ankle_pitch":    69,
            "r_ankle_roll":     71,
            # Remo
            "remo_button":      15,
            "remo_l_analog":    16,
            "remo_r_analog":    17,
            "remo_l2r2":        18,
        }

    def get_local_ip(self):
        """自身のIPアドレスを取得する"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            IP = s.getsockname()[0]
            s.close()
            return IP
        except Exception as e:
            return "Error: " + str(e)
            
    def calculate_checksum(self, data):
        """NumPyでチェックサムを計算（符号付きshortで返す）"""
        arr = np.array(data[:MSG_SIZE - 1], dtype=np.int16)
        checksum = ~np.sum(arr, dtype=np.int32) & 0xFFFF  # 16bitに収める
        if checksum >= 0x8000:
            checksum -= 0x10000  # int16に変換
        return checksum
        
    def update_timing_stats(self):
        """フレーム時間を計測し、統計情報を更新する"""
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        # フレーム時間を記録
        self.frame_times.append(frame_time)
        if len(self.frame_times) > self.max_frame_times:
            self.frame_times.pop(0)  # 古いデータを削除
        
        # 統計情報の更新
        self.timing_stats["total_frames"] += 1
        
        if frame_time > self.target_frame_time:
            self.timing_stats["over_budget_count"] += 1
        
        # 最小値、最大値、平均値を計算
        if self.frame_times:
            self.timing_stats["min"] = min(self.frame_times)
            self.timing_stats["max"] = max(self.frame_times)
            self.timing_stats["avg"] = sum(self.frame_times) / len(self.frame_times)
        
        return frame_time

# メインクラスのインスタンス生成
mrd = MeridianConsole()

def print_timing_histogram(frame_times, target_time):
    """フレーム時間のヒストグラムを表示する"""
    if not frame_times:
        return
    
    # ヒストグラムの範囲を設定
    min_time = min(frame_times)
    max_time = max(frame_times)
    range_ms = max(1, int((max_time - min_time) * 1000 * 1.1))
    
    # 10区間のヒストグラムを作成
    bins = 10
    histogram = [0] * bins
    bin_size = range_ms / bins
    
    # 各フレーム時間をビンに割り当て
    for t in frame_times:
        t_ms = t * 1000
        bin_index = min(bins - 1, int((t_ms - min_time * 1000) / bin_size))
        histogram[bin_index] += 1
    
    # ヒストグラムの表示
    print("\n----- Frame Time Histogram (ms) -----")
    print(f"Target: {target_time*1000:.2f} ms")
    for i in range(bins):
        start = min_time * 1000 + i * bin_size
        end = start + bin_size
        bar = "#" * int(histogram[i] / len(frame_times) * 50)
        print(f"{start:5.2f}-{end:5.2f}: {bar} {histogram[i]/len(frame_times)*100:.1f}%")
    print("---------------------------------------\n")

def fetch_redis_data():
    """Redisからデータを取得してMeridim配列に適用する"""
    try:
        # redis_receiver.pyを使用してデータを取得
        data = mrd.receiver.get_data(REDIS_KEY_READ)
        #print(f"[Debug] Redis data: {data}")

        if data is None or len(data) != MSG_SIZE:
            print(f"[Redis Error] Invalid data format from Redis key {REDIS_KEY_READ}")
            return False
        
        # サーボ位置データをMeridim配列に反映
        with mrd.lock:
            # 送信用Meridim配列を更新
            for i in range(21, 81, 2):
                mrd.s_meridim[i] = int(data[i] * 100)

            # サーボの動作状態を指定する
            if mrd.flag_servo_power > 0:
                for i in range(20, 80, 2):
                    mrd.s_meridim[i] = 1
            else:
                for i in range(20, 80, 2):
                    mrd.s_meridim[i] = 0

            #print(f"[Debug] Meridim data: {mrd.s_meridim}")

            # フレームカウントとチェックサムを更新
            mrd.s_meridim[0] = mrd.frame_sync_s
            mrd.s_meridim[MSG_CKSM] = mrd.calculate_checksum(mrd.s_meridim)
            
        return True
        
    except Exception as e:
        print(f"[Redis Error] Unexpected error in fetch_redis_data: {str(e)}")
        return False

def write_redis_data():
    """Meridim配列のデータをRedisに書き込む"""
    try:
        with mrd.lock:
                  
            # 受信データをfloatに変換してRedisに書き込む。
            data = [float(val) for val in mrd.r_meridim]
            for i in range(12, 15):
                data[i] = float(data[i] /100)
            for i in range(21, 81, 2):
                data[i] = float(data[i] /100)

            # for debug imu onvert r_meridim to redis
            #print(f"[Debug] Mrd RPY : {mrd.r_meridim[12], mrd.r_meridim[13], mrd.r_meridim[14]}")
            #print(f"[Debug] Rds RPY : {data[12], data[13], data[14]}")

            # for debug remo btn+Lxy convert r_meridim to redis
            #print(f"[Debug] Mrd btn+Lxy: {mrd.r_meridim[15]}, {mrd.r_meridim_char[33]}, {mrd.r_meridim_char[32]})")
            #print(f"[Debug] Rds btn+Lxy : {data[15]}, {data[16]}")

            # Remo
            CMD_VEL_GAIN = 1.0
            cmd_btn = float(mrd.r_meridim[15])
            lx = round(float(mrd.r_meridim_char[33]) / 127.0 * CMD_VEL_GAIN, 2)
            ly = round(float(mrd.r_meridim_char[32]) / 127.0 * CMD_VEL_GAIN, 2)
            rx = round(float(mrd.r_meridim_char[35]) / 127.0 * CMD_VEL_GAIN, 2)
            ry = round(float(mrd.r_meridim_char[34]) / 127.0 * CMD_VEL_GAIN, 2)

            print(f"[Debug] gene-sensor: {data[12]}, {data[13]}, {data[14]} + cmd_vel: {cmd_btn}, {lx}, {ly}, {rx}, {ry}")

        # redis_transfer.pyを使用してデータを書き込む
        mrd.transfer.set_data(REDIS_KEY_WRITE, data)
        
        return True
        
    except Exception as e:
        print(f"[Redis Error] Failed to write to Redis: {str(e)}")
        return False

def send_udp_data(sock):
    """UDPデータを送信する"""
    try:
        with mrd.lock:
            # Meridim配列をバイト列に変換
            s_bin_data = struct.pack('90h', *mrd.s_meridim)
            
            # フレームカウンタを更新（0-59999の範囲でループ）
            mrd.frame_sync_s = (mrd.frame_sync_s + 1) % 60000
            
        # UDPでデータを送信
        sock.sendto(s_bin_data, (UDP_SEND_IP, UDP_SEND_PORT))
        return True
    except Exception as e:
        print(f"[UDP Error] Failed to send data: {str(e)}")
        mrd.error_count_pc_to_esp += 1
        return False

def meridian_loop():
    """メインループ - UDP通信とデータ処理を行う"""
    # UDP用のsocket設定
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # UDP受信バッファを2MBに拡張 +Hori 20250510 Test
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)

    sock.bind((mrd.get_local_ip(), UDP_RESV_PORT))
    sock.settimeout(1.0)  # タイムアウト設定
    
    atexit.register(cleanup)  # クリーンアップ関数の登録
    
    print(f"Meridis Manager started. Listening on port {UDP_RESV_PORT}")
    print(f"Sending to {UDP_SEND_IP}:{UDP_SEND_PORT}")
    print(f"Redis keys: Read from {REDIS_KEY_READ}, Write to {REDIS_KEY_WRITE}")
    print(f"Target frame rate: {mrd.target_fps} Hz (frame time: {mrd.target_frame_time*1000:.2f} ms)")

    # 初期データ設定
    _r_bin_data_past = np.zeros(MSG_BUFF, dtype=np.int8)
    _r_bin_data = np.zeros(MSG_BUFF, dtype=np.int8)
    
    # フレーム時間計測開始
    mrd.last_frame_time = time.time()

    with closing(sock):
        while True:
            loop_start_time = time.time()  # ループ開始時間を記録
            
            # デバッグメッセージは100フレームごとに出力
            if mrd.loop_count % 100 == 0:
                print(f"[Loop] Frame: {mrd.loop_count}")
            
            mrd.loop_count += 1  # フレーム数カウントアップ

            # ------------------------------------------------------------------------
            # [ 1 ] : UDPデータの受信
            # ------------------------------------------------------------------------
            try:
                _r_bin_data, addr = sock.recvfrom(MSG_BUFF)  # UDPに受信したデータを転記
                
                # 前回と同じデータならスキップして次のデータを待つ
                if np.array_equal(_r_bin_data_past, _r_bin_data):
                    continue
                
                _r_bin_data_past = _r_bin_data  # 今回のデータを保存
                
                # 受信データを配列に変換
                with mrd.lock:
                    mrd.r_meridim = struct.unpack('90h', _r_bin_data)
                    mrd.r_meridim_ushort = struct.unpack('90H', _r_bin_data)
                    mrd.r_meridim_char = struct.unpack('180b', _r_bin_data)
                    
                    # フレーム同期確認とスキップカウント
                    mrd.frame_sync_r_resv = mrd.r_meridim[0]
                    if (mrd.frame_sync_r_resv - mrd.frame_sync_r_last) != 1 and (mrd.frame_sync_r_resv - mrd.frame_sync_r_last) != -59999:
                        mrd.error_count_pc_skip += 1
                    mrd.frame_sync_r_last = mrd.frame_sync_r_resv
                
                    #print(f"[Info] mrd.r_meridim: {mrd.r_meridim}")

                # チェックサムの確認
                received_checksum = mrd.r_meridim[MSG_CKSM]
                calculated_checksum = mrd.calculate_checksum(mrd.r_meridim)
                
                if received_checksum != calculated_checksum:
                    print(f"[Error] Checksum mismatch: received={received_checksum}, calculated={calculated_checksum}")
                    mrd.error_count_esp_to_pc += 1
                    continue
                
                # Redisへのデータ書き込み
                write_redis_data()

            except socket.timeout:
                print("[Info] Socket timeout. Waiting for next packet...")
                continue
            except Exception as e:
                print(f"[Error] Error in UDP reception: {str(e)}")
                continue
            
            # ------------------------------------------------------------------------
            # [ 2 ] : Redisからデータ取得と送信データの準備
            # ------------------------------------------------------------------------
            fetch_redis_data()
            
            # ------------------------------------------------------------------------
            # [ 3 ] : UDPデータの送信
            # ------------------------------------------------------------------------
            send_udp_data(sock)
            
            # ------------------------------------------------------------------------
            # [ 4 ] : タイミング計測と統計情報の更新
            # ------------------------------------------------------------------------
            frame_time = mrd.update_timing_stats()
            
            # 統計情報の表示（100フレームごと）
            if mrd.loop_count % 100 == 0:
                now = time.time() - mrd.start_time
                fps = mrd.loop_count / now
                over_budget_percent = (mrd.timing_stats["over_budget_count"] / mrd.timing_stats["total_frames"]) * 100 if mrd.timing_stats["total_frames"] > 0 else 0
                
                print(f"[Stats] Frame: {mrd.loop_count}, Actual FPS: {fps:.2f}, "
                      f"Frame times (ms) - Min: {mrd.timing_stats['min']*1000:.2f}, "
                      f"Avg: {mrd.timing_stats['avg']*1000:.2f}, "
                      f"Max: {mrd.timing_stats['max']*1000:.2f}, "
                      f"Over budget: {over_budget_percent:.2f}%")
                
                print(f"[Errors] PC-ESP: {mrd.error_count_pc_to_esp}, "
                      f"ESP-PC: {mrd.error_count_esp_to_pc}, "
                      f"Skips: {mrd.error_count_pc_skip}")
            
            # 10000フレームごとにヒストグラムを表示
            if mrd.loop_count % 10000 == 0:
                print_timing_histogram(mrd.frame_times, mrd.target_frame_time)
                
            # フレームレートを安定させるためのスリープ調整
            elapsed = time.time() - loop_start_time
            if elapsed < mrd.target_frame_time:
                time.sleep(mrd.target_frame_time - elapsed)
            elif mrd.loop_count % 1000 == 0:  # 1000フレームに1回、遅延警告を表示

# フレームレートを安定させるためのスリープ調整
                elapsed = time.time() - loop_start_time
                if elapsed < mrd.target_frame_time:
                    time.sleep(mrd.target_frame_time - elapsed)
                elif mrd.loop_count % 1000 == 0:  # 1000フレームに1回、遅延警告を表示
                    print(f"[Warning] Frame {mrd.loop_count} is behind schedule by {(elapsed - mrd.target_frame_time)*1000:.2f} ms")

def cleanup():
    """終了時の後片付け"""
    print("Meridis Manager resources released.")
    try:
        mrd.receiver.close()
        mrd.transfer.close()
    except:
        pass

def main():
    """メインスレッド - シグナルハンドリングとキープアライブ"""
    print(f"Meridis Manager started. This PC's IP address is {mrd.get_local_ip()}")
    
    # シグナルハンドラの設定
    def signal_handler(sig, frame):
        print("Keyboard interrupt detected, cleaning up...")
        cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 定期的なステータス表示
        while True:
            time.sleep(5)  # CPUリソースを節約
            
            # タイミング情報の概要を表示
            over_budget_percent = (mrd.timing_stats["over_budget_count"] / mrd.timing_stats["total_frames"]) * 100 if mrd.timing_stats["total_frames"] > 0 else 0
            now = time.time() - mrd.start_time
            avg_fps = mrd.loop_count / now if now > 0 else 0
            
            print(f"[Status] Manager running for {now:.1f} seconds. Frames: {mrd.loop_count}, Avg FPS: {avg_fps:.2f}")
            print(f"[Timing] Target: {mrd.target_fps} Hz ({mrd.target_frame_time*1000:.2f} ms/frame)")
            print(f"[Timing] Actual: Min={mrd.timing_stats['min']*1000:.2f}ms, Avg={mrd.timing_stats['avg']*1000:.2f}ms, Max={mrd.timing_stats['max']*1000:.2f}ms")
            print(f"[Timing] Frames exceeding budget: {over_budget_percent:.2f}% ({mrd.timing_stats['over_budget_count']} of {mrd.timing_stats['total_frames']})")
            
    except KeyboardInterrupt:
        print("Keyboard interrupt detected, cleaning up...")
        cleanup()
        sys.exit(0)

if __name__ == '__main__':
    # サブスレッドでUDP通信処理を実行
    thread1 = threading.Thread(target=meridian_loop)
    thread1.daemon = True  # メインスレッドが終了したら自動的に終了
    thread1.start()
    
    # メインスレッドでキープアライブと監視
    main()