#!/usr/bin/env python3
"""
UDP受信テストプログラム（全ボディ座標対応版）
ポート27000で2つのロボットの全ボディ座標を受信して表示

データフォーマット:
  Header: [robot_id(1byte), num_bodies(1byte)]
  Body Data: [x, y, z, roll, pitch, yaw] * num_bodies (6 floats per body = 24 bytes/body)
"""

import socket
import struct
import sys

UDP_IP = "127.0.0.1"
UDP_PORT = 27000

# ロボットID定義
ROBOT_ID_MAIN = 1
ROBOT_ID_ENEMY = 2

# ロボット名のマッピング
ROBOT_NAMES = {
    ROBOT_ID_MAIN: "Main Robot",
    ROBOT_ID_ENEMY: "Enemy Robot"
}

# 受信したロボットデータを保存
robot_data = {
    ROBOT_ID_MAIN: None,
    ROBOT_ID_ENEMY: None
}

# UDPソケットを作成
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"[UDP Receiver] Listening on {UDP_IP}:{UDP_PORT}")
print("Waiting for robot body data...")
print("=" * 100)

def parse_robot_packet(data):
    """
    ロボットデータパケットを解析
    Returns: (robot_id, num_bodies, body_poses)
             body_poses = [(x, y, z, roll, pitch, yaw), ...]
    """
    if len(data) < 2:
        return None, None, None
    
    # ヘッダー解析
    robot_id, num_bodies = struct.unpack('BB', data[0:2])
    
    # ボディデータのサイズをチェック
    expected_size = 2 + (num_bodies * 6 * 4)  # header(2) + bodies(num_bodies * 6 floats * 4 bytes)
    if len(data) != expected_size:
        print(f"\n[Warning] Expected {expected_size} bytes, got {len(data)} bytes")
        return None, None, None
    
    # ボディデータを解析
    body_data_bytes = data[2:]
    floats = struct.unpack(f'{num_bodies * 6}f', body_data_bytes)
    
    # 6つずつのグループに分割 (x, y, z, roll, pitch, yaw)
    body_poses = []
    for i in range(num_bodies):
        start = i * 6
        pose = floats[start:start+6]
        body_poses.append(pose)
    
    return robot_id, num_bodies, body_poses


def display_robot_data():
    """受信したロボットデータを表示"""
    # 画面クリア（オプション）
    # print("\033[2J\033[H", end='')
    
    print("\r", end='')
    output = []
    
    for robot_id in [ROBOT_ID_MAIN, ROBOT_ID_ENEMY]:
        data = robot_data[robot_id]
        robot_name = ROBOT_NAMES.get(robot_id, f"Robot {robot_id}")
        
        if data is None:
            output.append(f"[{robot_name}] No data")
        else:
            num_bodies, body_poses = data
            # ベースボディ（最初のボディ）の情報を表示
            if len(body_poses) > 0:
                base = body_poses[0]
                x, y, z, roll, pitch, yaw = base
                output.append(f"[{robot_name}] Bodies:{num_bodies:2d} | "
                            f"Pos:({x:6.3f},{y:6.3f},{z:6.3f}) "
                            f"RPY:({roll:6.1f},{pitch:6.1f},{yaw:6.1f})")
    
    print(" | ".join(output), end='', flush=True)


def display_detailed_robot_data():
    """受信したロボットデータを詳細表示"""
    print("\n" + "=" * 100)
    
    for robot_id in [ROBOT_ID_MAIN, ROBOT_ID_ENEMY]:
        data = robot_data[robot_id]
        robot_name = ROBOT_NAMES.get(robot_id, f"Robot {robot_id}")
        
        print(f"\n[{robot_name}]")
        if data is None:
            print("  No data received")
        else:
            num_bodies, body_poses = data
            print(f"  Total bodies: {num_bodies}")
            for i, pose in enumerate(body_poses[:5]):  # 最初の5ボディのみ表示
                x, y, z, roll, pitch, yaw = pose
                print(f"    Body {i:2d}: Pos=({x:7.3f}, {y:7.3f}, {z:7.3f}) "
                      f"RPY=({roll:7.2f}, {pitch:7.2f}, {yaw:7.2f})")
            if num_bodies > 5:
                print(f"    ... and {num_bodies - 5} more bodies")
    
    print("=" * 100)


packet_count = 0
last_detailed_display = 0

try:
    while True:
        # データ受信
        data, addr = sock.recvfrom(4096)  # バッファサイズを増やす
        
        # パケット解析
        robot_id, num_bodies, body_poses = parse_robot_packet(data)
        
        if robot_id is not None:
            # ロボットデータを更新
            robot_data[robot_id] = (num_bodies, body_poses)
            
            # 簡易表示（リアルタイム）
            display_robot_data()
            
            packet_count += 1
            
            # 100パケットごとに詳細表示
            if packet_count % 100 == 0:
                display_detailed_robot_data()
        
except KeyboardInterrupt:
    print("\n\n[UDP Receiver] Stopped by user")
    display_detailed_robot_data()  # 最終状態を表示
finally:
    sock.close()
    print("[UDP Receiver] Socket closed")
