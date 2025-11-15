# UDP送信機能の使い方（全ボディ座標版）

## 概要
`udp_vs_merimujoco_walk.py`は2つのヒューマノイドロボットの全ボディ座標（位置と姿勢）をリアルタイムでUDP 27000ポートに送信します。

## データフォーマット

### パケット構造（最小ヘッダー設計）
各ロボットごとに独立したパケットを送信：

```
[Header: 2 bytes]
  - robot_id (1 byte): ロボット識別子 (1=Main, 2=Enemy)
  - num_bodies (1 byte): ボディ数

[Body Data: 24 bytes × num_bodies]
  各ボディごとに6個のfloat値 (各4バイト):
  - X position (float)
  - Y position (float)
  - Z position (float)
  - Roll angle (float, degrees)
  - Pitch angle (float, degrees)
  - Yaw angle (float, degrees)
```

### パケットサイズ例
- ヘッダー: 2バイト
- 20ボディの場合: 2 + (20 × 24) = 482バイト/パケット
- 2ロボット合計: 約964バイト/フレーム

### ロボットID
```python
ROBOT_ID_MAIN = 1   # メインロボット
ROBOT_ID_ENEMY = 2  # Enemyロボット
```

## 使用方法

### 1. シミュレーション実行（送信側）
```bash
cd /home/hori/mujoco
python udp_vs_merimujoco_walk.py
```

起動時に以下の情報が表示されます：
```
[Setup] Main robot bodies: 20
[Setup] Enemy robot bodies: 20
[Setup] Total UDP packet size per robot: 482 bytes (main), 482 bytes (enemy)
```

### 2. UDP受信テスト（別ターミナル）
```bash
cd /home/hori/mujoco
python udp_receiver_test.py
```

受信プログラムは以下を表示：
- リアルタイム表示: 両ロボットのベースボディ座標
- 100パケットごとに詳細表示: 各ロボットの全ボディ座標（最初の5ボディ）

### 3. カスタム送信先の設定
ファイル冒頭の設定を変更：
```python
UDP_IP = "127.0.0.1"    # 送信先IPアドレス
UDP_PORT = 27000        # 送信ポート
```

## Pythonでの受信例

```python
import socket
import struct

UDP_IP = "127.0.0.1"
UDP_PORT = 27000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(4096)
    
    # ヘッダー解析
    robot_id, num_bodies = struct.unpack('BB', data[0:2])
    
    # ボディデータ解析
    body_data = data[2:]
    floats = struct.unpack(f'{num_bodies * 6}f', body_data)
    
    # 各ボディの座標を取得
    for i in range(num_bodies):
        start = i * 6
        x, y, z = floats[start:start+3]
        roll, pitch, yaw = floats[start+3:start+6]
        
        print(f"Robot {robot_id}, Body {i}: "
              f"Pos=({x:.3f}, {y:.3f}, {z:.3f}), "
              f"RPY=({roll:.2f}, {pitch:.2f}, {yaw:.2f})")
```

## データの座標系
- **Position (XYZ)**: MuJoCoのワールド座標系での絶対位置 [m]
- **Orientation (RPY)**: ロール、ピッチ、ヨー角 [degrees]

## パフォーマンス
- 送信頻度: MuJoCoシミュレーションステップごと（約1000Hz @ timestep=0.001）
- 各ロボット独立送信: パケットロス時も他ロボットのデータは影響なし
- 最小ヘッダー設計: オーバーヘッドはわずか2バイト/パケット

## トラブルシューティング

### ポートが既に使用中
```bash
# ポート使用状況の確認
sudo lsof -i :27000

# プロセスの終了
kill <PID>
```

### データが受信できない
1. ファイアウォールの確認
2. IPアドレスの確認（ローカルなら127.0.0.1、リモートなら適切なIPを設定）
3. 送受信プログラムが同じポート番号を使用しているか確認
4. バッファサイズの確認（受信側で`recvfrom(4096)`以上を推奨）

### パケットサイズが大きすぎる
- UDPの最大パケットサイズは通常1500バイト程度（MTU制限）
- ボディ数が多い場合（60ボディ以上）は、データ圧縮や分割送信を検討

## 利点
1. **最小ヘッダー**: わずか2バイトで効率的
2. **ロボット識別**: robot_idで明確に区別
3. **可変長対応**: num_bodiesでボディ数を動的に送信
4. **独立送信**: 各ロボットのパケットが独立し、信頼性向上
