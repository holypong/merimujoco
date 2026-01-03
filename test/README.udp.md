# UDP送信機能の使い方（全ボディ座標版）

## 概要
`vs_merimujoco_walk_udp`は2つのヒューマノイドロボットの全ボディ座標（位置と姿勢）をリアルタイムでUDP 27000ポートに送信します。

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
python vs_merimujoco_walk_udp
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


# 実装

# 全ボディ座標UDP送信機能 - 実装完了

## 実装内容

### データ設計（最小ヘッダー）

**パケット構造:**
```
┌─────────────────────────────────────┐
│ Header (2 bytes)                    │
│  - robot_id: 1 byte (1=Main, 2=Enemy)│
│  - num_bodies: 1 byte               │
├─────────────────────────────────────┤
│ Body 0: [X, Y, Z, Roll, Pitch, Yaw] │ 24 bytes
│ Body 1: [X, Y, Z, Roll, Pitch, Yaw] │ 24 bytes
│ Body 2: [X, Y, Z, Roll, Pitch, Yaw] │ 24 bytes
│ ...                                 │
│ Body N: [X, Y, Z, Roll, Pitch, Yaw] │ 24 bytes
└─────────────────────────────────────┘
```

**設計のポイント:**
- ヘッダーはわずか2バイト（最小限）
- ロボットID（1バイト）で2つのロボットを明確に区別
- ボディ数（1バイト）で最大255ボディまで対応
- 各ロボットのパケットは独立（片方のロスが他方に影響しない）

### 変更ファイル

#### 1. `vs_merimujoco_walk_udp`
**追加機能:**
- `get_body_pose()`: 任意のボディのXYZ+RPYを取得
- `send_robot_all_bodies_udp()`: 1ロボット分の全ボディデータをUDP送信
- 起動時に各ロボットのボディリストを自動生成
- メインループで2ロボット分を独立送信

**送信タイミング:**
- MuJoCoの各ステップ（約1000Hz @ timestep=0.001）
- Main Robot → Enemy Robot の順で2パケット送信

#### 2. `udp_receiver_test.py`
**新機能:**
- パケット解析: robot_idとnum_bodiesを自動認識
- 2ロボットのデータを区別して保存
- リアルタイム表示: 1行でコンパクトに表示
- 詳細表示: 100パケットごとに各ロボットの詳細情報

**表示モード:**
1. **リアルタイム（1行）**: 各ロボットのベースボディ座標
2. **詳細（100パケット毎）**: 各ロボットの最初の5ボディの詳細座標

#### 3. `UDP_README.md`
全面更新して新フォーマットに対応

## 使用例

### ターミナル1: シミュレーション起動
```bash
python vs_merimujoco_walk_udp
```

**出力例:**
```
[Setup] Main robot bodies: 20
[Setup] Enemy robot bodies: 20
[Setup] Total UDP packet size per robot: 482 bytes (main), 482 bytes (enemy)
[Main] Starting walking simulation. Press Ctrl+C to stop.
[Main] Sending robot poses to UDP 127.0.0.1:27000
```

### ターミナル2: データ受信
```bash
python udp_receiver_test.py
```

**出力例:**
```
[Main Robot] Bodies:20 | Pos:( 0.000, 0.000, 0.350) RPY:(  0.1, -2.3, 89.5) | [Enemy Robot] Bodies:20 | Pos:( 1.500, 0.000, 0.350) RPY:( -0.2,  1.8,-90.2)
====================================================================================================
[Main Robot]
  Total bodies: 20
    Body  0: Pos=(  0.000,   0.000,   0.350) RPY=(   0.12,  -2.34,  89.52)
    Body  1: Pos=(  0.050,   0.020,   0.280) RPY=(   1.23,  -1.45,  88.67)
    Body  2: Pos=( -0.030,   0.015,   0.250) RPY=(  -0.45,  -2.11,  90.33)
    ...
[Enemy Robot]
  Total bodies: 20
    Body  0: Pos=(  1.500,   0.000,   0.350) RPY=(  -0.21,   1.82, -90.18)
    ...
```

## パフォーマンス

### 通信量
- 20ボディ/ロボット: 482バイト/パケット × 2ロボット = 964バイト/フレーム
- 1000Hz送信: 約964KB/秒 = 7.7Mbps（ローカル通信なら余裕）

### 効率性
- ヘッダーオーバーヘッド: わずか2バイト（0.4%）
- バイナリ形式: テキストの約1/4のサイズ
- パケット独立: ロスの影響を最小化

## データの活用例

### 1. リアルタイム可視化
受信データを3Dビューアで表示

### 2. モーションキャプチャ
全ボディ座標を記録してモーション再生

### 3. 機械学習
ロボットの動作データを学習用データセットに

### 4. 対戦ゲーム
2ロボットの位置関係を判定して対戦ゲーム化

## まとめ

✅ 2つのロボットを明確に区別（robot_id）
✅ 全ボディ座標を送信（XYZ + RPY）
✅ 最小限のヘッダー設計（2バイト）
✅ 高速通信対応（約1000Hz）
✅ 受信側で詳細表示機能

全ボディ座標のリアルタイムUDP送信機能が完成しました！

