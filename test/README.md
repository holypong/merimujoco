# testディレクトリプログラム概要

このディレクトリには、MuJoCoシミュレーション、URDF処理、UDP通信、Redis連携などのテストプログラムが含まれています。各プログラムの概要を以下に記載します。

## プログラム一覧

### MuJoCo基本テスト
- **`test_mujoco.py`** - MuJoCoの基本的な動作確認プログラム
  - URDFモデル（scene.xml）を読み込んでMuJoCoビューアを起動
  - 物理シミュレーションの基本動作確認用

- **`test_urdf.py`** - URDFファイルのMuJoCo変換テスト
  - URDFファイルからMuJoCoモデルを作成して表示
  - カメラ設定やレンダリング処理のテスト

- **`test_urdf_control.py`** - URDF関節制御テスト
  - URDFからMuJoCoモデルを作成し、関節にモータを追加
  - 制御可能なロボットモデルの生成テスト

### UDP通信テスト
- **`test_udp_receiver.py`** - UDP受信テストプログラム
  - ポート27000でロボットの全ボディ座標データを受信
  - 2つのロボット（Main/Enemy）のデータを解析・表示
  - データフォーマット: [robot_id, num_bodies, body_poses...]

- **`udp_vs_merimujoco_walk.py`** - UDP送信対応歩行シミュレーション
  - MuJoCoでロボット歩行シミュレーションを実行
  - 全ボディ座標をUDPでリアルタイム送信（ポート27000）
  - 2ロボット同時シミュレーション対応

### Redis連携テスト
- **`vs_merimujoco.py`** - Redis連携MuJoCoシミュレーション
  - Redis経由で外部制御システムと連携
  - IMUデータ算出・関節制御・リアルタイムデータ交換
  - merimujoco.pyのバリエーション版

### ユーティリティ
- **`stl_bbox.py`** - STLファイル解析ユーティリティ
  - STLファイル（ASCII/Binary両対応）の頂点データを抽出
  - 3Dモデルのバウンディングボックス計算用

## 主要機能の概要

### MuJoCoシミュレーション機能
- **物理演算**: 重力、摩擦、関節動力学のシミュレーション
- **IMU計算**: ロボットの姿勢・角速度・加速度の算出
- **関節制御**: Redis/UDP経由の角度指令値適用
- **リアルタイム表示**: MuJoCoビューアによる3D可視化

### 通信機能
- **Redis連携**: 制御データ・状態データの送受信
- **UDP通信**: ロボットボディ座標のリアルタイム送信
- **データフォーマット**: Meridim配列、ボディ座標データ

### ロボットモデル
- **URDF対応**: ROS標準のURDFファイル読み込み
- **多関節ロボット**: ヒューマノイド型ロボット（股関節・膝関節・足首等）
- **複数ロボット**: Main/Enemyロボットの同時シミュレーション

## 使用方法

### 基本的なテスト実行
```bash
# MuJoCo基本テスト
python test_mujoco.py

# URDF読み込みテスト
python test_urdf.py

# UDP受信テスト（別ターミナルで実行）
python test_udp_receiver.py

# UDP送信テスト
python udp_vs_merimujoco_walk.py
```

### Redis連携テスト
```bash
# Redisサーバーが起動している状態で実行
python vs_merimujoco.py
```

## 設定ファイル

- **Redis設定**: `../mujoco-redis.json`（Redis接続設定）
- **ロボットモデル**: `../urdf/scene.xml`（MuJoCoシミュレーションモデル）
- **URDFファイル**: `../urdf/*.urdf`（ロボット定義ファイル）

## 関連ドキュメント

- `README.udp.md` - UDP通信機能の詳細仕様
- `../README.md` - プロジェクト全体の概要
- `../UDP_README.md` - UDP通信プロトコルの仕様

## 注意事項

- Redis連携プログラムはRedisサーバーの起動が必要
- UDP通信はポート27000を使用
- MuJoCoビューアの起動にはGUI環境が必要
- 一部のプログラムは別ターミナルでの同時実行が必要