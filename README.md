# merimujoco

`merimujoco` is a lightweight MuJoCo-based simulator
designed to connect simulation, real robots, and AI agents
via Redis at real-time frequency.

## 背景

- 近年、AIの行動範囲はコンピュータの中だけでなく現実世界にも拡がりはじめました。
- とくに、身体を得て行動し現実世界に物理的に作用するようになったAIを、フィジカルAI(Physical AI)またはエンボディードAI(Embodied AI) と呼びます。
- ロボットを現実世界で動かす前に、何度でも試行を繰り返し、自分と周囲環境を壊すことなく、安全に検証できる、シミュレーション環境の価値が高まっています。
- **merimujoco**は、ロボットの動きをシミュレーションで確かめ、同じデータを使ってリアルロボットにもシームレスにつながる、そんな開発を目的としたツールです

## 目的

### 研究開発
- フィジカルAI・エンボディードAIの研究では、アルゴリズムを現実に近い条件で、どれだけ効率よく検証できるかが重要になっています。
- 実機のみを用いた検証は、準備やコスト、再現性の面で制約が多く、研究の試行回数そのものを制限してしまいます。
- ロボットの運動や制御を、現実の物理法則に基づいて何度も安全に試せるシミュレーション環境は、仮説検証と改良のサイクルを高速に回すための重要な基盤です。
- **merimujoco**は、MuJoCoによる物理シミュレーションと、共通のデータ構造に基づいて外部システムと連携するモジュールの**meridis**の組合せにより、シミュレーションとリアルロボットをシームレスに扱える研究開発環境を提供することを目的としています。

### エンジニアリング
- ロボット開発では、制御ロジックの実装、動作検証、パラメータ調整を何度も繰り返す必要がありますが、これらを人手で行うには時間と労力がかかります。
- 近年は、AIエージェントが制御指令や動作パターンを生成し、それを高速に検証・評価する開発スタイルが注目されています。
- AIエージェントによる開発を現実的にするためには、指令生成・実行・結果取得を自動化できる実行基盤が不可欠です。
- **merimujoco**は、MCPサーバを介してAIエージェントからの指令を受け取り、**meridis**による共通データ構造を通じて、シミュレーションおよびリアルロボットを同じ手順で制御・評価できる環境を提供します。
- これにより、AIエージェントが「考える → 試す → 評価する」というループを高速に回しながら、シミュレーションと実機の両方で開発・検証を進めることが可能になります。

## 概要

本リポジトリの**merimujoco**は、物理シミュレーションエンジン`MuJoCo`を使用したロボットシミュレーションシステムです。

高速なインメモリデータベース **Redis** の read/write キーを共通I/Fとして扱うことで、外部システムを差し替え可能にするための**meridis**モジュールを提供します。

![merimujoco](image/merimujoco.png)

## 主な機能

- **MuJoCo物理シミュレーション**
  高精度な物理演算を備えた3Dロボットシミュレーション環境を提供する

- **リアルタイム制御**  
  専用スレッドによる最大100Hzの高頻度でデータ処理する

- **Redis経由のデータ処理（meridisモジュール）**
  インメモリデータベースRedis経由でロボット制御データ・状態データを送受信する

  - **Sim2Real**<br>
  リアルロボットの制御システムと連携することで、シミュレーションロボットとリアルロボットの関節の動きを同期・遠隔制御する

  - **Real2Sim**<br>
  リアルロボットの制御システムと連携することで、リアルロボットの関節の動きをシミュレーションロボットで再現する

  - **動作生成プログラム**<br>
  論文研究をベースとした動作生成プログラムの演算結果を入力として、シミュレーション上で再生し動作検証する

  - **MCPサーバー（予告）**<br>
  MCPサーバーと連携することで、AIエージェントからの指令をトリガとして、シミュレーションロボットの制御データ・状態データを送受信できる

- **リセット機能**  
  与えるデータ先頭 data[0]=5556 とするとき、MuJoCoのシミュレーション環境をリセットし初期条件に戻すことができる

- **マルチプラットフォーム**
  本プログラムは、Linux/WSL/Windows11/MacOS で動作確認済です。

### MuJoCo の選定理由

数あるロボットシミュレータの中で **MuJoCo（Multi-Joint dynamics with Contact）** が広く選ばれています

- **高速・高精度な接触演算**: 多関節ロボットの接触力学を正確かつ高速に計算でき、歩行・把持など複雑なタスクに対応
- **研究コミュニティとの親和性**: DeepMind による OSS 化以降、強化学習・運動制御の研究論文で事実上の標準シミュレータとして採用が急増
- **軽量・高い拡張性**: CPU のみで動作する軽量設計でノートPCでも手軽に実行可能。さらに **NVIDIA Warp** などGPU並列化技術を活用することで数百倍の高速化を実現でき、大規模強化学習や並列シミュレーションにも対応
- **最新ロボティクスAIとの統合**: **NVIDIA Isaac Sim/Lab**、**Genesis** などの次世代ロボティクスプラットフォームが MuJoCo をコアエンジンとして採用しており、強化学習ライブラリとシームレスに連携可能。論文で発表された最新アルゴリズムを即座に実装・検証できる
- **優れたビルトインUI**: インタラクティブなビューワーが標準搭載され、マウスでの直感的なカメラ操作、スライダーによる関節角度調整、物理パラメータのリアルタイム表示など、デバッグと開発効率を大幅に向上させる機能が提供されている


## セットアップ

merimujoco は **Redis経由でデータ処理** を前提に設計されているため、最初に **meridisモジュール** のセットアップが必要です。

### ステップ1: meridisモジュールのセットアップ ⭐ **必須**

merimujoco の **Redis経由のデータ処理** を利用するために、事前に**meridisモジュール**をセットアップしてください：

[meridis マニュアル](https://github.com/holypong/meridis/blob/main/README.md)

**必要な作業:**
- ✅ Redis サーバーのインストールと起動確認
- ✅ meridisリポジトリのダウンロード
- ✅ 必要なPython依存パッケージのインストール
- ✅ Redisキー初期化
- ✅ ネットワーク設定の確認
---

### ステップ2: Merimujoco のインストール

meridis のセットアップが完了したら、merimujoco をインストールします。

#### 必要なパッケージのインストール

```bash
pip install mujoco numpy redis
```

---
## クイックスタート 🚀

- 本プロジェクトの核心は、インメモリデータベース Redisの read/write キーを共通I/F扱いにして Sim/Real/AI を同じような手順でシームレスにつなぐことです。

- セットアップ完了後、以下の手順で基本的な動作確認を行いましょう。

## このクイックスタートでできること

- 🔰 シミュレーションと外部システム連携を試したい人  
  → Step 1～2 を実施

- 🤖 リアルロボットとつなぎたい人  
  → Step 3～5 を実施

- 💡 AIエージェントを開発したい人（予告）
  → AIエージェントとMCPサーバを組合せることで、Aが生成した制御指令をそのままシミュレーションまたはリアルのロボットに渡すことができます。
  - `merimujoco.py`の設定ファイル `redis_mcp.json`を提供済
  - `meridis_manager.py`の設定ファイル `mgr_mcp2real.json`を提供済
  -  歩行モーションを生成するMCPサーバーのサンプルプログラムは2026年に公開予定

---
### 🎯 Step 1: 基本動作確認（シミュレーションのみ）

まず merimujoco 単体での動作を確認します：

```bash
# デフォルト設定で起動テスト
python merimujoco.py
```

**✅ 成功確認:**
![merimujoco](image/merimujoco_start.png)

- MuJoCoビューワーウィンドウが開いている
- シミュレーションロボットが表示されている  
- マウスのドラッグ操作でカメラ操作ができる
  - L-Button: 左右上下回転
  - R-Button: 左右上下移動
  - M-Button: 前後移動
- 左メニュー `Option`->`Font` 100% でメニューサイズを調整する
- →メニュー `Control`で任意の関節をL-Buttonドラッグ操作する

**⚠️ 重要：merimujoco 終了方法**  
**ウィンドウ右上の「×」ボタン、または左メニュー `File`->`Quit`で終了してください。**

---
### 🔗 Step 2: 動作生成プログラムとの連携

このステップでは、merimujoco と外部システムが、関節角度の指令（read）と状態（write）を共通I/F扱いで接続できることを確認します。
- `calc_dance_motion.py`というダンスの動作生成プログラムを用意しました。
- このプログラムを改造すれば、歩行モーションなど数値計算結果をシミュレーション上で検証できます。


```bash
# ターミナル1: シミュレーション起動
python merimujoco.py --redis redis-calc.json
```

```bash
# ターミナル2: 別ターミナルでダンスモーションを作成する
python calc_dance_motion.py
```

**✅ 成功確認:**
![merimujoco](image/merimujoco_dance.png)

- シミュレーションロボットがダンスを再生する

**⚠️ 重要：merimujoco 終了方法**  
**ウィンドウ右上の「×」ボタン、または左メニュー `File`->`Quit`で終了してください。**

**⚠️ 重要：merimujoco以外の 終了方法**  
**起動したターミナル内で、CTRL+Cで終了してください**


---
### 🤖 Step 3: シミュレーションとリアルロボットを同期

リアルロボットがある場合：シミュレーションロボットのダンスの動きをリアルロボットに同期させる。
シミュレータとリアルのデジタルツインを体験してください。

**⚠️ 重要：meridis_manager.py を実行するときは [meridis マニュアル](https://github.com/holypong/meridis/blob/main/README.md)をよく読んでください**
- meridis の インストールディレクトリ下でうごかしてください
- `network.json`のネットワーク設定を確認してください
- `mgr_sim2real.json`のネットワーク設定を確認してください

```bash
# ターミナル1: シミュレーション起動
python merimujoco.py --redis redis-calc.json
```

```bash
# ターミナル2: モーション生成
python meridis_motion_calc.py
```

```bash
# ターミナル3: ブリッジ(Sim2Real)
python meridis_manager.py --mgr mgr_sim2real.json
```

**✅ 成功確認:**
![merimujoco](image/merimujoco_sim2real.png)

- シミュレーションロボットがダンスを再生する
- 同じ動きがリアルロボットでも再現される

**⚠️ 重要：merimujoco 終了方法**  
**ウィンドウ右上の「×」ボタン、または左メニュー `File`->`Quit`で終了してください。**

**⚠️ 重要：merimujoco以外の 終了方法**  
**起動したターミナル内で、CTRL+Cで終了してください**

---
### 🤖 Step 4: シミュレーションからリアルロボットを操作

リアルロボットがある場合：シミュレーションロボットの関節操作をリアルロボットに同期させる。
要するに「MuJoCoの標準UIからリアルロボットを遠隔操作する」体験ができます。

```bash
# ターミナル1: シミュレーション起動
python merimujoco.py --redis redis-mgr-direct.json
```

```bash
# ターミナル2: ブリッジ(Sim2Real)
python meridis_manager.py --mgr mgr_sim2real.json
```

**✅ 成功確認:**
![merimujoco](image/merimujoco_sim2real_mujocoui.png)

- シミュレーションロボットが表示される
- メニュー `Control`で任意の関節をL-Buttonドラッグ操作する
- 同じ動きがリアルロボットでも再現される

**⚠️ 重要：merimujoco 終了方法**  
**ウィンドウ右上の「×」ボタン、または左メニュー `File`->`Quit`で終了してください。**

**⚠️ 重要：merimujoco以外の 終了方法**  
**起動したターミナル内で、CTRL+Cで終了してください**

---
### 🤖 Step 5: リアルロボットの動きをシミュレーション上で再現

リアルロボットがある場合：リアルロボットの関節の動きをシミュレーションロボットで再現できます。

```bash
# ターミナル1: シミュレーション起動
python merimujoco.py --redis redis-mgr.json
```

```bash
# ターミナル2: ブリッジ(Real2Sim)
python meridis_manager.py --mgr mgr_real2sim.json
```

**✅ 成功確認:**
![merimujoco](image/merimujoco_real2sim.png)

- シミュレーションロボットが表示される
- リアルロボットの各関節に触れて動かす
- 同じ動きがシミュレーションロボットでも再現される

**⚠️ 重要：merimujoco 終了方法**  
**ウィンドウ右上の「×」ボタン、または左メニュー `File`->`Quit`で終了してください。**

**⚠️ 重要：merimujoco以外の 終了方法**  
**起動したターミナル内で、CTRL+Cで終了してください**

---
## 使い方

### コマンド
```bash
# デフォルト設定で起動する場合
python merimujoco.py
```

**⚠️ 重要：merimujoco 終了方法**  
**ウィンドウ右上の「×」ボタン、または左メニュー `File`->`Quit`で終了してください。**

### コマンドオプション
- `--redis <ファイル名>`: Redis設定JSONファイルを指定（デフォルト: `redis.json`）


---
#### 設定ファイルの形式

Redis接続設定を JSON ファイルで管理します。
ファイルが存在しない場合は安全なデフォルト値（127.0.0.1:6379）を使用します。

```json
{
  "redis": {
    "host": "127.0.0.1",
    "port": 6379
  },
  "redis_keys": {
    "read": "meridis_mgr_pub",
    "write": "meridis_sim_pub"
  },
  "data_flow": {
    "redis_to_joint": false,
    "joint_to_redis": true
  }
}
```
##### 設定項目

- **redis**: Redisサーバーの接続情報
  - `host`: Redisサーバーのホスト名またはIPアドレス
  - `port`: Redisサーバーのポート番号
- **redis_keys**: データ交換用のRedisキー
  - `read`: 制御システムからの指令データを読み取るキー
  - `write`: シミュレーション状態データを書き込むキー
- **data_flow**: データフローの制御 **[試験中]**
  - `redis_to_joint`: Redisから受信した値をMuJoCoの関節にセット (デフォルト: `true`)
  - `joint_to_redis`: MuJoCoの関節角度をRedisに送信 (デフォルト: `false`)

##### 各設定ファイルの違い

本リポジトリには、用途別に6個のJSON設定ファイルが用意されています。すべてのファイルで redis 接続設定（host: 127.0.0.1, port: 6379）は共通ですが、`redis_keys`と`data_flow`が異なります。

| ファイル名 | read キー | write キー | redis_to_joint | joint_to_redis | 用途 |
|-----------|----------|-----------|----------------|----------------|------|
| [redis.json](redis.json) | `meridis_mgr_pub` | `meridis_sim_pub` | ❌ false | ✅ true | デフォルト設定 |
| [redis-mgr-direct.json](redis-mgr-direct.json) | `meridis_mgr_pub` | `meridis_sim_pub` | ❌ false | ✅ true | MuJoCo UI操作 |
| [redis-mgr.json](redis-mgr.json) | `meridis_mgr_pub` | `meridis_sim_pub` | ✅ true | ❌ false | Sim2Real/Real2Sim（リアル←→シミュレーション） |
| [redis-calc.json](redis-calc.json) | `meridis_calc_pub` | `meridis_sim_pub` | ✅ true | ✅ true | 動作生成プログラムとの連携（双方向） |
| [redis-console.json](redis-console.json) | `meridis_console_pub` | `meridis_sim_pub` | ✅ true | ❌ false | コンソール入力からの制御【予告】 |
| [redis-mcp.json](redis-mcp.json) | `meridis_mcp_pub` | `meridis_sim_pub` | ✅ true | ❌ false | MCPサーバーとの連携【予告】 |

---
## 技術詳細

### データフロー図

```mermaid
flowchart LR
  Controller[外部システム]
  Simulation[merimujoco.py]
  subgraph Redisサーバー
    ReadKey["【read キー】<br/>meridis_mgr_pub/<br>meridis_calc_pub/<br>meridis_console_pub/<br> meridis_mcp_pub"]
    WriteKey["【write キー】<br/>meridis_sim_pub"]
  end
  Controller -- 書き込み/送信 --> ReadKey
  ReadKey -- 読み出し/取得 --> Simulation
  Simulation -- 書き込み/送信 --> WriteKey
  WriteKey -- 読み出し/取得 --> Controller
```

#### 関節マッピング
##### joint_names[] と XMLファイルのjoint名
- **概要**: `merimujoco.py` の `joint_names` リストは、MuJoCoモデルのactuator順序に基づいてインデックス付けされた関節名を定義しています。
- **注意点**: 読み込む `roid1_mjcf.xml` のjoint名と `joint_names[]` が一致しない場合でも、MuJoCo の `data.ctrl` はモデルのactuator順序に基づいてインデックス付けされることから、`joint_names` リストの順序がXMLファイルのactuator順序と一致していれば、問題なく扱えます。
- **推奨**: 可読性のためには、`joint_names[]` リストの関節名をXMLファイルのjoint名と一致させることを推奨します。

```python
joint_names = [
    "c_chest", "c_head", "l_shoulder_pitch", "l_shoulder_roll", "l_elbow_yaw", "l_elbow_pitch",
    "r_shoulder_pitch", "r_shoulder_roll", "r_elbow_yaw", "r_elbow_pitch",
    "l_hip_yaw", "l_hip_roll", "l_thigh_pitch", "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
    "r_hip_yaw", "r_hip_roll", "r_thigh_pitch", "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll"
]
```

##### joint_to_meridis[] と meridis_sim_pub テーブル
- **概要**: `joint_to_meridis` 辞書は、各関節名をMeridisデータ配列のインデックスと乗数にマッピングします。
これにより、Redisから受信した関節角度データを適切に変換してMuJoCoの`data.ctrl`に適用できます。
- **構造**: 各エントリは `[インデックス, 乗数]` の形式です
  - インデックスはMeridis配列の位置
  - 乗数は符号反転などの調整
- **用途**: Redis経由のデータ交換で、外部制御システムの指令値をシミュレーション内の関節制御に変換します。

```python
joint_to_meridis = {
    # Base link
    "base_roll":        [12, 1],
    "base_pitch":       [13, 1],
    "base_yaw":         [14, 1],
    # Head
    "c_head":           [21, 1],
    # Left arm
    "l_shoulder_pitch": [23, 1],
    "l_shoulder_roll":  [25, 1],
    "l_elbow_yaw":      [27, 1],
    "l_elbow_pitch":    [29, 1],
    # Left leg
    "l_hip_yaw":        [31, 1],
    "l_hip_roll":       [33, 1],
    "l_thigh_pitch":    [35, 1],
    "l_knee_pitch":     [37, 1],
    "l_ankle_pitch":    [39, 1],
    "l_ankle_roll":     [41, 1],
    # chest
    "c_chest":          [51, 1],
    # Right arm
    "r_shoulder_pitch": [53, 1],
    "r_shoulder_roll":  [55,-1],
    "r_elbow_yaw":      [57,-1],
    "r_elbow_pitch":    [59, 1],
    # Right leg
    "r_hip_yaw":        [61,-1],
    "r_hip_roll":       [63,-1],
    "r_thigh_pitch":    [65, 1],
    "r_knee_pitch":     [67, 1],
    "r_ankle_pitch":    [69, 1],
    "r_ankle_roll":     [71,-1]
}
```
### 特殊機能

#### リセット機能
- **条件**: Redis経由で `data[0] == 5556` を受信
- **動作**: MuJoCoシミュレーション状態を初期化（mj_resetData）
- **用途**: 制御実験の初期化、異常状態からの復旧

---

## 用語集

### 基本用語

**MuJoCo（ムジョコ）**  
Multi-Joint dynamics with Contact の略。ロボットや物体の物理的な動きを高精度にシミュレーションするためのエンジン。DeepMindがオープンソース化し、現在は無料で利用可能。

**Redis（レディス）**  
Remote Dictionary Server の略。高速なインメモリ型データベース。プログラム間でデータを瞬時にやり取りするために使用。本プロジェクトでは、シミュレーションと外部システムの橋渡し役として機能。

**meridis（メリディス）**  
ロボットシミュレータ と Redis を連携させ、シミュレーションと実機ロボット、AIエージェント等の外部システムをシームレスに接続するための補助ツール群。MuJoCo以外に、Genesis AI、IsaacSim への適用実績がある。`#Meridian計画`にてholypongが開発。

**MCP（Model Context Protocol）**  
AIエージェントが外部ツールやデータソースと標準的な方法で連携するためのプロトコル。本プロジェクトでは、AIエージェントからロボット制御を実現するために活用（予定）。

### ロボット関連用語

**Sim2Real（シム・トゥ・リアル）**  
Simulation to Real の略。シミュレーション環境で作成・検証した動作を、実際のロボット（実機）に適用すること。開発効率と安全性の両立に有効。

**Real2Sim（リアル・トゥ・シム）**  
Real to Simulation の略。実際のロボットの動きやセンサーデータをシミュレーション環境に取り込むこと。実機の動作解析やデバッグに活用。

**関節（Joint）**  
ロボットの各可動部分。人間の肩、肘、膝などに相当。各関節には角度や可動範囲が設定される。

**アクチュエータ（Actuator）**  
関節を動かすための駆動装置。シミュレーションでは、制御指令を受けて関節を動かす仮想的な装置として機能。

**IMU（Inertial Measurement Unit）**  
慣性計測装置。ロボットの姿勢（傾き）、角速度、加速度を測定するセンサー。本プロジェクトではシミュレーション内で仮想的に算出。

### 技術用語

**インメモリデータベース**  
データをディスクではなくメモリ（RAM）上に保存するデータベース。読み書きが超高速だが、電源を切るとデータが消える。Redisはこのタイプ。

**JSON（JavaScript Object Notation）**  
設定情報やデータを記述するための軽量なテキスト形式。本プロジェクトではRedis接続設定などに使用（例：`redis.json`）。


**物理演算（Physics Simulation）**  
重力、摩擦、衝突などの物理法則をコンピューター上で計算し、リアルな動きを再現すること。MuJoCoの得意分野。

**ターミナル / コマンドライン**  
文字で命令を入力してプログラムを実行する画面。Windows の PowerShell、macOS/Linux の Terminal など。

### フィジカルAI関連用語

**フィジカルAI（Physical AI）**  
物理世界で行動できる身体性を持つAI。ロボットやドローンなど、現実空間で動作するAIシステムを指す。現実的な実装に寄っている。

**エンボディードAI（Embodied AI）**  
身体（Body）を持ち、環境と相互作用しながら学習・行動するAI。フィジカルAIとほぼ同義で使われることが多い。やや研究論文に寄っている。

**強化学習（Reinforcement Learning）**  
AIが試行錯誤を通じて最適な行動を学習する手法。シミュレーションで大量の試行を繰り返し、実機に適用することが多い。

**デジタルツイン（Digital Twin）**  
実世界の物体やシステムをデジタル空間に再現したもの。本プロジェクトでは、シミュレーションロボットと実機ロボットが同期して動く状態を指す。

**NVIDIA Isaac Sim / Isaac Lab（アイザック・シム / アイザック・ラボ）**  
NVIDIAが開発するロボット開発プラットフォーム。Isaac Simはリアルタイムレイトレーシングによるフォトリアリスティックなシミュレーション環境、Isaac Labは強化学習に特化したフレームワーク。内部でMuJoCoを物理エンジンとして利用可能で、GPU並列化により数千体のロボットを同時シミュレーションできる。

**Genesis（ジェネシス）**  
物理シミュレーションとAIロボティクスを統合した次世代プラットフォーム。MuJoCoをベースに、GPU加速による超高速シミュレーション（最大43万FPS）を実現。生成AI時代のロボット開発に最適化され、単一のPythonスクリプトで物理シミュレーション、レンダリング、学習を完結できる設計が特徴。

---

### さらに詳しく知りたい方へ

- **MuJoCo公式ドキュメント**: https://mujoco.readthedocs.io/
- **Redis公式サイト**: https://redis.io/
- **meridis詳細マニュアル**: https://github.com/holypong/meridis/blob/main/README.md
- **Model Context Protocol**: https://modelcontextprotocol.io/
- **NVIDIA Isaac Sim**: https://developer.nvidia.com/isaac-sim
- **NVIDIA Isaac Lab**: https://isaac-sim.github.io/IsaacLab/
- **Genesis**: https://genesis-world.readthedocs.io/
