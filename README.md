# Coppeliasim_Synchro

CoppeliaSim を使用したマルチドローン編隊飛行制御シミュレーションシステムです。6台のドローンエージェントがターゲット（リーダー機）を中心に、半円フォーメーションから円形フォーメーションへ遷移しながら追従する制御アルゴリズムを実装しています。

---

## 概要

本システムは、以下の制御フローで動作します。

1. **巡回フェーズ**: ターゲットとの距離が閾値（10m）より大きい間、各エージェントは正方形経路を反時計回りに巡回します。
2. **半円フォーメーション**: ターゲットとの距離が10m以下になると、エージェントは半円状に整列します。
3. **円形フォーメーションへの遷移**: 半円フォーメーションが安定したと判断されると、d_iパラメータを徐々に変化させて円形フォーメーションへ移行します。
4. **円形フォーメーション**: ターゲットを中心に半径 R[m] の円上を理想角速度 Ω[rad/s] で旋回します。300ステップごとに角速度・制御パラメータが切り替わります。

---

## ファイル構成

| ファイル名 | 説明 |
|---|---|
| `main.py` | エントリーポイント。シミュレーションのメインループを管理し、フォーメーション状態を制御する |
| `animation.py` | 各フレームの制御計算・位置更新・CSV保存を担当するAnimationクラス |
| `caluculation.py` | 制御入力 u_r（放射方向）・u_theta（接線方向）を計算する関数 |
| `various_calculation.py` | 角度・距離・角速度・座標変換などの数値計算ユーティリティ |
| `connect_Coppelia.py` | CoppeliaSim との ZMQ 通信（位置・速度の取得・設定）を管理するSimulationクラス |
| `patroll.py` | 巡回フェーズの移動制御（正方形経路の巡回）を担当するPatrollクラス |
| `parameter.py` | 全パラメータの定義と動的更新関数 |
| `DataStrage.py` | 制御誤差の時間積分値を保持するグローバルストレージ |
| `csv_save.py` | CSV保存・読み込みおよびグラフプロット機能 |
| `run_experiments.py` | config.yaml に基づいて複数パラメータ設定を自動実行するバッチ実験スクリプト |
| `config.yaml` | バッチ実験用のパラメータスイープ設定ファイル |

---

## 必要環境

- Python 3.x
- CoppeliaSim（ZMQ Remote API 対応バージョン）
- `coppeliasim_zmqremoteapi_client`
- `numpy`
- `matplotlib`
- `pyyaml`（バッチ実験使用時）

---

## 使い方

### 単独実行

```bash
python main.py
```

`parameter.py` に記述されたパラメータでシミュレーションが開始されます。CoppeliaSim が起動済みであることを確認してください。

### バッチ実験（複数パラメータ自動実行）

```bash
python run_experiments.py
```

または設定ファイルを指定する場合：

```bash
python run_experiments.py --config custom_config.yaml
```

`config.yaml` に定義されたパラメータの全組み合わせを順次実行します。

---

## 主要パラメータ（parameter.py）

### システム設定

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `use_mocap` | `False` | `True`: モーションキャプチャ使用、`False`: CoppeliaSim使用 |
| `save_csv` | `True` | CSVデータ保存の有無 |
| `num_agents` | `6` | エージェント数 |

### シミュレーション設定

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `frames` | `2400` | 総フレーム数 |
| `frame_time` | `0.05` | 1フレームの時間 [s]（= 20fps） |

### ターゲット設定

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `target_move` | `True` | ターゲットを移動させるか |
| `target_move_speed` | `0.1` | 目標位置への移動速度 [m/s] |
| `target_goal_x` | `-10` | 目標位置 X 座標 [m] |
| `target_goal_y` | `-10` | 目標位置 Y 座標 [m] |

### 制御パラメータ

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `R` | `4` | 理想フォーメーション半径 [m] |
| `Omega` | `0.5` | 理想角速度 [rad/s] |
| `l1` | `6` | 放射方向の制御ゲイン |
| `l2` | `2.5` | 接線方向の制御ゲイン |
| `agent_speed_max` | `6` | エージェントの最大速度 [m/s] |
| `d_i` | `[π/3] × 6` | 各エージェントの理想相対角距離 [rad] |

---

## 制御アルゴリズム

`caluculation.py` に実装されたスライディングモード制御に基づく制御入力：

```
u_r    = -ρ_i * ω_i² - η_i - l1 * sign(ρ_i - R + η_i)
u_theta = (ω_i + Ω + f_i) * η_i + z_i * ρ_i + l2 * sign(f_i + Ω - ω_i)
```

- **u_r**: ターゲットとの距離を理想半径 R に収束させる放射方向速度
- **u_theta**: 各エージェント間の角距離を均等化する接線方向速度
- **f_i**: 隣接エージェントとの角距離誤差から計算される補正項
- **z_i**: 隣接エージェントの角速度差分から計算される項

---

## 出力ファイル

### CSV（`save_csv: True` の場合）

`CSV/YYYY-MM-DD-HH-MM-SS/` フォルダに以下のファイルが保存されます。

| ファイル名 | 内容 |
|---|---|
| `name[ro_i].csv` | 各エージェントのターゲットとの距離 |
| `name[theta].csv` | 各エージェントの角度 |
| `name[alpha_i].csv` | 前方隣接エージェントとの角距離 |
| `name[alpha_i_minus].csv` | 後方隣接エージェントとの角距離 |
| `name[omega_i].csv` | 各エージェントの角速度（生値） |
| `name[eta].csv` | 距離の時間微分 |
| `name[e_i_1].csv` | 距離誤差の時間積分 |
| `name[e_i_2].csv` | 角速度誤差の時間積分 |
| `name[fi].csv` | 角距離誤差補正項 |
| `name[target_error].csv` | ターゲットのドローンとボールの位置誤差 |
| `name[agents_error_norm].csv` | 各エージェントの位置誤差ノルム |
| `name[agents_error_detail].csv` | 各エージェントの位置誤差（XYZ成分） |

### SVGグラフ

シミュレーション終了後、`SVG/` フォルダに全データのプロット画像が保存されます。

---

## 収束が確認されているパラメータ設定

`うまく収束する値調整.txt` より（理想相対距離 R=4 固定）：

- **目標角速度 0.2 rad/s**: `agent_speed_max=8.5`, `l2=2.5`, `omega_history_size=1`
- **目標角速度 0.15 rad/s**: `agent_speed_max=7`, `l2=1.5`, `omega_history_size=1`

---

## 注意事項

- モーションキャプチャ対応（`use_mocap: True`）は未実装です（`connect_Coppelia.py` の `_get_position_from_mocap` / `_get_velocity_from_mocap` を実装する必要があります）。
- CoppeliaSim のシーンには `Quadcopter[0]`（ターゲット）と `Quadcopter[1]`～`Quadcopter[6]`（エージェント）が配置されている必要があります。
- `DataStrage.py` のグローバル変数はシミュレーション間でリセットされないため、`run_experiments.py` でバッチ実行する際は自動的にモジュールが再読み込みされます。
